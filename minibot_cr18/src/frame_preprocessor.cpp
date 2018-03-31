#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/point_cloud_conversion.h>

// includes for passthrough filter
#include <pcl/filters/passthrough.h>

// includes for Occupancy Grid message
#include <nav_msgs/OccupancyGrid.h>

ros::Publisher concatenated_publisher;
ros::Publisher occupancy_grid_publisher;

int frames_to_accumulate;
int current_frame_count;
int count;

float x_min;  // -2.0
float y_min;
float z_min;  // 0.0
float x_max;  // 2.0
float y_max;
float z_max;  // 4.0

float block_size; // this is the max obstacle size from the competition 0.3
float dev_percent;

int occupancy_grid_width;
int occupancy_grid_height;
int occupancy_grid_size;
long long *occupancy_grid_pt_counts;
long long *row_averages;


const char *point_topic = "/kinect2/qhd/points";

pcl::PointCloud<pcl::PointXYZ> final_cloud;  // TODO: Should really pre-allocate enough memory initially for performance


int get_occupancy_grid_location(float x, float y)
{
  int x_count = 0;
  int y_count = 0;

  while (x_min + (x_count + 1)*block_size < x)
  {
    x_count++;
  }

  while (z_max - (y_count + 1)*block_size > y)
  {
    y_count++;
  }
  int result = y_count*occupancy_grid_width + x_count;
  return y_count*occupancy_grid_width + x_count;


  /*
  int x_index;
  int y_index;
  x = x + 2;

  x_index = (int)(x / block_size);
  y_index = (int)(y / block_size);

  x_index = (x_index >= occupancy_grid_width) ? occupancy_grid_width - 1: x_index;
  y_index = (y_index >= occupancy_grid_height) ? occupancy_grid_height - 1 : y_index;

  return y_index * occupancy_grid_width + x_index;
  */
}


void passthrough_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, const char *axis, float lower_lim, float upper_lim)
{
  /* COPIED FROM obstacle_detection.cpp
   * Runs the input_cloud through a passthrough filter, which basically throws out all data points
   * above upper_lim and below lower_lim along the specified axis. Useful for reducing the number of
   * errenous obstacles detected, and to some extent reducing the processing overhead.
   *
   * Returns the concatenated cloud data in the input cloud.
   */
  pcl::PointCloud<pcl::PointXYZ> xyz_intermediate;
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(input_cloud);
  pass.setFilterFieldName(axis);  // apparently this was the problem.
  pass.setFilterLimits(lower_lim, upper_lim);
  pass.filter(xyz_intermediate);

  input_cloud = xyz_intermediate.makeShared();
}


void frame_callback( const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  pcl::PCLPointCloud2* initial_cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr initial_cloud_ptr(initial_cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr accumulator_input_cloud (new pcl::PointCloud<pcl::PointXYZ>);

  pcl_conversions::toPCL(*cloud_msg, *initial_cloud);
  pcl::fromPCLPointCloud2(*initial_cloud, *accumulator_input_cloud);

  /*
  sensor_msgs::PointCloud2::Ptr ros_cloud (new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*initial_cloud_ptr, *ros_cloud);
  ros_cloud->header.frame_id = "/kinect2_link";
  ros_cloud->header.stamp = ros::Time::now();
  */

  passthrough_filter(accumulator_input_cloud, "x", x_min, x_max);
  passthrough_filter(accumulator_input_cloud, "y", y_min, y_max);
  passthrough_filter(accumulator_input_cloud, "z", z_min, z_max);

  if (current_frame_count < frames_to_accumulate)  // don't have enough frames yet
  {
    final_cloud += *accumulator_input_cloud;
    current_frame_count += 1;

    //std::cout << "frame " << current_frame_count << " processed." << std::endl;
    //std::cout << "point count: " << point_count << std::endl;
    // for debugging purposes:
    //std::cout << "width: " << accumulator_input_cloud->width << std::endl;
    //std::cout << "height: " << accumulator_input_cloud->height << std::endl;

  } else {  // sufficient number of frames, go ahead and publish to the main obstacle program.

    sensor_msgs::PointCloud2 ros_cloud;

    int nan_count = 0;
    int point_count = 0;
    int occupied = 0;

    for (int i = 0 ; i < occupancy_grid_size; i++)
    {
      occupancy_grid_pt_counts[i] = 0;
    }
    for (int i = 0 ; i < occupancy_grid_height; i++)
    {
      row_averages[i] = 0;
    }

    for(int i = 0; i < static_cast<int> (final_cloud.points.size()); ++i)  // iterate through all points and place them in an occupancy grid
    {
      if (pcl_isnan(final_cloud.points[i].x)){
        continue;  // don't count NaN points.
      }
      occupancy_grid_pt_counts[get_occupancy_grid_location(final_cloud.points[i].x, final_cloud.points[i].z)]++;  // determine location in grid for point
      point_count++;
    }

    //std::cout << nan_count<< std::endl;

    //std::cout << "Occupancy_Grid size: " << occupancy_grid_size << std::endl;
    //std::cout << "Block size: " << block_size << " height: " << occupancy_grid_height << " width: " << occupancy_grid_width << std::endl;

    long long row_point_count = 0;

    for (int row_ind = 0; row_ind < occupancy_grid_height; row_ind++)  // compute averages for each row in point cloud.
    {
      row_point_count = 0;
      for (int col_ind = 0; col_ind < occupancy_grid_width; col_ind++)
      {
        row_point_count += occupancy_grid_pt_counts[row_ind*occupancy_grid_width + col_ind];
      }
      row_averages[row_ind] = row_point_count / occupancy_grid_width;
    }


    // create the occupancy grid here
    char *occupancy_grid_data = new char[occupancy_grid_size];
    int row_count = 0;

    for (int i = 0; i < occupancy_grid_size; i++)
    {

      long long current_row_avg = row_averages[(int)(i / occupancy_grid_width)];

      /*
      int percent = 0;

      if(current_row_avg > 0)
      {
        percent = (1 - (occupancy_grid_pt_counts[i] / current_row_avg) * 100);
      }

      if(percent < 0) percent = 0;
      occupancy_grid_data[i] = percent;
      */

      if (occupancy_grid_pt_counts[i] < current_row_avg * (1-dev_percent))
      {
        occupancy_grid_data[i] = 100;

      } else {

        occupancy_grid_data[i] = 0;
      }

      //occupancy_grid_data[i] = (occupancy_grid_pt_counts[i] < row_averages[(int)(i / occupancy_grid_width)] * (1-dev_percent)) ? 100 : 0;
    }

    nav_msgs::OccupancyGrid *occupancyGrid = new nav_msgs::OccupancyGrid();
    occupancyGrid->info.resolution = block_size;
    occupancyGrid->info.width = occupancy_grid_width;
    occupancyGrid->info.height = occupancy_grid_height;
    occupancyGrid->data = std::vector<int8_t>(occupancy_grid_data, occupancy_grid_data + occupancy_grid_size);

    occupancy_grid_publisher.publish(*occupancyGrid);

    //std::cout << "Percent occupied: " << ((float)occupied / occupancy_grid_size) * 100 << std::endl;  // how many cells are occupied? (debug only)

    pcl::toROSMsg(final_cloud, ros_cloud);

    ros_cloud.header.frame_id = "/kinect2_link";
    ros_cloud.header.stamp = ros::Time::now();

    concatenated_publisher.publish(ros_cloud);

    //std::cout << " total points: " << final_cloud.points.size() << std::endl;
    //std::cout << "----------------- frame count reset " << count << " -------------- " << std::endl;
    count++;
    final_cloud.clear();
    //final_cloud.reserve(frames_to_accumulate * topic_size);
    current_frame_count = 0;
  }

}

int main (int argc, char** argv)
{
  // Initialize ros
  ros::init (argc, argv, "frame_preprocessor");
  ros::NodeHandle nh("~");
  ros::NodeHandle nh_pub;

  nh.param("accumulate_count", frames_to_accumulate, 2);

  nh_pub.param("x_min", x_min, (float) -1.89);
  nh_pub.param("x_max", x_max, (float) 1.89);
  nh_pub.param("y_min", y_min, (float) -1.0);
  nh_pub.param("y_max", y_max, (float) 1.0);
  nh_pub.param("z_min", z_min, (float) 1.5);
  nh_pub.param("z_max", z_max, (float) 5.0);

  nh.param("block_size", block_size, (float) 0.15);
  nh.param("dev_percent", dev_percent, (float) 0.5);

  occupancy_grid_width = (int)ceil((fabs(x_min) + fabs(x_max)) / block_size);
  occupancy_grid_height = (int)ceil((fabs(z_max) - fabs(z_min)) / block_size);
  occupancy_grid_size = occupancy_grid_width * occupancy_grid_height;
  occupancy_grid_pt_counts = new long long[occupancy_grid_size];
  row_averages = new long long[occupancy_grid_height];

  current_frame_count = 0;
  count = 0;

  ros::Subscriber sub = nh.subscribe (point_topic, 1, frame_callback);

  concatenated_publisher = nh.advertise<sensor_msgs::PointCloud2> ("accumulated_depth_1", 1);
  occupancy_grid_publisher = nh.advertise<nav_msgs::OccupancyGrid> ("occupancy_grid", 1);

  ros::spin();

}
