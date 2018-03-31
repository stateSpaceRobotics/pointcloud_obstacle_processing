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


int frames_to_accumulate;
int current_frame_count;
int count;

float x_min;  // -2.0
float y_min;
float z_min;  // 0.0
float x_max;  // 2.0
float y_max;
float z_max;  // 4.0


const char *point_topic = "/kinect2/sd/points";

pcl::PointCloud<pcl::PointXYZ> final_cloud;  // TODO: Should really pre-allocate enough memory initially for performance


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



  current_frame_count = 0;
  count = 0;

  ros::Subscriber sub = nh.subscribe (point_topic, 1, frame_callback);

  concatenated_publisher = nh.advertise<sensor_msgs::PointCloud2> ("accumulated_depth_1", 1);

  ros::spin();

}
