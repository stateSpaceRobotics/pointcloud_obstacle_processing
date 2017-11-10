#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/point_cloud_conversion.h>

ros::Publisher concatenated_publisher;

int frames_to_accumulate;
int current_frame_count;

const char *point_topic = "/kinect2/qhd/points";

pcl::PointCloud<pcl::PointXYZ> final_cloud;  // TODO: Should really pre-allocate enough memory initially for performance

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
  if (current_frame_count < frames_to_accumulate)
  {
    final_cloud += *accumulator_input_cloud;
    current_frame_count += 1;
    std::cout << "frame " << current_frame_count << " processed." << std::endl;

  } else {

    sensor_msgs::PointCloud2 ros_cloud;

    pcl::toROSMsg(final_cloud, ros_cloud);

    ros_cloud.header.frame_id = "/kinect2_link";
    ros_cloud.header.stamp = ros::Time::now();

    concatenated_publisher.publish(ros_cloud);

    std::cout << " total points: " << final_cloud.points.size() << std::endl;
    std::cout << "----------------- frame count reset -------------- " << std::endl;
    final_cloud.clear();
    current_frame_count = 0;
  }

}

int main (int argc, char** argv)
{
  // Initialize ros
  ros::init (argc, argv, "frame_preprocessor");
  ros::NodeHandle nh;

  frames_to_accumulate = 4;
  current_frame_count = 0;

  //final_cloud = new pcl::PointCloud<pcl::PointXYZ>;

  ros::Subscriber sub = nh.subscribe (point_topic, 30, frame_callback);

  concatenated_publisher = nh.advertise<sensor_msgs::PointCloud2> ("accumulated_depth", 1);

  ros::spin();

}
