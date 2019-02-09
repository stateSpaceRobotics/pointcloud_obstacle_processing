//ROS Includes
#include <ros/ros.h>
#include <ros/console.h>

// for transforms
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

//PointCloud Processing Functions
#include "../include/ProcessingFunctions.h"

//Parameter Structs
#include "../include/Parameters.h"

//
void PointCloudReceivedCallback(const sensor_msgs::PointCloud2ConstPtr& receivedPointCloud)
{
    pcl::PCLPointCloud2 *initial = new pcl::PCLPointCloud2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZ>);

    //Convert from sensor_msgs pointcloud to pcl pointcloud
    pcl_conversions::toPCL(*receivedPointCloud, *initial);
    pcl::fromPCLPointCloud2(*initial, *newCloud);

    //Transform the pointcloud to the global coordinate frame
    geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform("world", "kinect2_link", ros::Time(0));
    tf::StampedTransform transform;
    tf::transformStampedMsgToTF(transformStamped, transform);
    pcl_ros::transformPointCloud(*newCloud, *transformedCloud);

    sensor_msgs::PointCloud2 publishCloud;
    pcl::toROSMsg(transformedCloud, publishCloud);
    publishCloud.header.frame_id = "world";
    publishCloud.header.stamp = ros::Time::now();
    publisher.publish(transformedCloud);
}

void LoadParameters(ros::NodeHandle)
{
    
}

Parameters params;
ros::Publisher publisher;
tf2_ros::Buffer tfBuffer;

//
int Main(int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "pointcloud_obstacle_detection");
    ros::NodeHandle nh ("~");
    ros::NodeHandle nh_pub;
    publisher = nh.advertise<sensor_msgs::PointCloud2>("transformedCloud", 1);
    tf2_ros::TransformListener tfListener(tfBuffer);

    const char *point_topic = "/kinect2/qhd/points";
    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe (point_topic, 1, PointCloudReceivedCallback);

    ros::spin ();
    return 0;
}