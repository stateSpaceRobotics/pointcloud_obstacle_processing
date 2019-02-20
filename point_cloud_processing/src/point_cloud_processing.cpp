//ROS Includes
#include <ros/ros.h>
#include <ros/console.h>

//PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/transforms.h>

//includes for plane segmentation
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

//Euclidian Cluster Extraction
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

//Transforms
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_datatypes.h>

// for transforms
//#include <tf2_ros/transform_listener.h>
//#include <geometry_msgs/TransformStamped.h>

//PointCloud Processing Functions
#include "../include/ProcessingFunctions.h"

//Parameter Structs
//#include "../include/Parameters.h"


//Parameters params;
ros::Publisher transformedCloudPublisher;
ros::Publisher filteredCloudPublisher;
ros::Publisher segmentedCloudPublisher;
tf2_ros::Buffer tfBuffer;

void Publish(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, ros::Publisher publisher)
{
    sensor_msgs::PointCloud2 publishCloud;

    pcl::toROSMsg(*cloud, publishCloud);
    publishCloud.header.frame_id = "world";
    publishCloud.header.stamp = ros::Time::now();
    publisher.publish(publishCloud);
}

//
void PointCloudReceivedCallback(const sensor_msgs::PointCloud2ConstPtr& receivedPointCloud)
{
    pcl::PCLPointCloud2 *initial = new pcl::PCLPointCloud2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr segmentedCloud(new pcl::PointCloud<pcl::PointXYZ>);

    //Convert from sensor_msgs pointcloud to pcl pointcloud
    pcl_conversions::toPCL(*receivedPointCloud, *initial);
    pcl::fromPCLPointCloud2(*initial, *newCloud);
    
    delete initial;    

    //Transform the pointcloud to the global coordinate frame
    ProcessingFunctions::Transform(newCloud, transformedCloud, tfBuffer.lookupTransform("world", "kinect2_link", ros::Time(0)));

    //Publish the transformed cloud
    Publish(transformedCloud, transformedCloudPublisher);

    //Filter out points that lie outside the bounds we're interested in
    ProcessingFunctions::Filter(transformedCloud, filteredCloud, 0.0, 3.6322, 0.0, 10, 0.0, 0.5);

    //Publish the filtered cloud
    Publish(filteredCloud, filteredCloudPublisher);

    //Segment away the ground plane
    ProcessingFunctions::SegmentPlane(filteredCloud, segmentedCloud);

    //Publish the segmented cloud
    Publish(segmentedCloud, segmentedCloudPublisher);
}

void LoadParameters(ros::NodeHandle)
{
    
}

//
int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "pointcloud_obstacle_detection");
    ros::NodeHandle nh ("~");
    ros::NodeHandle nh_pub;

    //Publishers for each stage of processing
    transformedCloudPublisher = nh.advertise<sensor_msgs::PointCloud2>("transformedCloud", 1);
    filteredCloudPublisher = nh.advertise<sensor_msgs::PointCloud2>("filteredCloud", 1);
    segmentedCloudPublisher = nh.advertise<sensor_msgs::PointCloud2>("segmentedCloud", 1);

    tf2_ros::TransformListener tfListener(tfBuffer);

    const char *point_topic = "/kinect2/qhd/points";

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe (point_topic, 1, PointCloudReceivedCallback);

    ros::spin ();
    return 0;
}