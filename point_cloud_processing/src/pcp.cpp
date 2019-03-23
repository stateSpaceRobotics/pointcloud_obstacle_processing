#include "../include/pcp.hpp"

//PointCloud Processing Functions
#include "../include/ProcessingFunctions.h"

pcp::pcp(){
    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = this->nh.subscribe ("/kinect2/qhd/points", 1, this->processingCB);

    this->init();
}

void pcp::init(){
    this->nh.param("x_min", this->Params["x_min"], "0.0");
    this->nh.param("x_max", this->Params["x_max"], "4.0");
    this->nh.param("y_min", this->Params["y_min"], "0.0");
    this->nh.param("y_max", this->Params["y_max"], "6.0");
    this->nh.param("z_min", this->Params["z_min"], "0.0");
    this->nh.param("z_max", this->Params["z_max"], "0.5");
    this->nh.param("distance_threshold", this->Params["distance_threshold"], "0.0");
    this->nh.param("eps_angle", this->Params["eps_angle"], "0.0");
    this->nh.param("publish_trans_cloud", this->Params["publish_trans_cloud"], "0");
    this->nh.param("publish_filtered_cloud", this->Params["publish_filtered_cloud"], "0");
}

void pcp::processingCB(const sensor_msgs::PointCloud2ConstPtr& receivedPointCloud){
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
    if((int)this->Params["publish_trans_cloud"]) Publish(transformedCloud, transformedCloudPublisher);

    //Filter out points that lie outside the bounds we're interested in
    ProcessingFunctions::Filter(transformedCloud, filteredCloud, 0.0, 3.6322, 0.0, 10, 0.0, 0.5);

    //Publish the filtered cloud
    if((int)this->Params["publish_filtered_cloud"]) Publish(filteredCloud, filteredCloudPublisher);

    //Segment away the ground plane
    ProcessingFunctions::SegmentPlane(filteredCloud, segmentedCloud);

    //Publish the segmented cloud
    Publish(segmentedCloud, segmentedCloudPublisher);


}