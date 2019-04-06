#include "../include/pcp.hpp"

//PointCloud Processing Functions
#include "../include/ProcessingFunctions.h"

pcp::pcp(){
    this->init();
    
    // Create a ROS subscriber for the input point cloud
    const std::string point_topic = "/kinect2/qhd/points";
    ros::Subscriber sub = this->nh.subscribe (point_topic, 1, &pcp::processingCB);
}

void pcp::init(){

    std::string x_min;
    std::string x_max;
    std::string y_min;
    std::string y_max;
    std::string z_min;
    std::string z_max;

    this->nh.param<std::string>("x_min", x_min, "0.0");
    this->nh.param<std::string>("x_max", x_max, "4.0");
    this->nh.param<std::string>("y_min", y_min, "0.0");
    this->nh.param<std::string>("y_max", y_max, "6.0");
    this->nh.param<std::string>("z_min", z_min, "0.0");
    this->nh.param<std::string>("z_max", z_max, "0.5");
    this->nh.param<std::string>("distance_threshold", this->Params["distance_threshold"], "0.0");
    this->nh.param<std::string>("eps_angle", this->Params["eps_angle"], "0.0");
    this->nh.param<std::string>("publish_trans_cloud", this->Params["publish_trans_cloud"], "0");
    this->nh.param<std::string>("publish_filtered_cloud", this->Params["publish_filtered_cloud"], "0");
    this->transformCloudPublisher = this->nh.advertise<sensor_msgs::PointCloud2>("transformedCloud", 1);
    this->segmentedCloudPublisher = this->nh.advertise<sensor_msgs::PointCloud2>("segmentedCloud", 1);
    this->filteredCloudPublisher = this->nh.advertise<sensor_msgs::PointCloud2>("filteredCloud", 1);

    this->Params["x_min"] = x_min;
    this->Params["x_max"] = x_max;
    this->Params["y_min"] = y_min;
    this->Params["y_max"] = y_max;
    this->Params["z_min"] = z_min;
    this->Params["z_max"] = z_max;
}

void pcp::Publish(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, ros::Publisher publisher) {
    sensor_msgs::PointCloud2 publishCloud;

    pcl::toROSMsg(*cloud, publishCloud);
    publishCloud.header.frame_id = "world";
    publishCloud.header.stamp = ros::Time::now();
    publisher.publish(publishCloud);
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
    this->Publish(segmentedCloud, segmentedCloudPublisher);


}