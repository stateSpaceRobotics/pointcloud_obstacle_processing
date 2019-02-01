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

namespace CloudProcessing
{
    //Transform the given point cloud using the given transform
    void Transform(const pcl::PointCloud<pcl::PointXYZ>::Ptr& inCloud, pcl::<pcl::PointXYZ>& outCloud, geometry_msgs::TransformStamped t)
    {
        tf::StampedTransform transform;
        tf::transformStampedMsgToTF(t, transform);
        pcl_ros::transformPointCloud(inCloud, outCloud, transform);
    }

    //Downsample the point cloud
    void Downsample(const pcl::PointCloud<pcl::PointXYZ>::Ptr& inCloud, pcl::<pcl::PointXYZ>& outCloud, float leafSize, ros::Publisher &publisher, bool publish = false)
    {
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud (input_cloud);
        sor.setLeafSize(leaf_size, leaf_size, leaf_size);
        sor.filter (output_cloud);

        if (!publish) return;
    }

    //Remove any statistical outliers
    void RemoveStatisticalOutliers(pcl::PointCloud<pcl::PointXYZ>::Ptr& inCloud, pcl::PointCloud<pcl::PointXYZ>& outCloud, float meanK, float stdDeviationThreshold, ros::Publisher &publisher, bool publish = false)
    {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(input_cloud);
        sor.setMeanK(meanK);
        sor.setStddevMulThresh(stdDevThres);
        sor.filter(output_cloud);

        if (!publish) return;
    }

    //Segment away the specified plane
    void SegmentPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr& inCloud, pcl::PointCloud<pcl::PointXYZ>& outCloud, float distanceThreshold, ros::Publisher &publisher)
    {
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PCDWriter writer;
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (100);
        seg.setDistanceThreshold (0.02);

        int nr_points = (int)inCloud->points.size();

        *outCloud = *inCloud;

        while (outCloud->points.size() > 0.3 * nr_points)
        {
            // Segment the largest planar component from the remaining cloud
            seg.setInputCloud(outCloud);
            seg.segment(*inliers, *coefficients);

            if (inliers->indices.size () == 0)
            {
                std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
                break;
            }

            // Extract the planar inliers from the input cloud
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(outCloud);
            extract.setIndices(inliers);
            extract.setNegative(false);

            // Get the points associated with the planar surface
            extract.filter(*cloud_plane);
            //std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;

            // Remove the planar inliers, extract the rest
            extract.setNegative(true);
            extract.filter(*cloud_f);
            *outCloud = *cloud_f;
        }
    }
}