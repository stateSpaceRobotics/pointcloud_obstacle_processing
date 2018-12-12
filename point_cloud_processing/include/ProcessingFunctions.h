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
    void Transform()
    {

    }

    //Downsample the point cloud
    void Downsample(const pcl::PointCloud<pcl::PointXYZ>::Ptr& inCloud, pcl::<pcl::PointXYZ>& outCloud, float leafSize, ros::Publisher &publisher, bool publish)
    {

    }

    //Remove any statistical outliers
    void RemoveStatisticalOutliers(pcl::PointCloud<pcl::PointXYZ>::Ptr& inCloud, pcl::PointCloud<pcl::PointXYZ>& outCloud, float meanK, float stdDeviationThreshold, ros::Publisher &publisher, bool publish)
    {

    }

    //Segment away the specified plane
    void SegmentPlane()
    {

    }

    //Extract clusters of points
    void ExtractEuclideanClusters()
    {

    }
}