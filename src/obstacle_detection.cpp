/* Program to detect obstacles on a field using a KinectV2, ROS Kinectic, and the Point Cloud Library
* Written by Trey Franklin, Fall 2017
*
* Credits:
* http://pointclouds.org/documentation/tutorials/statistical_outlier.php#statistical-outlier-removal
* http://pointclouds.org/documentation/tutorials/planar_segmentation.php#planar-segmentation
* http://pointclouds.org/documentation/tutorials/extract_indices.php#extract-indices
* http://docs.pointclouds.org/trunk/classpcl_1_1_extract_indices.html#details
*https://github.com/arunavanag591/tabletop_operation
*/

#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/transforms.h>

// includes for planar_segmentation
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

// passthrough filters
#include <pcl/filters/passthrough.h>

// Euclidian Cluster Extraction
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

// Convex hull estimation
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>

// source: wiki.ros.org/pcl/Tutorials

ros::Publisher downsample_publisher;
ros::Publisher statistical_outlier_publisher;
ros::Publisher indices_cloud_publisher;
ros::Publisher planar_cloud_publisher;
ros::Publisher filtered_cloud_publisher;
ros::Publisher pub6;
ros::Publisher pub7;
ros::Publisher pub8;

bool downsample;
bool passthrough;
float passthrough_lower_lim;
float passthrough_upper_lim;
const char *passthrough_filter_axis = "y";

float leaf_size;

int meanK;
float stdDevThres;

float distThres;

float cluster_tolerance;
int min_cluster_size;
int max_cluster_size;
float hull_alpha;


void downsample_cloud(const pcl::PCLPointCloud2ConstPtr& input_cloud, pcl::PCLPointCloud2& output_cloud, float leaf_size,
                      ros::Publisher &pub, bool publish)
{
	/* downsamples the input cloud using a VoxelGrid algorithm down to the specified leaf size
	* and then can publish this on the specified publisher based on the boolean value of publish.
	*/
    // can greatly help performance
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (input_cloud);
    sor.setLeafSize(leaf_size, leaf_size, leaf_size);
    sor.filter (output_cloud);

    // Publish the data
    if (publish)
    {
    	pub.publish(output_cloud);
    }
}

void passthrough_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, const char *axis, float lower_lim, float upper_lim)
{
  pcl::PointCloud<pcl::PointXYZ> xyz_intermediate;
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(input_cloud);
  pass.setFilterFieldName(axis);  // apparently this was the problem.
  pass.setFilterLimits(lower_lim, upper_lim);
  pass.filter(xyz_intermediate);

  input_cloud = xyz_intermediate.makeShared();
}

void remove_statistical_outliers(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                                 pcl::PointCloud<pcl::PointXYZ>& output_cloud, float meanK, float stdDevThres,
                                 ros::Publisher &pub, bool publish)
{
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor2;
  sor2.setInputCloud(input_cloud);
  sor2.setMeanK(meanK);
  sor2.setStddevMulThresh(stdDevThres);
  sor2.filter(output_cloud);

  if (publish)
  {
    pub.publish(output_cloud);
  }
}

void segment_plane_and_extract_indices(pcl::PointCloud<pcl::PointXYZ>::Ptr& planar_cloud,
                                       pcl::PointCloud<pcl::PointXYZ>::Ptr& indices_cloud,
                                       pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_f,
                                       pcl::ModelCoefficients::Ptr& coefficients, pcl::PointIndices::Ptr& inliers,
                                       pcl::PointIndices::Ptr& outliers,
                                       ros::Publisher &pub1, ros::Publisher &pub2, ros::Publisher &pub3,
                                       bool publish_planar, bool publish_indices, bool publish_filtered)
{
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);  //  TODO: Investigate how to do this properly
  seg.setMethodType (pcl::SAC_RANSAC);  // estimator to be used ???
  seg.setAxis(Eigen::Vector3f(1, 0, 0));
  seg.setEpsAngle(20);
  seg.setDistanceThreshold (distThres);  // how close must it be to considered an inlier?

  pcl::ExtractIndices<pcl::PointXYZ> extract (true);

  int i = 0, nr_points = (int) planar_cloud->points.size();
  // While 30% of the original cloud is still there

  while (planar_cloud->points.size () > 0.3 * nr_points)
  {
    seg.setInputCloud (planar_cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size() == 0)
    {
      std::cerr <<"Couldn't estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud(planar_cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*indices_cloud);

    extract.setNegative(true);
    extract.filter(*cloud_f);
    planar_cloud.swap(cloud_f);
    i++;
  }

  if (publish_planar)
  {
    pub1.publish(planar_cloud);
  }

  if (publish_indices)
  {
    pub2.publish(indices_cloud);
  }

  if (publish_filtered)
  {
    pub3.publish(cloud_f);
  }

}

void extract_euclidian_clusters(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                                std::vector<pcl::PointIndices>& cluster_indices, float cluster_tolerance,
                                int min_cluster_size, int max_cluster_size,
                                pcl::search::KdTree<pcl::PointXYZ>::Ptr& search_method)
{
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (cluster_tolerance);  // too high means multiple objects in cluster, too low mean multiple clusters per object
  ec.setMinClusterSize (min_cluster_size);  // min points in a cluster
  ec.setMaxClusterSize (max_cluster_size); // max points in a cluster
  ec.setSearchMethod (search_method);
  ec.setInputCloud (input_cloud);

  //extract the indices of the clusters
  ec.extract (cluster_indices);
}

void create_cluster_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_f,
                          std::vector<pcl::PointIndices> &cluster_indices,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr &clustered_cloud, int &j)
{
  j= 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
    {
      cloud_cluster->points.push_back (cloud_f->points[*pit]);
    }
    j += 1;

    // TODO: Fine-tune the parameters and determine concave vs convex hull
    // ---------- convex hull -----------------
    pcl::ConcaveHull<pcl::PointXYZ> hull;
    pcl::PointCloud<pcl::PointXYZ>::Ptr convexHull (new pcl::PointCloud<pcl::PointXYZ>);

    hull.setInputCloud(cloud_cluster);
    hull.setAlpha(hull_alpha);
    hull.reconstruct(*convexHull);

    *clustered_cloud += *convexHull;
  }
}


void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original and filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

  // Convert to PCLPointCloud2 data type from ROS sensor_msgs
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud (new pcl::PointCloud<pcl::PointXYZ>);  // new xyz point cloud to use in PCL
  pcl::PointCloud<pcl::PointXYZ> xyz_cloud_filtered;


  if (downsample)
  {
  	pcl::PCLPointCloud2 cloud_filtered;
    downsample_cloud(cloudPtr, cloud_filtered, leaf_size, downsample_publisher, true);
    pcl::fromPCLPointCloud2(cloud_filtered, *xyz_cloud);  // Convert from PCLPointCloud2 to PointCloud<PointXYZ>

  } else {

    // set input to raw (unfiltered) data
    pcl::fromPCLPointCloud2(*cloud, *xyz_cloud);  // Convert from PCLPointCloud2 to PointCloud<PointXYZ>
  }

  if (passthrough)
  {
    passthrough_filter(xyz_cloud, passthrough_filter_axis, passthrough_lower_lim, passthrough_upper_lim);
  }



  remove_statistical_outliers(xyz_cloud, xyz_cloud_filtered, meanK, stdDevThres, statistical_outlier_publisher, true);



  pcl::PointCloud<pcl::PointXYZ>::Ptr planar_cloud (xyz_cloud_filtered.makeShared());
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::PointIndices::Ptr outliers (new pcl::PointIndices);
  pcl::PointCloud<pcl::PointXYZ>::Ptr indices_cloud (xyz_cloud_filtered.makeShared());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (xyz_cloud_filtered.makeShared());

  segment_plane_and_extract_indices(planar_cloud, indices_cloud, cloud_f, coefficients, inliers, outliers,
                                    planar_cloud_publisher, indices_cloud_publisher, filtered_cloud_publisher,
                                    true, true, true);



  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_f);

  std::vector<pcl::PointIndices> cluster_indices;  // each entry is a vector of indices_cloud

  extract_euclidian_clusters(cloud_f, cluster_indices, cluster_tolerance, min_cluster_size, max_cluster_size, tree);


  pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  sensor_msgs::PointCloud2::Ptr clusters (new sensor_msgs::PointCloud2);

  int cluster_count;
  create_cluster_cloud(cloud_f, cluster_indices, clustered_cloud, cluster_count);


  // -------- Publish results --------------
  pcl::toROSMsg(*clustered_cloud, *clusters);  // this probably isn't necessary
  clusters->header.frame_id = "/kinect2_link";
  clusters->header.stamp = ros::Time::now();
  pub8.publish(clusters);
  std::cout << "clusters found: " << cluster_count << std::endl;

}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "obstacle_detection");
  ros::NodeHandle nh;

  downsample = true;  // this should really be set as an argument to the program...
  passthrough = true;  // do we wanna cut things out?

  passthrough_lower_lim = -1.0;
  passthrough_upper_lim = 0.7;


  leaf_size = 0.015;  // for VoxelGrid (default: 0.1)

  meanK = 15;  // how many neighbor points to examine? (default: 50)
  stdDevThres = 1.0;  // deviation multiplier (default: 1.0)

  distThres = 0.03;  // how close to be an inlier? (default: 0.01) IDEAL VAL CHANGES W/ ANGLE also directly determines size of obstacle detected.

  cluster_tolerance = 0.05;  // 5 cm
  min_cluster_size = 5;
  max_cluster_size = 2000;
  hull_alpha = 180.0;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/kinect2/qhd/points", 2, cloud_cb);

  // Create a ROS publisher for the output point cloud
  downsample_publisher = nh.advertise<sensor_msgs::PointCloud2> ("voxel_grid", 1);
  statistical_outlier_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("statistical_outliers", 1);
  indices_cloud_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("indices_cloud", 1000);
  planar_cloud_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("planar_cloud", 1000);
  filtered_cloud_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("cloud_f", 1000);
  pub6 = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("passthrough", 1000);
  pub7 = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("planar_cloud", 1000);
  pub8 = nh.advertise<pcl::PointCloud<pcl::PointXYZ> > ("euc_clusters", 5);

  // Spin
  ros::spin ();
}
