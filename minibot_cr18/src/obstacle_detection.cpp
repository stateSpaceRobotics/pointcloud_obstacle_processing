/* Program to detect obstacles on a field using a KinectV2, ROS Kinectic, and the Point Cloud Library
 * This is probably not the "one best solution", but it's a solution that works reasonably well
 * and it's hopefully written in a way that makes it possible to tweak a lot of things with relative
 * ease.
 * Written by Trey Franklin, Fall 2017
 *
 * Credits/Sources consulted:
 * http://wiki.ros.org/pcl/Tutorials
 * http://pointclouds.org/documentation/tutorials/statistical_outlier.php#statistical-outlier-removal
 * http://pointclouds.org/documentation/tutorials/planar_segmentation.php#planar-segmentation
 * http://docs.pointclouds.org/1.8.1/group__sample__consensus.html
 * http://pointclouds.org/documentation/tutorials/extract_indices.php#extract-indices
 * http://docs.pointclouds.org/trunk/classpcl_1_1_extract_indices.html#details
 * https://github.com/arunavanag591/tabletop_operation
 * https://github.com/ahestevenz/pcl-edge-detection/blob/master/src/pcl_ed_visualizer.cpp
 * http://www.pointclouds.org/blog/gsoc12/cchoi/index.php
*/

#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/transforms.h>
#include <NASA_ARMS/PointIndicesArray.h>
#include <math.h>

// includes for planar_segmentation
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

// passthrough_filter_enable filters
#include <pcl/filters/passthrough.h>

// Euclidian Cluster Extraction
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

// Convex hull estimation
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>

// edge detection
#include <pcl/features/organized_edge_detection.h>
#include <pcl/features/impl/organized_edge_detection.hpp>
#include <pcl/features/integral_image_normal.h>
#include <sensor_msgs/point_cloud_conversion.h>


// very bad, no good global variables init
ros::Publisher downsample_publisher;
ros::Publisher statistical_outlier_publisher;
ros::Publisher indices_cloud_publisher;
ros::Publisher planar_cloud_publisher;
ros::Publisher filtered_cloud_publisher;
ros::Publisher centroid_publisher;
ros::Publisher edge_cloud_publisher;
ros::Publisher filtered_edge_cloud_publisher;
ros::Publisher euc_cluster_publisher;
ros::Publisher hole_centroid_publisher;
ros::Publisher hole_cluster_publisher;

const char *point_topic = "/accumulated_depth_1";  // where are we getting the depth data from?
//const char *point_topic = "/kinect2/qhd/points";

bool downsample_input_data;
bool passthrough_filter_enable;
bool edge_detection;
float pt_upper_lim_y;
float pt_lower_lim_y;
float pt_upper_lim_x;
float pt_lower_lim_x;
float pt_upper_lim_z;
float pt_lower_lim_z;

float downsample_leaf_size;

int statistical_outlier_meanK;
float statistical_outlier_stdDevThres;

float plane_segment_dist_thres;
int plane_segment_angle;

float euc_cluster_tolerance;
int euc_min_cluster_size;
int euc_max_cluster_size;
float convex_hull_alpha;

float hole_euc_cluster_tolerance;
float edge_depth_tolerance;
int edge_max_neighbor_search;
int frames_accumulated;


void downsample_cloud(const pcl::PCLPointCloud2ConstPtr& input_cloud, pcl::PCLPointCloud2& output_cloud, float leaf_size,
                      ros::Publisher &pub, bool publish)
{
	/* downsamples the input cloud using a VoxelGrid algorithm down to the specified leaf size
	 * and then can publish this on the specified publisher based on the boolean value of publish.
	 * Useful for quickly improving the performance of the program, but also quickly decreases the
	 * quality of the obstacle detection.
	 *
	 * Option to publish the output cloud on a Point Cloud ROS topic, if enabled with publish
	*/

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
  /* Runs the input_cloud through a passthrough filter, which basically throws out all data points
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

void remove_statistical_outliers(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                                 pcl::PointCloud<pcl::PointXYZ>& output_cloud, float meanK, float stdDevThres,
                                 ros::Publisher &pub, bool publish)
{
  /* Helps filter the data by removing points that fall outside of a specified threshold.
   * Takes the input_cloud and then, for each point in the cloud, examines meanK points around it.
   * Any points that fall outside of the standard deviation threshold (stdDevThres) are removed.
   * Can publish the resulting cloud into a topic, if desired. Always returns the filtered cloud in output_cloud.
   *
   */
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor2;
  sor2.setInputCloud(input_cloud);
  sor2.setMeanK(meanK);
  sor2.setStddevMulThresh(stdDevThres);
  sor2.filter(output_cloud);

  if (publish)
  {
    sensor_msgs::PointCloud2 publish_cloud;
    pcl::toROSMsg(output_cloud, publish_cloud);
    pub.publish(publish_cloud);
  }
}

void segment_plane_and_extract_indices(pcl::PointCloud<pcl::PointXYZ>::Ptr& planar_cloud,
                                       pcl::PointCloud<pcl::PointXYZ>::Ptr& indices_cloud,
                                       pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_f,
                                       pcl::ModelCoefficients::Ptr& coefficients, pcl::PointIndices::Ptr& inliers,
                                       pcl::PointIndices::Ptr& outliers, const Eigen::Matrix<float, 3, 1> &axis,
                                       float dist_thresh, int eps_angle,
                                       ros::Publisher &pub1, ros::Publisher &pub2, ros::Publisher &pub3,
                                       bool publish_planar, bool publish_indices, bool publish_filtered)
{
  /* Takes the planar cloud as input to the program (and later output, too). Segments out any planes detected based
   * on the model type used (in this case SACMODEL_PERPENDICULAR_PLANE) based on the axis and the variance angle (eps_angle).
   * Uses the distance threshold to determine how closely the points must adhere to the ideal model to be considered
   * inliers (fitting the model). Very complex, and I truthfully don't understand it that well.
   *
   * planar_cloud is the input cloud as well as the output cloud that contains all points not considered in a plane
   * indices_cloud is the point cloud of the detected planes only
   * cloud_f is the same as planar_cloud, at the end. It has the points of things not considered to be a plane.
   *
   * Explanation of the various models possible for the segmentation:
   * http://docs.pointclouds.org/1.8.1/group__sample__consensus.html
   *
   */
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients (true);  // this is good, but I don't know why
  seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);  //  TODO: Investigate how to do this properly
  //seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);  // estimator to be used ???
  seg.setAxis(axis);  // axis to look for planes parallel to

  seg.setEpsAngle(eps_angle);  // degree offset from that axis
  seg.setDistanceThreshold (dist_thresh);  // how close must it be to considered an inlier?

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
    sensor_msgs::PointCloud2 publish_cloud1;
    pcl::toROSMsg(*planar_cloud, publish_cloud1);
    pub1.publish(publish_cloud1);
  }

  if (publish_indices)
  {
    sensor_msgs::PointCloud2 publish_cloud2;
    pcl::toROSMsg(*indices_cloud, publish_cloud2);
    pub2.publish(publish_cloud2);
  }

  if (publish_filtered)
  {
    sensor_msgs::PointCloud2 publish_cloud3;
    pcl::toROSMsg(*cloud_f, publish_cloud3);
    pub3.publish(publish_cloud3);
  }

}

void extract_euclidian_clusters(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                                std::vector<pcl::PointIndices>& cluster_indices, float cluster_tolerance,
                                int min_cluster_size, int max_cluster_size,
                                pcl::search::KdTree<pcl::PointXYZ>::Ptr& search_method)
{
  /* This extracts all the clusters of points that meet certain specifications, and returns a vector of groups of
   * indices (points) representing the clusters that met specification.
   *
   * Parameters:
   * cluster_tolerance: too small, and each object could have multiple clusters within it, but too big
   *                    and multiple objects could be within the same cluster. Experiment to determine the best value.
   * min_cluster_size: the smallest number of adjascent points to consider as a cluster
   * max_cluster_size: the largest number of points to consider as a cluster
   * search_method: what algorithm to use to search through the point cloud (default is KD tree)
   */

  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (cluster_tolerance);  // too high means multiple objects in cluster, too low mean multiple clusters per object
  ec.setMinClusterSize (min_cluster_size);  // min points in a cluster
  ec.setMaxClusterSize (max_cluster_size); // max points in a cluster
  ec.setSearchMethod (search_method);
  ec.setInputCloud (input_cloud);

  //extract the indices of the clusters
  ec.extract (cluster_indices);
}

float calculate_distance(float x1, float x2, float y1, float y2, float z1, float z2)
{
  float x_dist = (float)pow((x2 - x1), 2);
  float y_dist = (float)pow((y2 - y1), 2);
  float z_dist = (float)pow((z2 - z1), 2);

  return (float)sqrt(x_dist + y_dist + z_dist);
}

void create_cluster_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                          std::vector<pcl::PointIndices> &cluster_indices,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud, int &cluster_count,
                          NASA_ARMS::PointIndicesArray &centroids)
{
  /* Works with the extract_euclidian_clusters function to create a point cloud of the outlines of the euclidian cluster
   * points. The convex (or concave) hull part creates a polygon from the points and that's what's added to the point
   * cloud. This is useful because we don't care about all the points within the obstacle, we just want to know where
   * its borders are.
   *
   * Also calculates the centroids of each cluster of points and the maximum distance from a centroid to a border point
   * for use to pass in the obstacle locations to Stage.
   *
   * Parameters:
   * input_cloud: input point cloud used
   * cluster_indices: points in each cluster in the input_cloud
   * output_cloud: cloud containing the convex or concave hulls of each group of points
   * cluster_count: Integer telling us how many clusters (obstacles) have been found.
   * centroids: empty vector that will hold the centroid values and the associated distances.
   */

  cluster_count = 0;
  // iterate through the list of clusters
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    float x_sum = 0, y_sum = 0, z_sum = 0;
    int point_count = 0;

    // iterate through each point in the cluster, adding it to the point cloud
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
    {
      cloud_cluster->points.push_back (input_cloud->points[*pit]);
      x_sum += input_cloud->points[*pit].x;
      y_sum += input_cloud->points[*pit].y;
      z_sum += input_cloud->points[*pit].z;
      point_count += 1;
    }

    NASA_ARMS::PointWithRad point;
    point.x = x_sum/point_count;
    point.y = y_sum/point_count;
    point.z = z_sum/point_count;

    float max_dist = 0;

    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
    {
      float dist = calculate_distance(input_cloud->points[*pit].x, point.x, input_cloud->points[*pit].y, point.y,
                                      input_cloud->points[*pit].z, point.z);
      if (dist > max_dist)
      {
        max_dist = dist;
      }
    }

    point.r = max_dist;

    centroids.points.push_back(point);

    cluster_count += 1;

    // ---------- convex hull -----------------
    // create a 'hull', or outline of points for each cluster.
    pcl::ConvexHull<pcl::PointXYZ> hull;
    pcl::PointCloud<pcl::PointXYZ>::Ptr convexHull (new pcl::PointCloud<pcl::PointXYZ>);

    hull.setInputCloud(cloud_cluster);
    // hull.setAlpha(convex_hull_alpha);
    hull.reconstruct(*convexHull);

    *output_cloud += *convexHull;  // add these border points to the output cloud
  }
}

void detect_edges(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud,
                  float depth_dist, int neighbors, ros::Publisher &pub, bool publish)
{
  // TODO: Document!
  // TODO: parameterize!
  // TODO: understand what the hell this does

  pcl::PointCloud<pcl::Normal>::Ptr normal (new pcl::PointCloud<pcl::Normal>);
  /*
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(0.4);
  */

  if (frames_accumulated > 1)
  {
    input_cloud->resize(540*960*frames_accumulated);
    input_cloud->height = 540*sqrt(frames_accumulated);
    input_cloud->width = 960*sqrt(frames_accumulated);
    input_cloud->is_dense = false;

  }

  pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
  ne.setNormalSmoothingSize (10.0f);
  ne.setBorderPolicy (ne.BORDER_POLICY_MIRROR);


  ne.setInputCloud (input_cloud);
  ne.compute (*normal);

  pcl::OrganizedEdgeFromNormals<pcl::PointXYZ, pcl::Normal, pcl::Label> oed;
  //pcl::OrganizedEdgeBase<pcl::PointXYZ, pcl::Label> oed;

  oed.setInputNormals (normal);
  oed.setInputCloud (input_cloud);
  oed.setDepthDisconThreshold (depth_dist);  // *apparently* this makes it more sensitive.
  oed.setMaxSearchNeighbors (neighbors);  // TODO:: what even?
  oed.setEdgeType (oed.EDGELABEL_NAN_BOUNDARY | oed.EDGELABEL_OCCLUDING | oed.EDGELABEL_OCCLUDED | oed.EDGELABEL_HIGH_CURVATURE | oed.EDGELABEL_RGB_CANNY);

  pcl::PointCloud<pcl::Label> labels;
  std::vector<pcl::PointIndices> label_indices;
  oed.compute (labels, label_indices);

  pcl::PointCloud<pcl::PointXYZ>::Ptr occluding_edges (new pcl::PointCloud<pcl::PointXYZ>),
          occluded_edges (new pcl::PointCloud<pcl::PointXYZ>),
          nan_boundary_edges (new pcl::PointCloud<pcl::PointXYZ>),
          high_curvature_edges (new pcl::PointCloud<pcl::PointXYZ>),
          rgb_edges (new pcl::PointCloud<pcl::PointXYZ>);

  copyPointCloud (*input_cloud, label_indices[0].indices, *nan_boundary_edges);
  copyPointCloud (*input_cloud, label_indices[1].indices, *occluding_edges);
  copyPointCloud (*input_cloud, label_indices[2].indices, *occluded_edges);
  copyPointCloud (*input_cloud, label_indices[3].indices, *high_curvature_edges);
  copyPointCloud (*input_cloud, label_indices[4].indices, *rgb_edges);

  *output_cloud += *occluding_edges;
  //*output_cloud += *occluded_edges;
  //*output_cloud += *nan_boundary_edges;

  output_cloud->header.frame_id = "/kinect2_link";

  if (publish)
  {
    sensor_msgs::PointCloud2 publish_edge_cloud;
    pcl::toROSMsg(*output_cloud, publish_edge_cloud);
    pub.publish(publish_edge_cloud);
  }
}


void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  /* Called when a PointCloud is received from the Kinect.
   * Orchestrates the obstacle detection using a bunch of other functions.
   *
   */

  // Container for original and filtered data
  pcl::PCLPointCloud2* initial_cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr initial_cloud_ptr(initial_cloud);

  // Convert to PCLPointCloud2 data type from ROS sensor_msgs
  pcl_conversions::toPCL(*cloud_msg, *initial_cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr statistical_outlier_input_cloud (new pcl::PointCloud<pcl::PointXYZ>);  // new point cloud to use in PCL
  pcl::PointCloud<pcl::PointXYZ> statistical_outlier_output_cloud;


  // ----------- edge detection for hole detection ----------------------

  pcl::PointCloud<pcl::PointXYZ>::Ptr edge_cloud_output (new pcl::PointCloud<pcl::PointXYZ>);
  if (edge_detection) {
    pcl::fromPCLPointCloud2(*initial_cloud, *statistical_outlier_input_cloud);
    detect_edges(statistical_outlier_input_cloud, edge_cloud_output, edge_depth_tolerance, edge_max_neighbor_search,
                 edge_cloud_publisher, true);
  }



    // ---------------------- downsampling --------------------------------
  if (downsample_input_data)  // reduce the number of points we have to work with
  {
  	pcl::PCLPointCloud2 cloud_filtered;
    downsample_cloud(initial_cloud_ptr, cloud_filtered, downsample_leaf_size, downsample_publisher, true);
    pcl::fromPCLPointCloud2(cloud_filtered, *statistical_outlier_input_cloud);  // Convert from PCLPointCloud2 to PointCloud<PointXYZ>

  } else {

    // set input to raw (unfiltered) data
    pcl::fromPCLPointCloud2(*initial_cloud, *statistical_outlier_input_cloud);  // Convert from PCLPointCloud2 to PointCloud<PointXYZ>
  }

  // --------------------- passthrough filtering ------------------------

  if (passthrough_filter_enable)  // chop off points above and below certain values along a specified plane_axis
  {
    passthrough_filter(statistical_outlier_input_cloud, "y", pt_lower_lim_y, pt_upper_lim_y);  // limits up/down input size
    passthrough_filter(statistical_outlier_input_cloud, "x", pt_lower_lim_x, pt_upper_lim_x);  // limits left/right input size
    passthrough_filter(statistical_outlier_input_cloud, "z", pt_lower_lim_z, pt_upper_lim_z);  // limits forward/backward input size

    if (edge_detection)
    {
      passthrough_filter(edge_cloud_output, "y", pt_lower_lim_y, pt_upper_lim_y);  // limits up/down input size
      passthrough_filter(edge_cloud_output, "x", pt_lower_lim_x, pt_upper_lim_x);  // limits left/right input size
      passthrough_filter(edge_cloud_output, "z", pt_lower_lim_z, pt_upper_lim_z);  // limits forward/backward input size

      sensor_msgs::PointCloud2 published_edge_cloud;
      pcl::toROSMsg(*edge_cloud_output, published_edge_cloud);
      filtered_edge_cloud_publisher.publish(published_edge_cloud);
    }

  }


  // ------------------- statistical outlier removal ---------------------
  remove_statistical_outliers(statistical_outlier_input_cloud, statistical_outlier_output_cloud,
                              statistical_outlier_meanK, statistical_outlier_stdDevThres, statistical_outlier_publisher,
                              true);

  // ----------------- plane segmentation (floor removal) ----------------

  // Create the data structures necessary for segmenting the planes and extracting the indices remaining.
  pcl::PointCloud<pcl::PointXYZ>::Ptr planar_cloud_y (statistical_outlier_output_cloud.makeShared());
  pcl::ModelCoefficients::Ptr coefficients_y (new pcl::ModelCoefficients);

  const Eigen::Matrix<float, 3, 1> &plane_axis_y = Eigen::Vector3f(0, 1, 0);  // x, y, z

  pcl::PointIndices::Ptr inliers_y (new pcl::PointIndices);
  pcl::PointIndices::Ptr outliers_y (new pcl::PointIndices);

  pcl::PointCloud<pcl::PointXYZ>::Ptr indices_cloud_y (statistical_outlier_output_cloud.makeShared());
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_y (statistical_outlier_output_cloud.makeShared());

  // perform the planar segmentation
  segment_plane_and_extract_indices(planar_cloud_y, indices_cloud_y, filtered_cloud_y, coefficients_y, inliers_y,
                                    outliers_y, plane_axis_y, plane_segment_dist_thres, plane_segment_angle,
                                    planar_cloud_publisher, indices_cloud_publisher, filtered_cloud_publisher,
                                    true, true, true);


  // ------------------ euclidian cluster extraction ------------------

  // create data structures for euclidian cluster extraction.
  pcl::search::KdTree<pcl::PointXYZ>::Ptr euc_cluster_tree (new pcl::search::KdTree<pcl::PointXYZ>);
  euc_cluster_tree->setInputCloud (planar_cloud_y);
  std::vector<pcl::PointIndices> euc_cluster_indices;  // each entry is a vector of indices_cloud

  // extract the clusters of points that meet certain parameters in the point cloud
  extract_euclidian_clusters(planar_cloud_y, euc_cluster_indices, euc_cluster_tolerance, euc_min_cluster_size,
                             euc_max_cluster_size, euc_cluster_tree);

  // cluster extraction for the holes
  pcl::search::KdTree<pcl::PointXYZ>::Ptr hole_euc_cluster_tree (new pcl::search::KdTree<pcl::PointXYZ>);
  std::vector<pcl::PointIndices> hole_euc_cluster_indices;

  if (edge_detection)
  {
    hole_euc_cluster_tree->setInputCloud(edge_cloud_output);

    extract_euclidian_clusters(edge_cloud_output, hole_euc_cluster_indices, hole_euc_cluster_tolerance, euc_min_cluster_size,
      euc_max_cluster_size, hole_euc_cluster_tree);

  }


  // ---------------------- cluster cloud creation ------------------------
  // create a Point Cloud containing the points of of the clusters (just the outline points)
  pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  sensor_msgs::PointCloud2::Ptr clusters (new sensor_msgs::PointCloud2);
  NASA_ARMS::PointIndicesArray centroids;
  int cluster_count;  // how many clusters detected?

  create_cluster_cloud(planar_cloud_y, euc_cluster_indices, clustered_cloud, cluster_count, centroids);

  // hole cluster cloud creation
  int hole_cluster_count;
  NASA_ARMS::PointIndicesArray hole_centroids;
  pcl::PointCloud<pcl::PointXYZ>::Ptr hole_clustered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  sensor_msgs::PointCloud2::Ptr hole_clusters (new sensor_msgs::PointCloud2);

  if (edge_detection)
  {

    create_cluster_cloud(edge_cloud_output, hole_euc_cluster_indices, hole_clustered_cloud, hole_cluster_count,
      hole_centroids);

    // combine the centroids from the holes and the regular obstacles

    centroids.points.reserve(centroids.points.size() + hole_centroids.points.size());
    centroids.points.insert(centroids.points.end(), hole_centroids.points.begin(), hole_centroids.points.end());
  }


  // -------- Publish results --------------
  centroid_publisher.publish(centroids);

  pcl::toROSMsg(*clustered_cloud, *clusters);

  clusters->header.frame_id = "/kinect2_link";
  clusters->header.stamp = ros::Time::now();

  euc_cluster_publisher.publish(clusters);


  // hole publish results
  if (edge_detection)
  {
    hole_centroid_publisher.publish(hole_centroids);

    pcl::toROSMsg(*hole_clustered_cloud, *hole_clusters);

    hole_clusters->header.frame_id = "/kinect2_link";
    hole_clusters->header.stamp = ros::Time::now();
    hole_cluster_publisher.publish(hole_clusters);

  }

  std::cout << " ----------- " << std::endl;
  std::cout << "clusters found: " << cluster_count << " hole clusters found: " << hole_cluster_count << " combined size: " << centroids.points.size() << std::endl;

}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "obstacle_detection");
  ros::NodeHandle nh;

  downsample_input_data = true;  // make the dataset smaller (and faster to process)
  passthrough_filter_enable = true;  // do we wanna cut things out? (useful for trimming the points down to the dimensions of the field)
  edge_detection = true;  // do we wanna see holes? (but slowly)

  pt_lower_lim_y = -0.5;  // upper limit on the y axis filtered by the passthrough filter (INVERTED B/C KINECT)
  pt_upper_lim_y = 0.6;  // lower limit on the y axis filtered by the passthrough filter (INVERTED B/C KINECT)

  pt_lower_lim_x = -1.8;  // lower lim on x axis for passthrough filter
  pt_upper_lim_x = 1.8;   // upper lim on x axis for passthrough filter

  pt_lower_lim_z = 0;     // lower lim on z axis for passthrough filter
  pt_upper_lim_z = 5;  // upper lim on z axis for passthrough filter


  downsample_leaf_size = 0.015;  // for VoxelGrid (default: 0.1)

  statistical_outlier_meanK = 15;  // how many neighbor points to examine? (default: 50)
  statistical_outlier_stdDevThres = 1.0;  // deviation multiplier (default: 1.0)

  plane_segment_dist_thres = 0.040;  // how close to be an inlier? (default: 0.01) Hella sensitive!
  plane_segment_angle = 20;

  euc_cluster_tolerance = 0.4;
  euc_min_cluster_size = 5;  // min # of points in an object cluster
  euc_max_cluster_size = 20000;  // max # of points in an object cluster
  convex_hull_alpha = 180.0;  // max internal angle of the geometric shape formed by points.

  hole_euc_cluster_tolerance = 0.02;
  edge_depth_tolerance = 0.1;
  edge_max_neighbor_search = 100;

  frames_accumulated = 4; // must have an int square root (really good to know)

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe (point_topic, 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  downsample_publisher = nh.advertise<sensor_msgs::PointCloud2> ("voxel_grid", 1);
  statistical_outlier_publisher = nh.advertise<sensor_msgs::PointCloud2>("statistical_outliers", 1);
  indices_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("indices_cloud", 1000);
  planar_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("planar_cloud", 1000);
  filtered_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("cloud_f", 1000);
  centroid_publisher = nh.advertise<NASA_ARMS::PointIndicesArray>("centroids", 1);
  edge_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("edge_cloud", 1);
  euc_cluster_publisher = nh.advertise<sensor_msgs::PointCloud2> ("euc_clusters", 5);
  filtered_edge_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2> ("edge_cloud_filtered", 1);
  hole_centroid_publisher = nh.advertise<NASA_ARMS::PointIndicesArray>("hole_centroids", 1);
  hole_cluster_publisher = nh.advertise<sensor_msgs::PointCloud2>("hole_euc_clusters", 1);

  // Spin
  ros::spin ();
}
