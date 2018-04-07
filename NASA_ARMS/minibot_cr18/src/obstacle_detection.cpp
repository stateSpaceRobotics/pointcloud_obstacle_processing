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

// TODO: make this safe to use with a transform
// TODO: remove all traces of edge detection
// TODO: remove things that are being done in the frame_preprocessor
// TODO: integrate with occupancy grid?
// TODO: make this work with a config file

#include <ros/ros.h>
#include <ros/console.h>
#include <chrono>
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

// Occupancy Grid
#include <nav_msgs/OccupancyGrid.h>


// very bad, no good global variables init
ros::Publisher downsample_publisher;
ros::Publisher statistical_outlier_publisher;
ros::Publisher indices_cloud_publisher;
ros::Publisher planar_cloud_publisher;
ros::Publisher filtered_cloud_publisher;
ros::Publisher centroid_publisher;
ros::Publisher euc_cluster_publisher;
ros::Publisher occupancy_grid_publisher;

pcl::PointCloud<pcl::PointXYZ> passthrough_cloud; // for the accumulation

//const char *point_topic = "/NASA_ARMS/obstacle_preprocessor/accumulated_depth_1";  // where are we getting the depth data from?
const char *point_topic = "/kinect2/sd/points";

int frames_to_accumulate;
int current_frame_count;


float block_size;
float dev_percent;

int occupancy_grid_width;
int occupancy_grid_height;
int occupancy_grid_size;
long long *occupancy_grid_pt_counts;
long long *row_averages;

bool downsample_input_data;
bool passthrough_filter_enable;
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

std::chrono::high_resolution_clock::time_point occ_grid_loop_start;
std::chrono::high_resolution_clock::time_point occ_grid_loop_stop;


// TODO: Document
int get_occupancy_grid_location(float x, float z, float x_min, float z_max, float block_size, int occupancy_grid_width, int occupancy_grid_height)
{
  int x_count = 0;
  int y_count = 0;

  while (x_min + (x_count + 1)*block_size < x)
  {
    x_count++;
  }

  while (z_max - (y_count + 1)*block_size > z)
  {
    y_count++;
  }
  //int result = y_count*occupancy_grid_width + x_count;
  return y_count*occupancy_grid_width + x_count;


  /*
  int x_index;
  int y_index;
  x = x + 2;

  x_index = (int)(x / block_size);
  y_index = (int)(y / block_size);

  x_index = (x_index >= occupancy_grid_width) ? occupancy_grid_width - 1: x_index;
  y_index = (y_index >= occupancy_grid_height) ? occupancy_grid_height - 1 : y_index;

  return y_index * occupancy_grid_width + x_index;
  */
}

void build_initial_occupancy_grid_dataset(int grid_size, int grid_height, int grid_width, char* grid_data, float x_min, float x_max,
                                          float y_min, float y_max, float z_min, float z_max, long long *point_count_array,
                                          long long *row_point_averages, pcl::PointCloud<pcl::PointXYZ>& input_cloud,
                                          pcl::PointCloud<pcl::PointXYZ>& output_cloud)
{
  int nan_count = 0;
  int point_count = 0;
  int occupied = 0;

  for (int i = 0 ; i < grid_size; i++) {
    point_count_array[i] = 0;
  }

  for (int i = 0 ; i < occupancy_grid_height; i++) {
    row_averages[i] = 0;
  }

  // time
  occ_grid_loop_start = std::chrono::high_resolution_clock::now();

  for(int i = 0; i < static_cast<int> (input_cloud.points.size()); ++i)  // iterate through all points and place them in an occupancy grid
  {
    if (pcl_isnan(input_cloud.points[i].x) || input_cloud.points[i].x < x_min || input_cloud.points[i].x > x_max
        || input_cloud.points[i].z < z_min || input_cloud.points[i].z > z_max || input_cloud.points[i].y < y_min || input_cloud.points[i].y > y_max){
      continue;  // don't count NaN points or out of bounds points.
    }

    point_count_array[get_occupancy_grid_location(input_cloud.points[i].x, input_cloud.points[i].z, x_min, z_max,
                                                  block_size, grid_width, grid_height)]++;  // determine location in grid for point
    point_count++;

    output_cloud.push_back(input_cloud.points[i]);
  }

  occ_grid_loop_stop = std::chrono::high_resolution_clock::now();

  //std::cout << nan_count<< std::endl;

  //std::cout << "Occupancy_Grid size: " << occupancy_grid_size << std::endl;
  //std::cout << "Block size: " << block_size << " height: " << occupancy_grid_height << " width: " << occupancy_grid_width << std::endl;

  long long row_point_count = 0;

  for (int row_ind = 0; row_ind < grid_height; row_ind++)  // compute averages for each row in point cloud.
  {
    row_point_count = 0;
    for (int col_ind = 0; col_ind < grid_width; col_ind++)
    {
      row_point_count += point_count_array[row_ind*grid_width + col_ind];
    }
    row_point_averages[row_ind] = row_point_count / grid_width;
  }


  // create the occupancy grid here
  //char *occupancy_grid_data = new char[grid_size];
  int row_count = 0;

  for (int i = 0; i < grid_size; i++)
  {

    long long current_row_avg = row_point_averages[(int)(i / grid_width)];

    /*
    int percent = 0;

    if(current_row_avg > 0)
    {
      percent = (1 - (occupancy_grid_pt_counts[i] / current_row_avg) * 100);
    }

    if(percent < 0) percent = 0;
    occupancy_grid_data[i] = percent;
    */

    if (point_count_array[i] < current_row_avg * (1-dev_percent))
    {
      grid_data[i] = 100;

    } else {

      grid_data[i] = 0;
    }

    //occupancy_grid_data[i] = (occupancy_grid_pt_counts[i] < row_averages[(int)(i / occupancy_grid_width)] * (1-dev_percent)) ? 100 : 0;
  }
}

void downsample_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, pcl::PointCloud<pcl::PointXYZ>& output_cloud, float leaf_size,
                      ros::Publisher &pub, bool publish)
{
	/* downsamples the input cloud using a VoxelGrid algorithm down to the specified leaf size
	 * and then can publish this on the specified publisher based on the boolean value of publish.
	 * Useful for quickly improving the performance of the program, but also quickly decreases the
	 * quality of the obstacle detection.
	 *
	 * Option to publish the output cloud on a Point Cloud ROS topic, if enabled with publish
	*/

    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (input_cloud);
    sor.setLeafSize(leaf_size, leaf_size, leaf_size);
    sor.filter (output_cloud);

    // Publish the data
    if (publish)
    {
      sensor_msgs::PointCloud2 publish_cloud;
      pcl::toROSMsg(output_cloud, publish_cloud);
      publish_cloud.header.frame_id = "/kinect2_link";
      publish_cloud.header.stamp = ros::Time::now();
      pub.publish(publish_cloud);
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
    publish_cloud.header.frame_id = "/kinect2_link";
    publish_cloud.header.stamp = ros::Time::now();
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
  // While 30% of the original cloud is still there ... ?

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
    publish_cloud1.header.frame_id = "/kinect2_link";
    publish_cloud1.header.stamp = ros::Time::now();
    pub1.publish(publish_cloud1);
  }

  if (publish_indices)
  {
    sensor_msgs::PointCloud2 publish_cloud2;
    pcl::toROSMsg(*indices_cloud, publish_cloud2);
    publish_cloud2.header.frame_id = "/kinect2_link";
    publish_cloud2.header.stamp = ros::Time::now();
    pub2.publish(publish_cloud2);
  }

  if (publish_filtered)
  {
    sensor_msgs::PointCloud2 publish_cloud3;
    pcl::toROSMsg(*cloud_f, publish_cloud3);
    publish_cloud3.header.frame_id = "/kinect2_link";
    publish_cloud3.header.stamp = ros::Time::now();
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


void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
  /* Called when a PointCloud is received from the Kinect.
   * Orchestrates the obstacle detection using a bunch of other functions.
   *
   */
  // time

  // Container for original and filtered data
  pcl::PCLPointCloud2 *initial_cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr initial_cloud_ptr(initial_cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr accumulator_input_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // Convert to PCLPointCloud2 data type from ROS sensor_msgs
  pcl_conversions::toPCL(*cloud_msg, *initial_cloud);
  pcl::fromPCLPointCloud2(*initial_cloud, *accumulator_input_cloud);

  if (current_frame_count < frames_to_accumulate) {
    passthrough_cloud += *accumulator_input_cloud;
    current_frame_count += 1;
  } else {

    // TODO: Add output for final_cloud

    current_frame_count = 0;

    auto init_start = std::chrono::high_resolution_clock::now();
    //pcl::PointCloud<pcl::PointXYZ>::Ptr passthrough_cloud(
    //        new pcl::PointCloud<pcl::PointXYZ>);  // for the occupancy grid
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsample_input_cloud(new pcl::PointCloud<pcl::PointXYZ>);  // for downsampling

    pcl::PointCloud<pcl::PointXYZ>::Ptr statistical_outlier_input_cloud(
            new pcl::PointCloud<pcl::PointXYZ>);  // new point cloud to use in PCL
    pcl::PointCloud<pcl::PointXYZ> statistical_outlier_output_cloud;

    //pcl::fromPCLPointCloud2(*initial_cloud,
    //                        *passthrough_cloud);  // convert PCLPointCloud2 to PointCloud<PointXYZ> for processing (annoying, but seemingly necessary)
    // time
    auto init_stop = std::chrono::high_resolution_clock::now();
    // ^^^ THAT PART IS MASSIVELY SLOW (compared to everything except euclidean cluster extraction)

    // --------------------- passthrough filter, detect holes and build the occupancy grid data set-----
    auto occ_start = std::chrono::high_resolution_clock::now();
    char *occupancy_grid_data = new char[occupancy_grid_size];

    build_initial_occupancy_grid_dataset(occupancy_grid_size, occupancy_grid_height, occupancy_grid_width,
                                         occupancy_grid_data, pt_lower_lim_x,
                                         pt_upper_lim_x, pt_lower_lim_y, pt_upper_lim_y, pt_lower_lim_z, pt_upper_lim_z,
                                         occupancy_grid_pt_counts, row_averages, passthrough_cloud,
                                         *downsample_input_cloud);
    // time
    auto occ_stop = std::chrono::high_resolution_clock::now();

    // TODO: should this be done before everything else?
    // ---------------------- downsample using VoxelGrid --------------------------------

    // time
    auto downsample_start = std::chrono::high_resolution_clock::now();
    downsample_cloud(downsample_input_cloud, *statistical_outlier_input_cloud, downsample_leaf_size,
                     downsample_publisher, true);

    // time
    auto downsample_stop = std::chrono::high_resolution_clock::now();

    // ------------------- statistical outlier removal ---------------------  // TODO: Determine if this is needed

    // time
    auto stat_out_rem_start = std::chrono::high_resolution_clock::now();

    remove_statistical_outliers(statistical_outlier_input_cloud, statistical_outlier_output_cloud,
                                statistical_outlier_meanK, statistical_outlier_stdDevThres,
                                statistical_outlier_publisher,
                                true);

    // time
    auto stat_out_rem_stop = std::chrono::high_resolution_clock::now();

    // ----------------- plane segmentation (floor removal) ----------------

    // time
    auto plane_seg_start = std::chrono::high_resolution_clock::now();

    // Create the data structures necessary for segmenting the planes and extracting the indices remaining.
    pcl::PointCloud<pcl::PointXYZ>::Ptr planar_cloud_y(statistical_outlier_output_cloud.makeShared());
    pcl::ModelCoefficients::Ptr coefficients_y(new pcl::ModelCoefficients);

    // TODO: Change for the transform
    const Eigen::Matrix<float, 3, 1> &plane_axis_y = Eigen::Vector3f(0, 1, 0);  // x, y, z

    pcl::PointIndices::Ptr inliers_y(new pcl::PointIndices);
    pcl::PointIndices::Ptr outliers_y(new pcl::PointIndices);

    pcl::PointCloud<pcl::PointXYZ>::Ptr indices_cloud_y(statistical_outlier_output_cloud.makeShared());
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_y(statistical_outlier_output_cloud.makeShared());

    // perform the planar segmentation
    segment_plane_and_extract_indices(planar_cloud_y, indices_cloud_y, filtered_cloud_y, coefficients_y, inliers_y,
                                      outliers_y, plane_axis_y, plane_segment_dist_thres, plane_segment_angle,
                                      planar_cloud_publisher, indices_cloud_publisher, filtered_cloud_publisher,
                                      true, true, true);

    // time
    auto plane_seg_stop = std::chrono::high_resolution_clock::now();

    // ------------------ euclidian cluster extraction ------------------
    // time
    auto euc_start = std::chrono::high_resolution_clock::now();
    
    // create data structures for euclidian cluster extraction.
    pcl::search::KdTree<pcl::PointXYZ>::Ptr euc_cluster_tree(new pcl::search::KdTree<pcl::PointXYZ>);
    euc_cluster_tree->setInputCloud(planar_cloud_y);
    std::vector<pcl::PointIndices> euc_cluster_indices;  // each entry is a vector of indices_cloud

    // extract the clusters of points that meet certain parameters in the point cloud
    extract_euclidian_clusters(planar_cloud_y, euc_cluster_indices, euc_cluster_tolerance, euc_min_cluster_size,
                               euc_max_cluster_size, euc_cluster_tree);


    // time
    auto euc_stop = std::chrono::high_resolution_clock::now();
    // ---------------------- cluster cloud creation ------------------------

    // time
    auto cluster_start = std::chrono::high_resolution_clock::now();

    // create a Point Cloud containing the points of of the clusters (just the outline points)
    pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::PointCloud2::Ptr clusters(new sensor_msgs::PointCloud2);
    NASA_ARMS::PointIndicesArray centroids;
    int cluster_count;  // how many clusters detected?

    create_cluster_cloud(planar_cloud_y, euc_cluster_indices, clustered_cloud, cluster_count, centroids);


    for (int i = 0; i < planar_cloud_y->points.size(); i++) {
      // safety first!
      if (pcl_isnan(planar_cloud_y->points[i].x)) {
        continue;
      }
      // get the index of the point
      int index = get_occupancy_grid_location(planar_cloud_y->points[i].x, planar_cloud_y->points[i].z, pt_lower_lim_x,
                                              pt_upper_lim_z, block_size, occupancy_grid_width, occupancy_grid_height);
      occupancy_grid_data[index] = 100; // mark it as a bad thing.
    }

    nav_msgs::OccupancyGrid *occupancyGrid = new nav_msgs::OccupancyGrid();
    occupancyGrid->info.resolution = block_size;
    occupancyGrid->info.width = occupancy_grid_width;
    occupancyGrid->info.height = occupancy_grid_height;
    occupancyGrid->data = std::vector<int8_t>(occupancy_grid_data, occupancy_grid_data + occupancy_grid_size);
    occupancyGrid->header.frame_id = "/kinect2_link";

    occupancy_grid_publisher.publish(*occupancyGrid);

    //time
    auto cluster_stop = std::chrono::high_resolution_clock::now();

    /*
    // -------- Publish results --------------
    centroid_publisher.publish(centroids);

    pcl::toROSMsg(*clustered_cloud, *clusters);

    clusters->header.frame_id = "/kinect2_link";
    clusters->header.stamp = ros::Time::now();

    euc_cluster_publisher.publish(clusters);

    std::cout << " ----------- " << std::endl;
    std::cout << "clusters found: " << cluster_count << " combined size: " << centroids.points.size() << std::endl;
    */

    // TIME CALCS
    std::chrono::duration<double> init_time_taken = std::chrono::duration_cast<std::chrono::duration<double>>(
            init_stop - init_start);
    //std::chrono::duration<double> passthrough_time_taken = std::chrono::duration_cast<std::chrono::duration<double>> (passthrough_stop - passthrough_start);
    std::chrono::duration<double> occ_grid_time_taken = std::chrono::duration_cast<std::chrono::duration<double>>(
            occ_stop - occ_start);
    std::chrono::duration<double> occ_grid_loop_time_taken = std::chrono::duration_cast<std::chrono::duration<double>>(
            occ_grid_loop_stop - occ_grid_loop_start);
    std::chrono::duration<double> downsample_time_taken = std::chrono::duration_cast<std::chrono::duration<double>>(
            downsample_stop - downsample_start);
    std::chrono::duration<double> stat_out_time_taken = std::chrono::duration_cast<std::chrono::duration<double>>(
            stat_out_rem_stop - stat_out_rem_start);
    std::chrono::duration<double> plane_seg_time_taken = std::chrono::duration_cast<std::chrono::duration<double>>(
            plane_seg_stop - plane_seg_start);
    std::chrono::duration<double> euc_time_taken = std::chrono::duration_cast<std::chrono::duration<double>>(
            euc_stop - euc_start);
    std::chrono::duration<double> cluster_time_taken = std::chrono::duration_cast<std::chrono::duration<double>>(
            cluster_stop - cluster_start);

    auto init_time_elapsed = (float) init_time_taken.count();
    //auto passthrough_time_elapsed = (float) passthrough_time_taken.count();
    auto occ_elapsed_time = (float) occ_grid_time_taken.count();
    auto occ_grid_loop_elapsed_time = (float) occ_grid_loop_time_taken.count();
    auto downsample_elapsed_time = (float) downsample_time_taken.count();
    auto stat_out_rem_elapsed_time = (float) stat_out_time_taken.count();
    auto plane_seg_elapsed_time = (float) plane_seg_time_taken.count();
    auto euc_cluster_elapsed_time = (float) euc_time_taken.count();
    auto cluster_elapsed_time = (float) cluster_time_taken.count();
    auto total_time = init_time_elapsed + occ_elapsed_time + downsample_elapsed_time + stat_out_rem_elapsed_time
                      + plane_seg_elapsed_time + euc_cluster_elapsed_time + cluster_elapsed_time;

    auto init_percent = (float) ((init_time_elapsed / total_time) * 100);
    //auto passthrough_percent = (float)((passthrough_time_elapsed / total_time)*100);
    auto occ_percent = (float) ((occ_elapsed_time / total_time) * 100);
    auto occ_grid_loop_percent = (float) ((occ_grid_loop_elapsed_time / occ_elapsed_time) * 100);
    auto downsample_percent = (float) ((downsample_elapsed_time / total_time) * 100);
    auto stat_out_rem_percent = (float) ((stat_out_rem_elapsed_time / total_time) * 100);
    auto plane_seg_percent = (float) ((plane_seg_elapsed_time / total_time) * 100);
    auto euc_cluster_percent = (float) ((euc_cluster_elapsed_time / total_time) * 100);
    auto cluster_percent = (float) ((cluster_elapsed_time / total_time) * 100);

    ROS_ERROR("---------Time summaries:-------------------");
    ROS_ERROR("-------------------TOTAL TIME: %f seconds", total_time);
    ROS_ERROR("----------initial conversions: %f seconds (%f) percent", init_time_elapsed, init_percent);
    //ROS_ERROR("--------passthrough filtering: %f seconds (%f) percent", passthrough_time_elapsed, passthrough_percent);
    ROS_ERROR("------occupancy grid creation: %f seconds (%f) percent", occ_elapsed_time, occ_percent);
    //ROS_ERROR("......iteration through cloud: %f seconds (%f) percent (of parent)", occ_grid_loop_elapsed_time, occ_grid_loop_percent);
    ROS_ERROR("-----------------downsampling: %f seconds (%f) percent", downsample_elapsed_time, downsample_percent);
    ROS_ERROR("--statistical outlier removal: %f seconds (%f) percent", stat_out_rem_elapsed_time,
              stat_out_rem_percent);
    ROS_ERROR("-----------plane segmentation: %f seconds (%f) percent", plane_seg_elapsed_time, plane_seg_percent);
    ROS_ERROR("---------euclidian clustering: %f seconds (%f) percent", euc_cluster_elapsed_time, euc_cluster_percent);
    ROS_ERROR("-------cluster cloud creation: %f seconds (%f) percent", cluster_elapsed_time, cluster_percent);
    ROS_ERROR("-------------------------------------------");
    passthrough_cloud.clear();
  }
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "obstacle_detection");
  ros::NodeHandle nh ("~");
  ros::NodeHandle nh_pub;

  nh.param("accumulate_count", frames_to_accumulate, 2);
  current_frame_count = 0;

  nh.param("downsample_input_data", downsample_input_data, true);  // make the dataset smaller (and faster to process)
  nh.param("passthrough_filter_enable", passthrough_filter_enable, true);  // do we wanna cut things out? (useful for trimming the points down to the dimensions of the field)

  nh_pub.param("x_min", pt_lower_lim_x, (float) -1.0);  // lower lim on x axis for passthrough filter
  nh_pub.param("x_max", pt_upper_lim_x, (float) 1.0);  // upper lim on x axis for passthrough filter
  nh_pub.param("y_min", pt_lower_lim_y, (float) -0.5);  // upper limit on the y axis filtered by the passthrough filter (INVERTED B/C KINECT)
  nh_pub.param("y_max", pt_upper_lim_y, (float) 0.6);  // lower limit on the y axis filtered by the passthrough filter (INVERTED B/C KINECT)
  nh_pub.param("z_min", pt_lower_lim_z, (float) 0);  // lower lim on z axis for passthrough filter
  nh_pub.param("z_max", pt_upper_lim_z, (float)-0.5);  // upper lim on z axis for passthrough filter

  nh.param("block_size", block_size, (float) 0.15);
  nh.param("dev_percent", dev_percent, (float) 0.5);

  occupancy_grid_width = (int)ceil((fabs(pt_lower_lim_x) + fabs(pt_upper_lim_x)) / block_size);
  occupancy_grid_height = (int)ceil((fabs(pt_upper_lim_z) - fabs(pt_lower_lim_z)) / block_size);
  occupancy_grid_size = occupancy_grid_width * occupancy_grid_height;
  occupancy_grid_pt_counts = new long long[occupancy_grid_size];
  row_averages = new long long[occupancy_grid_height];

  nh.param("downsample_size", downsample_leaf_size, (float)0.015);  // for VoxelGrid (default: 0.1)

  nh.param("statistical_outlier_meanK", statistical_outlier_meanK, 15);  // how many neighbor points to examine? (default: 50)
  nh.param("statistical_outlier_stdDevThres", statistical_outlier_stdDevThres, (float)1.0);  // deviation multiplier (default: 1.0)

  nh.param("plane_segment_dist_thresh", plane_segment_dist_thres, (float)0.040);  // how close to be an inlier? (default: 0.01) Hella sensitive!
  nh.param("plane_segment_angle", plane_segment_angle, 20);

  nh.param("euc_cluster_tolerance", euc_cluster_tolerance, (float)0.4);
  nh.param("euc_min_cluster_size", euc_min_cluster_size, 5);  // min # of points in an object cluster
  nh.param("euc_max_cluster_size", euc_max_cluster_size, 20000);  // max # of points in an object cluster
  nh.param("convex_hull_alpha", convex_hull_alpha, (float)180.0);  // max internal angle of the geometric shape formed by points.

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe (point_topic, 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  downsample_publisher = nh.advertise<sensor_msgs::PointCloud2> ("voxel_grid", 1);
  statistical_outlier_publisher = nh.advertise<sensor_msgs::PointCloud2>("statistical_outliers", 1);
  indices_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("indices_cloud", 1000);
  planar_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("planar_cloud", 1000);
  filtered_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("cloud_f", 1000);
  centroid_publisher = nh.advertise<NASA_ARMS::PointIndicesArray>("centroids", 1);
  euc_cluster_publisher = nh.advertise<sensor_msgs::PointCloud2> ("euc_clusters", 5);
  occupancy_grid_publisher = nh.advertise<nav_msgs::OccupancyGrid> ("occupancy_grid", 1);

  // Spin
  ros::spin ();
}
