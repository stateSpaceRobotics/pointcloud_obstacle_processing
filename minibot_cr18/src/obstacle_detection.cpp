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
#include <pointcloud_obstacle_processing/PointIndicesArray.h>
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

// for transforms
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <pcl_ros/transforms.h>
#include <tf/transform_datatypes.h>

// very bad, no good global variables init
ros::Publisher downsample_publisher;
ros::Publisher statistical_outlier_publisher;
ros::Publisher indices_cloud_publisher;
ros::Publisher planar_cloud_publisher;
ros::Publisher filtered_cloud_publisher;
ros::Publisher centroid_publisher;
ros::Publisher euc_cluster_publisher;
ros::Publisher occupancy_grid_publisher;

pcl::PointCloud<pcl::PointXYZ> passthrough_input_cloud; // for the accumulation

const char *point_topic = "/kinect2/qhd/points";

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

bool publish_point_clouds;

std::chrono::high_resolution_clock::time_point occ_grid_loop_start;
std::chrono::high_resolution_clock::time_point occ_grid_loop_stop;


tf2_ros::Buffer tfBuffer;
geometry_msgs::TransformStamped transformStamped;


struct Vertex
{
    float x;
    float y;
};

std::pair<int, int> get_occupancy_grid_x_y(float &x, float &y, float &x_min, float &y_max, float &block_size)
{
  int x_count = 0;
  int y_count = 0;

  while (x_min + (x_count + 1)*block_size < x)
  {
    x_count++;
  }

  while (y_max - (y_count + 1)*block_size > y)
  {
    y_count++;
  }

  return std::pair<int, int>(x_count, y_count);
};

// TODO: Document
int get_occupancy_grid_location(float x, float y, float x_min, float y_max, float block_size, int occupancy_grid_width)
{

  std::pair<int, int>grid_xy = get_occupancy_grid_x_y(x, y, x_min, y_max, block_size);
  return grid_xy.second*occupancy_grid_width + grid_xy.first;


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
        || input_cloud.points[i].z < z_min || input_cloud.points[i].z > z_max || input_cloud.points[i].y < y_min ||
        input_cloud.points[i].y > y_max) {
      continue;  // don't count NaN points or out of bounds points.
    }

    int index = get_occupancy_grid_location(input_cloud.points[i].y, input_cloud.points[i].x, y_min, x_max,
                                            block_size, grid_width);
    if (index >= occupancy_grid_size) {
      ROS_ERROR("OUT OF BOUNDS INDEX ACCESS");

    } else {

      point_count_array[index]++;  // determine location in grid for point
      point_count++;
    }

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
      publish_cloud.header.frame_id = "world";
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
    publish_cloud.header.frame_id = "world";
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
    publish_cloud1.header.frame_id = "world";
    publish_cloud1.header.stamp = ros::Time::now();
    pub1.publish(publish_cloud1);
  }

  if (publish_indices)
  {
    sensor_msgs::PointCloud2 publish_cloud2;
    pcl::toROSMsg(*indices_cloud, publish_cloud2);
    publish_cloud2.header.frame_id = "world";
    publish_cloud2.header.stamp = ros::Time::now();
    pub2.publish(publish_cloud2);
  }

  if (publish_filtered)
  {
    sensor_msgs::PointCloud2 publish_cloud3;
    pcl::toROSMsg(*cloud_f, publish_cloud3);
    publish_cloud3.header.frame_id = "world";
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


void traceShadow(const Vertex& v1, const Vertex& v2, char* occupancy_grid_data)
{
  int x0 = v1.x, x1 = v2.x, y0 = v1.y, y1 = v2.y;
  int steep = abs(y1 - y0) > abs(x1 - x0);

  // swap the co-ordinates if slope > 1 or we
  // draw backwards
  if (steep)
  {
    std::swap(x0, y0);
    std::swap(x1, y1);
  }
  if (x0 > x1)
  {
    std::swap(x0, x1);
    std::swap(y0, y1);
  }

  //compute the slope
  float dx = x1 - x0;
  float dy = y1 - y0;
  float gradient = dy / dx;
  if (dx == 0.0)
    gradient = 1;

  int xpxl1 = x0;
  int xpxl2 = x1;
  float intersectY = y0;

  // main loop
  if (steep)
  {
    int x;
    for (x = xpxl1; x <= xpxl2; x++)
    {
      // pixel coverage is determined by fractional
      // part of y co-ordinate
      int grid_y = x;
      int grid_x = (int)std::floor(intersectY);

      occupancy_grid_data[grid_y*occupancy_grid_width + grid_x] = 0;
      grid_x += 1;
      occupancy_grid_data[grid_y*occupancy_grid_width + grid_x] = 0;

      intersectY += gradient;
    }
  }

  else
  {
    int x;
    for (x = xpxl1; x <= xpxl2; x++)
    {
      // pixel coverage is determined by fractional
      // part of y co-ordinate
      int grid_y = (int)std::floor(intersectY);
      int grid_x = x;

      occupancy_grid_data[grid_y*occupancy_grid_width + grid_x] = 0;
      grid_y += 1;
      occupancy_grid_data[grid_y*occupancy_grid_width + grid_x] = 0;

      intersectY += gradient;
    }
  }

}

std::pair<int, int> calculate_shadow_cast(char *occupancy_grid_data,
                                          const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, float horizontal_min,
                                          float horizontal_max, pcl::PointXYZ vertical_min_pt, float vertical_max, float &width) {
  float a = vertical_min_pt.z;
  float b = fabs(vertical_min_pt.x);
  float c = sqrt(a * a + b * b);
  float e = fabs(vertical_max) - fabs(vertical_min_pt.x);
  float D = asin(a / c);
  float d = tan(D) * e + 0.25;

  width = horizontal_max - horizontal_min;

  ROS_ERROR("distance of shadow: %f", d);
  ROS_ERROR("width of obstacle: %f", width);
  ROS_ERROR("height of obstacle: %f", e);

  float v_len = sqrt(vertical_min_pt.x * vertical_min_pt.x + vertical_min_pt.y * vertical_min_pt.y + vertical_min_pt.z * vertical_min_pt.z);

  ROS_ERROR("vector_length: %f", v_len);

  pcl::PointXYZ norm_vector(vertical_min_pt.x / v_len, vertical_min_pt.y / v_len, vertical_min_pt.z / v_len);

  norm_vector.x *= d;
  norm_vector.y *= d;
  norm_vector.z *= d;

  ROS_ERROR("normalized_vector: %f, %f, %f", norm_vector.x, norm_vector.y, norm_vector.z);

  pcl::PointXYZ shadow_end_point(norm_vector.x + vertical_min_pt.x, norm_vector.y + vertical_min_pt.y, norm_vector.z + vertical_min_pt.z);

  ROS_ERROR("shadow_end_point: %f, %f, %f", shadow_end_point.x, shadow_end_point.y, shadow_end_point.z);

  geometry_msgs::TransformStamped transform_to_world = tfBuffer.lookupTransform("world", "kinect2_link", ros::Time(0));
  tf::StampedTransform tf_to_world;
  tf::transformStampedMsgToTF(transform_to_world, tf_to_world);

  pcl::PointCloud<pcl::PointXYZ> kinect_cloud;
  pcl::PointCloud<pcl::PointXYZ> world_shadow_cloud;

  kinect_cloud.points.push_back(shadow_end_point);
  pcl_ros::transformPointCloud(kinect_cloud, world_shadow_cloud, tf_to_world);


  return get_occupancy_grid_x_y(world_shadow_cloud.points[0].y, world_shadow_cloud.points[0].x, pt_lower_lim_y, pt_upper_lim_x, block_size);
}

void handle_shadow_casting(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, std::vector<int> &cluster, char *occupancy_grid_data)
{
  if (cluster.size() < 2)
  {
    return;
  }

  // transform from the world frame into the kinect2 frame.
  geometry_msgs::TransformStamped reverse_transform= tfBuffer.lookupTransform("kinect2_link", "world", ros::Time(0));
  tf::StampedTransform reverse_tf;
  tf::transformStampedMsgToTF(reverse_transform, reverse_tf);

  pcl::PointCloud<pcl::PointXYZ> shadow_cloud;
  pcl::PointCloud<pcl::PointXYZ> transformed_point_cloud;

  for(int i = 0; i < cluster.size(); i++)
  {
    shadow_cloud.points.push_back(input_cloud->points[cluster[i]]);
  }

  pcl_ros::transformPointCloud(shadow_cloud, transformed_point_cloud, reverse_tf);



  pcl::PointXYZ vertical_axis_min_pt = transformed_point_cloud.points[0];
  float vertical_axis_max = transformed_point_cloud.points[0].x;
  float horizontal_axis_min = transformed_point_cloud.points[0].y;
  float horizontal_axis_max = transformed_point_cloud.points[0].y;
  float width = 0;

  for (int i = 1; i < transformed_point_cloud.points.size(); i++)
  {
    if (transformed_point_cloud.points[i].x < vertical_axis_min_pt.x) vertical_axis_min_pt = transformed_point_cloud.points[i];
    if (transformed_point_cloud.points[i].x > vertical_axis_max) vertical_axis_max = transformed_point_cloud.points[i].x;
    if (transformed_point_cloud.points[i].y < horizontal_axis_min) horizontal_axis_min = transformed_point_cloud.points[i].y;
    if (transformed_point_cloud.points[i].y > horizontal_axis_max) horizontal_axis_max = transformed_point_cloud.points[i].y;
  }

  ROS_ERROR("vertical_axis_min_pt: %f, %f, %f", vertical_axis_min_pt.x, vertical_axis_min_pt.y, vertical_axis_min_pt.z);
  ROS_ERROR("vertical_axis_max: %f", vertical_axis_max);
  ROS_ERROR("horizontal_axis_min: %f", horizontal_axis_min);
  ROS_ERROR("horizontal_axis_max: %f", horizontal_axis_max);



  // kinect2 frame input, world frame output
  std::pair<int, int> end_line_grid_xy = calculate_shadow_cast(occupancy_grid_data, transformed_point_cloud.makeShared(), horizontal_axis_min, horizontal_axis_max, vertical_axis_min_pt, vertical_axis_max, width);

  // transform from kinect2 to world frame
  geometry_msgs::TransformStamped transform_to_world = tfBuffer.lookupTransform("world", "kinect2_link", ros::Time(0));
  tf::StampedTransform world_tf;
  tf::transformStampedMsgToTF(transform_to_world, world_tf);

  pcl::PointCloud<pcl::PointXYZ> start_point;
  start_point.points.push_back(vertical_axis_min_pt);
  pcl::PointCloud<pcl::PointXYZ> transformed_start_point_cloud;

  pcl_ros::transformPointCloud(start_point, transformed_start_point_cloud, world_tf);

  // world frame
  std::pair<int, int> start_line_grid_xy = get_occupancy_grid_x_y(transformed_start_point_cloud.points[0].y, transformed_start_point_cloud.points[0].x, pt_lower_lim_y, pt_upper_lim_x, block_size);

  ROS_ERROR("line_start_point: %f, %f", transformed_start_point_cloud.points[0].x, transformed_start_point_cloud.points[0].y);
  ROS_ERROR("occupancy_grid start_point: %d, %d", start_line_grid_xy.first, start_line_grid_xy.second);

  Vertex v1, v2;
  v1.x = start_line_grid_xy.first;
  v1.y = start_line_grid_xy.second;

  v2.x = end_line_grid_xy.first;
  v2.y = end_line_grid_xy.second;

  ROS_ERROR("line end points: %d, %d", end_line_grid_xy.first, end_line_grid_xy.second);

  traceShadow(v1, v2, occupancy_grid_data);

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
  pcl::PointCloud<pcl::PointXYZ>::Ptr post_transform_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr accumulator_input_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // Convert to PCLPointCloud2 data type from ROS sensor_msgs
  pcl_conversions::toPCL(*cloud_msg, *initial_cloud);
  pcl::fromPCLPointCloud2(*initial_cloud, *post_transform_cloud);

  if (current_frame_count < frames_to_accumulate) {
    transformStamped = tfBuffer.lookupTransform("world", "kinect2_link", ros::Time(0));
    tf::StampedTransform new_tf;
    tf::transformStampedMsgToTF(transformStamped, new_tf);
    pcl::PointCloud<pcl::PointXYZ> passthrough_cloud;
    pcl_ros::transformPointCloud(*post_transform_cloud, *accumulator_input_cloud, new_tf);
    passthrough_input_cloud += *accumulator_input_cloud;
    current_frame_count += 1;
  } else {

    // TODO: Add output for final_cloud
    auto init_start = std::chrono::high_resolution_clock::now();



    ROS_DEBUG("point cloud size: %d", passthrough_input_cloud.points.size());

    current_frame_count = 0;
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
                                         occupancy_grid_pt_counts, row_averages, passthrough_input_cloud,
                                         *downsample_input_cloud);
    // time
    auto occ_stop = std::chrono::high_resolution_clock::now();
    // TODO: should this be done before everything else?
    ROS_DEBUG("after passthrough point cloud size: %d", downsample_input_cloud->points.size());
    // ---------------------- downsample using VoxelGrid --------------------------------

    // time
    auto downsample_start = std::chrono::high_resolution_clock::now();
    downsample_cloud(downsample_input_cloud, *statistical_outlier_input_cloud, downsample_leaf_size,
                     downsample_publisher, publish_point_clouds);

    // time
    auto downsample_stop = std::chrono::high_resolution_clock::now();

    // ------------------- statistical outlier removal ---------------------  // TODO: Determine if this is needed
    ROS_DEBUG("after downsampling point cloud size: %d", statistical_outlier_input_cloud->points.size());
    // time
    auto stat_out_rem_start = std::chrono::high_resolution_clock::now();

    remove_statistical_outliers(statistical_outlier_input_cloud, statistical_outlier_output_cloud,
                                statistical_outlier_meanK, statistical_outlier_stdDevThres,
                                statistical_outlier_publisher,
                                publish_point_clouds);

    // time
    auto stat_out_rem_stop = std::chrono::high_resolution_clock::now();

    // ----------------- plane segmentation (floor removal) ----------------

    // time
    auto plane_seg_start = std::chrono::high_resolution_clock::now();

    // Create the data structures necessary for segmenting the planes and extracting the indices remaining.
    pcl::PointCloud<pcl::PointXYZ>::Ptr planar_cloud_y(statistical_outlier_output_cloud.makeShared());
    pcl::ModelCoefficients::Ptr coefficients_y(new pcl::ModelCoefficients);

    // TODO: Change for the transform
    const Eigen::Matrix<float, 3, 1> &plane_axis_y = Eigen::Vector3f(0, 0, 1);  // x, y, z

    pcl::PointIndices::Ptr inliers_y(new pcl::PointIndices);
    pcl::PointIndices::Ptr outliers_y(new pcl::PointIndices);

    pcl::PointCloud<pcl::PointXYZ>::Ptr indices_cloud_y(statistical_outlier_output_cloud.makeShared());
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_y(statistical_outlier_output_cloud.makeShared());

    // perform the planar segmentation
    segment_plane_and_extract_indices(planar_cloud_y, indices_cloud_y, filtered_cloud_y, coefficients_y, inliers_y,
                                      outliers_y, plane_axis_y, plane_segment_dist_thres, plane_segment_angle,
                                      planar_cloud_publisher, indices_cloud_publisher, filtered_cloud_publisher,
                                      publish_point_clouds, publish_point_clouds, publish_point_clouds);

    // time
    auto plane_seg_stop = std::chrono::high_resolution_clock::now();

    // ------------------ euclidean cluster extraction ------------------
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
    /*
    // create a Point Cloud containing the points of of the clusters (just the outline points)
    pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::PointCloud2::Ptr clusters(new sensor_msgs::PointCloud2);
    pointcloud_obstacle_processing::PointIndicesArray centroids;
    int cluster_count;  // how many clusters detected?

    create_cluster_cloud(planar_cloud_y, euc_cluster_indices, clustered_cloud, cluster_count, centroids);
    */


    for (int i = 0; i < euc_cluster_indices.size(); i++)
    {
      ROS_ERROR("-------------------Cluster index: %d----------------------", i);
      handle_shadow_casting(planar_cloud_y, euc_cluster_indices[i].indices, occupancy_grid_data);
    }

    for (int i = 0; i < planar_cloud_y->points.size(); i++) {
      // safety first!
      if (pcl_isnan(planar_cloud_y->points[i].x)) {
        continue;
      }
      // get the index of the point
      int index = get_occupancy_grid_location(planar_cloud_y->points[i].y, planar_cloud_y->points[i].x, pt_lower_lim_y,
                                              pt_upper_lim_x, block_size, occupancy_grid_width);
      occupancy_grid_data[index] = 100; // mark it as a bad thing.
    }





    nav_msgs::OccupancyGrid *occupancyGrid = new nav_msgs::OccupancyGrid();
    occupancyGrid->info.resolution = block_size;
    occupancyGrid->info.width = occupancy_grid_width;
    occupancyGrid->info.height = occupancy_grid_height;
    occupancyGrid->data = std::vector<int8_t>(occupancy_grid_data, occupancy_grid_data + occupancy_grid_size);
    occupancyGrid->header.frame_id = "world";
    occupancyGrid->info.origin.orientation.w = 0.707;
    occupancyGrid->info.origin.orientation.z = 0.707;
    occupancyGrid->info.origin.orientation.y = 0;
    occupancyGrid->info.origin.orientation.x = 0;
    occupancyGrid->info.origin.position.x = pt_upper_lim_x;
    occupancyGrid->info.origin.position.y = 0;
    occupancyGrid->info.origin.position.z = 0;

    occupancy_grid_publisher.publish(*occupancyGrid);

    //time
    auto cluster_stop = std::chrono::high_resolution_clock::now();

    /*
    // -------- Publish results --------------
    centroid_publisher.publish(centroids);

    pcl::toROSMsg(*clustered_cloud, *clusters);

    clusters->header.frame_id = "/world";
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

    ROS_DEBUG("---------Time summaries:-------------------");
    ROS_ERROR("-------------------TOTAL TIME: %f seconds", total_time);
    ROS_DEBUG("----------initial conversions: %f seconds (%f) percent", init_time_elapsed, init_percent);
    //ROS_DEBUG("--------passthrough filtering: %f seconds (%f) percent", passthrough_time_elapsed, passthrough_percent);
    ROS_DEBUG("------occupancy grid creation: %f seconds (%f) percent", occ_elapsed_time, occ_percent);
    //ROS_DEBUG("......iteration through cloud: %f seconds (%f) percent (of parent)", occ_grid_loop_elapsed_time, occ_grid_loop_percent);
    ROS_DEBUG("-----------------downsampling: %f seconds (%f) percent", downsample_elapsed_time, downsample_percent);
    ROS_DEBUG("--statistical outlier removal: %f seconds (%f) percent", stat_out_rem_elapsed_time,
         stat_out_rem_percent);
    ROS_DEBUG("-----------plane segmentation: %f seconds (%f) percent", plane_seg_elapsed_time, plane_seg_percent);
    ROS_DEBUG("---------euclidian clustering: %f seconds (%f) percent", euc_cluster_elapsed_time, euc_cluster_percent);
    ROS_DEBUG("-------cluster cloud creation: %f seconds (%f) percent", cluster_elapsed_time, cluster_percent);
    ROS_DEBUG("-------------------------------------------");
    passthrough_input_cloud.clear();
  }
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pointcloud_obstacle_detection");
  ros::NodeHandle nh ("~");
  ros::NodeHandle nh_pub;


  tf2_ros::TransformListener tfListener(tfBuffer);

  nh.param("accumulate_count", frames_to_accumulate, 2);
  current_frame_count = 0;

  nh.param("downsample_input_data", downsample_input_data, true);  // make the dataset smaller (and faster to process)
  nh.param("passthrough_filter_enable", passthrough_filter_enable, true);  // do we wanna cut things out? (useful for trimming the points down to the dimensions of the field)
  nh.param("publish_point_clouds", publish_point_clouds, true);

  nh_pub.param("x_min", pt_lower_lim_x, (float) -1.0);  // lower lim on x axis for passthrough filter
  nh_pub.param("x_max", pt_upper_lim_x, (float) 1.0);  // upper lim on x axis for passthrough filter
  nh_pub.param("y_min", pt_lower_lim_y, (float) -0.5);  // upper limit on the y axis filtered by the passthrough filter (INVERTED B/C KINECT)
  nh_pub.param("y_max", pt_upper_lim_y, (float) 0.6);  // lower limit on the y axis filtered by the passthrough filter (INVERTED B/C KINECT)
  nh_pub.param("z_min", pt_lower_lim_z, (float) 0);  // lower lim on z axis for passthrough filter
  nh_pub.param("z_max", pt_upper_lim_z, (float)-0.5);  // upper lim on z axis for passthrough filter

  nh.param("block_size", block_size, (float) 0.15);
  nh.param("dev_percent", dev_percent, (float) 0.5);

  occupancy_grid_width = (int)ceil((fabs(pt_lower_lim_y) + fabs(pt_upper_lim_y)) / block_size);
  occupancy_grid_height = (int)ceil((fabs(pt_lower_lim_x) + fabs(pt_upper_lim_x)) / block_size);
  occupancy_grid_size = occupancy_grid_width * occupancy_grid_height;
  occupancy_grid_pt_counts = new long long[occupancy_grid_size];
  row_averages = new long long[occupancy_grid_height];

  nh.param("downsample_size", downsample_leaf_size, (float)0.015);  // for VoxelGrid (default: 0.1)

  nh.param("statistical_outlier_meanK", statistical_outlier_meanK, 15);  // how many neighbor points to examine? (default: 50)
  nh.param("statistical_outlier_stdDevThres", statistical_outlier_stdDevThres, (float)1.0);  // deviation multiplier (default: 1.0)

  nh.param("plane_segment_dist_thres", plane_segment_dist_thres, (float)0.040);  // how close to be an inlier? (default: 0.01) Hella sensitive!
  nh.param("plane_segment_angle", plane_segment_angle, 20);

  nh.param("euc_cluster_tolerance", euc_cluster_tolerance, (float)0.4);
  nh.param("euc_min_cluster_size", euc_min_cluster_size, 5);  // min # of points in an object cluster
  nh.param("euc_max_cluster_size", euc_max_cluster_size, 20000);  // max # of points in an object cluster
  nh.param("convex_hull_alpha", convex_hull_alpha, (float)180.0);  // max internal angle of the geometric shape formed by points.

  ROS_DEBUG("------------------------PARAM VALUES-------------------------");
  ROS_DEBUG("               accumulate_count: %d", frames_to_accumulate);
  ROS_DEBUG("                          x_min: %f", pt_lower_lim_x);
  ROS_DEBUG("                          x_max: %f", pt_upper_lim_x);
  ROS_DEBUG("                          y_min: %f", pt_lower_lim_y);
  ROS_DEBUG("                          y_max: %f", pt_upper_lim_y);
  ROS_DEBUG("                          z_min: %f", pt_lower_lim_z);
  ROS_DEBUG("                          z_max: %f", pt_upper_lim_z);
  ROS_DEBUG("                     block_size: %f", block_size);
  ROS_DEBUG("                    dev_percent: %f", dev_percent);
  ROS_DEBUG("                downsample_size: %f", downsample_leaf_size);
  ROS_DEBUG("     statisctical_outlier_meanK: %d", statistical_outlier_meanK );
  ROS_DEBUG("statistical_outlier_stdDevThres: %f", statistical_outlier_stdDevThres);
  ROS_DEBUG("       plane_segment_dist_thres: %f", plane_segment_dist_thres);
  ROS_DEBUG("            plane_segment_angle: %d", plane_segment_angle);
  ROS_DEBUG("          euc_cluster_tolerance: %f", euc_cluster_tolerance);
  ROS_DEBUG("           euc_min_cluster_size: %d", euc_min_cluster_size);
  ROS_DEBUG("           euc_max_cluster_size: %d", euc_max_cluster_size);
  ROS_DEBUG("           occupancy_grid_width: %d", occupancy_grid_width);
  ROS_DEBUG("----------------occupancy_grid_size----------------------");
  ROS_DEBUG("          occupancy_grid_heigth: %d", occupancy_grid_height);
  ROS_DEBUG("            occupancy_grid_size: %d", occupancy_grid_size);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe (point_topic, 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  downsample_publisher = nh.advertise<sensor_msgs::PointCloud2> ("voxel_grid", 1);
  statistical_outlier_publisher = nh.advertise<sensor_msgs::PointCloud2>("statistical_outliers", 1);
  indices_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("indices_cloud", 1000);
  planar_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("planar_cloud", 1000);
  filtered_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("cloud_f", 1000);
  centroid_publisher = nh.advertise<pointcloud_obstacle_processing::PointIndicesArray>("centroids", 1);
  euc_cluster_publisher = nh.advertise<sensor_msgs::PointCloud2> ("euc_clusters", 5);
  occupancy_grid_publisher = nh.advertise<nav_msgs::OccupancyGrid> ("occupancy_grid", 1);

  // Spin
  ros::spin ();
}
