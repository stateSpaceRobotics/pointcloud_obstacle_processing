// TODO: Document
// TODO: Determine what needs to happen here
// TODO: make this publish whatever is needed for navigation
// TODO: make this work with a config file

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <NASA_ARMS/PointIndicesArray.h>

#include <cmath>

struct Point
{
    float x;
    float y;
    float z;
    float r;
    int weight;
};


void process_obstacles(const std::vector<NASA_ARMS::PointWithRad> &points);
void create_final_obstacle_list(NASA_ARMS::PointIndicesArray &final_obstacle_list);
int frame_counter;
int frame_history_len;
bool print_first_point;
float x_sim_tolarance;
float y_sim_tolerance;
float z_sim_tolerance;
float r_sim_tolerance;
float max_obstacle_rad;

int min_weight;

std::vector <Point> centroid_holder;


ros::Publisher obstacle_list_publisher;

using namespace std;


void centroid_callback(const NASA_ARMS::PointIndicesArray& msg)
/*  Called when a new array of centroids is available
 *  Will contain both above-ground obstacles and holes, if hole detection was enabled
 *
 *  Processes the centroids by converting them to Point structs.
 *  Keeps track of how many frames have been received, and then once
 *  frame_history_len frames have been processed, it publishes the list of
 *  obstacles in the custom PointIndicesArray message type.
 */
{
  if (print_first_point)
  {
    std::cout << "first x: " << msg.points[0].x  << " first y: " << msg.points[0].y << " first z: " << msg.points[0].z
              << " first size: " << msg.points[0].r << std::endl;
    std::cout << "-----" << std::endl;
  }

  frame_counter += 1;

  if (frame_counter == frame_history_len)
  {
    std::cout << "count: " << frame_counter << " size: " << centroid_holder.size() << std::endl;

    NASA_ARMS::PointIndicesArray final_obstacle_list;

    create_final_obstacle_list(final_obstacle_list);

    frame_counter = 0;
    obstacle_list_publisher.publish(final_obstacle_list);

  } else {

    cout << "frame: " << frame_counter << endl;
    process_obstacles(msg.points);

  }
}

void process_obstacles(const std::vector<NASA_ARMS::PointWithRad> &points) {
  /* Processes the obstacles from the PointWithRad array to determine whether
   * or not the point has been seen based on the sim tolerances set globally.
   *
   * If it has been seen before, we increment the weight value attached to it
   * to indicate how many frames it's appeared in.
   *
   * If it hasn't been seen before, we add it to the list of points with a weight of 1
   */
  if (centroid_holder.empty()) // if it's empty, default to adding all of them (first frame processed, probably)
  {
    for (auto& obstacle : points) {

      if (obstacle.r < max_obstacle_rad)
      {
        centroid_holder.emplace_back(Point{obstacle.x, obstacle.y, obstacle.z, obstacle.r, 1});
      }
    }

  } else {  // if it's not empty (read: not the first frame),

    for (auto& potential_obstacle : points) {  // compare the new points to the old points

      if (potential_obstacle.r < max_obstacle_rad)  // temporary solution to the edge detection in obstacle_detection marking the borders
      {
        bool found_previously = false;

        // TODO: It'd be nice if this could do some type of running average where the point values changed with additional input
        for (auto& previous_obstacle : centroid_holder)  // for each old point, if the new point is within a tolerance to it
        {
          if ((fabs(previous_obstacle.x - potential_obstacle.x) < x_sim_tolarance)
              && (fabs(previous_obstacle.y - potential_obstacle.y) < y_sim_tolerance)
              && (fabs(previous_obstacle.z - potential_obstacle.z) < z_sim_tolerance)
              && (fabs(previous_obstacle.r - potential_obstacle.r) < r_sim_tolerance))
          {
            previous_obstacle.weight += 1;  // mark the old point as having been 'seen' again
            found_previously = true;
          }
        }

        if (!found_previously)  // if it wasn't found in the previous list it must be a new point, so add it
        {
          centroid_holder.emplace_back(Point{potential_obstacle.x, potential_obstacle.y, potential_obstacle.z,
                                             potential_obstacle.r, 1});
        }
      }
    }
  }
}


void create_final_obstacle_list(NASA_ARMS::PointIndicesArray &final_obstacle_list)
/* Yay! We have processed sufficient frames so that the obstacle detection
 * can be seen as 'reliable' and 'believable' (lol)
 *
 * Anyways, this means that we must go back through the list of obstacles and throws
 * out any that weren't seen a sufficient number of times. Once those have been
 * eliminated, we are reasonably confident that the ones that remain are actually
 * obstacles, and not just noise.
 */
{
  for(auto& old_obstacle : centroid_holder )
  {
    if (old_obstacle.weight >= min_weight)  // if the obstacle has been seen the minimum amount of times, add it to the final array.
    {
      NASA_ARMS::PointWithRad published_obstacle;
      published_obstacle.x = old_obstacle.x;
      published_obstacle.y = old_obstacle.y;
      published_obstacle.z = old_obstacle.z;
      published_obstacle.r = old_obstacle.r;
      final_obstacle_list.points.push_back(published_obstacle);
    }
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "obstacle_processor");

  ros::NodeHandle n;

  frame_counter = 0;  // count of how many frames have been looked at so far
  frame_history_len = 1;  // how many frames to look at before publishing obstacle list?
  print_first_point = false;

  // how similar must a point be to another for them to be considered the same obstacle?
  x_sim_tolarance = 0.2;  // in meters
  y_sim_tolerance = 0.2;
  z_sim_tolerance = 0.2;
  r_sim_tolerance = 0.2;

  max_obstacle_rad = 0.5; // in meters

  min_weight = 1; // must appear in at least two frames to be considered an obstacle (TODO: Change to a percentage)

  ros::Subscriber sub = n.subscribe("centroids", 10, centroid_callback);
  obstacle_list_publisher = n.advertise<NASA_ARMS::PointIndicesArray>("obstacles", 1);

  ros::spin();

  return 0;
}
