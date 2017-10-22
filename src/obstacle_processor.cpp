#include "ros/ros.h"
#include "std_msgs/String.h"
#include <NASA_ARMS/PointIndicesArray.h>

void centroid_callback(const NASA_ARMS::PointIndicesArray& msg)
{
  std::cout << "first x: " << msg.points[0].x  << "first y: " << msg.points[0].y << "first z: " << msg.points[0].z << "first size: " << msg.points[0].r << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "obstacle_processor");
  
  ros::NodeHandle n;
  
  ros::Subscriber sub = n.subscribe("centroids", 1000, centroid_callback);
  
  ros::spin();
  
  return 0;
}
