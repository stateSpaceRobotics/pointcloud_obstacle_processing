//ROS Includes
#include <ros/ros.h>
#include <ros/console.h>

//Class Include
#include "../include/pcp.hpp"


//
int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "pointcloud_obstacle_detection");
    
    pcp* processor = new pcp();

    ros::spin ();
    return 0;
}