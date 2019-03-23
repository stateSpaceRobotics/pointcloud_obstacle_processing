//Parameter Structs
#include "../include/Parameters.h"

//PCL specific includes
#include <sensor_msgs/PointCloud2.h>

//ROS Includes
#include <ros/ros.h>
#include <ros/console.h>

//STL Included
#include <map>
#include <string>

class pcp {
    public:
        pcp(); // conSTRUCTOR

        ~pcp(); // deSTRUCTOR(

        void init();
    private:
        ros::NodeHandle nh;

        ros::Subscriber sub;

        std::map<std::string, std::string> Params;

        void processingCB(const sensor_msgs::PointCloud2ConstPtr& receivedPointCloud);
};