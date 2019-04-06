//PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//ROS Includes
#include <ros/ros.h>
#include <ros/console.h>
#include <tf2_ros/transform_listener.h>

//STL Included
#include <map>
#include <string>

class pcp {
    public:
        pcp(); // conSTRUCTOR

        ~pcp(); // deSTRUCTOR(

        void init();

        void processingCB(const sensor_msgs::PointCloud2ConstPtr& receivedPointCloud);

    private:
        ros::NodeHandle nh;

        ros::Subscriber sub;

        ros::Publisher transformedCloudPublisher;
        ros::Publisher segmentedCloudPublisher;
        ros::Publisher filteredCloudPublisher;
        
        tf2_ros::TransformListener* tfListener;
        tf2_ros::Buffer tfBuffer;

        std::map<std::string, std::string> Params;

        
        void Publish(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, ros::Publisher publisher);

};