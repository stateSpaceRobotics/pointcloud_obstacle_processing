//ROS Includes
#include <ros/ros.h>
#include <ros/console.h>

//PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/transforms.h>

// Occupancy Grid
#include <nav_msgs/OccupancyGrid.h>

ros::Publisher mapPublisher;

//
int GetGridIndex(pcl::PointXYZ p, float mapBlockSize, float mapXMin, float mapYMax, int mapWidth)
{
    /*
    TODO:
    Calculate (and return) the index at which to place this point in the map grid.
    
    The tricky part is that the mapData is a 1-dimensional array, which necessitates more math than if it was 2-D.
    This isn't avoidable because the nav_msgs::OccupancyGrid wants a 1-D array. You can look through the old code for how to do the math.
    The variable is called occupancy_grid_data in the old code.
    
    Have fun!
    */

    int index;          //index of the point
    int xCount = 0;     //counter for x coordinate
    int yCount = 0;     //counter for y coordinate
    
    while (mapXMin + (xCount + 1)*mapBlockSize < p.x)   //run through the x coordinates on the map? also p.x may not be syntactically correct
    {
        xCount++;       
    }

    while (mapYMax - (yCount + 1)*mapBlockSize > p.y)   //run through the y coordinates on the map? also p.y may not be syntactically correct
    {
        yCount++;      
    }

    index = (yCount*mapWidth) + xCount;     //this is the math provided :]
    
    return index;
}

//This callback function receives the obstacle data from the point_cloud_processing node. We build and publish the map here.
void ObstacleCloudReceivedCallback(const sensor_msgs::PointCloud2ConstPtr& receivedPointCloud)
{
    float mapBlockSize = 0.15;  //The size of cells (in meters I believe)
    float mapXMin = 0.0;        //min x value (global)
    float mapXMax = 4.0;        //max x value (global)
    float mapYMin = 0.0;        //min y value (global)
    float mapYMax = 6.0;        //max y value (global)
    int mapWidth = (int)ceil((fabs(mapXMin) + fabs(mapXMax)) / mapBlockSize);   //Calculate the number of cells wide the map needs to be (ceil for safety)
    int mapHeight = (int)ceil((fabs(mapYMin) + fabs(mapYMax)) / mapBlockSize);  //Calculate the number of cells long the map needs to be (ceil for safety)
    int mapSize = mapHeight * mapWidth;     //Total number of cells in the map
    char* mapData = new char[mapSize];      //mapData that the OccupancyGrid message will send

    for(int i = 0; i < mapSize; i++) mapData[i] = 0;    //Cells are empty unless explicitly defined as not empty

    //HEY LOOK HERE AT THIS
    //MAKE SURE THIS IS NOT A TYPO
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*receivedPointCloud, *cloud);

    //This will loop over all of the points that correspond to obstacles.
    for(int i = 0; i < cloud->points.size(); i++)
    {
       int gridIndex = GetGridIndex(cloud->points[i], mapBlockSize, mapXMin, mapYMax, mapWidth);
       mapData[gridIndex] = 100;    //Really we should be doing probabilities, but for the sake of visualizing in RVIZ we set to 100 so that it shows up
    }

    //Configure the OccupancyGrid message
    nav_msgs::OccupancyGrid *map = new nav_msgs::OccupancyGrid();
    map->info.resolution = mapBlockSize;
    map->info.width = mapWidth;
    map->info.height = mapHeight;
    map->data = std::vector<int8_t>(mapData, mapData + mapSize);
    map->header.frame_id = "world";
    map->info.origin.orientation.w = 0;
    map->info.origin.orientation.z = 0;
    map->info.origin.orientation.y = 0;
    map->info.origin.orientation.x = 1;
    map->info.origin.position.x = 0;
    map->info.origin.position.y = mapYMax;
    map->info.origin.position.z = 0;

    //Publish the map
    mapPublisher.publish(*map);

    //Clean up your pointers kids!
    delete map;
    delete mapData;
}

//
int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "pointcloud_obstacle_detection");
    ros::NodeHandle nh ("~");
 
    //This is where we want to publish the map
    mapPublisher = nh.advertise<nav_msgs::OccupancyGrid>("map", 1);

    //The topic to subscribe to. From the point_cloud_processing node
    const char *obstacles_topic = "/point_cloud_processing/filteredCloud";

    // Create a ROS subscriber for the input point cloud that contains the obstacle points
    ros::Subscriber sub = nh.subscribe (obstacles_topic, 1, ObstacleCloudReceivedCallback);

    ros::spin ();
    return 0;
}