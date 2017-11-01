#include "ros/ros.h"
#include "std_msgs/String.h"
#include <NASA_ARMS/PointIndicesArray.h>

class Point {
	float x, y, z, radius;
	int weight;
public:
	Point(float, float, float, float);
};

Point::Point(float x, float y, float z, float radius)
{
	weight = 0;
}


int counter;

std::vector <Point> centroid_holder;
NASA_ARMS::PointIndicesArray obstacle_list;

ros::Publisher pub;


void centroid_callback(const NASA_ARMS::PointIndicesArray& msg)
{
  std::cout << "first x: " << msg.points[0].x  << " first y: " << msg.points[0].y << " first z: " << msg.points[0].z << " first size: " << msg.points[0].r << std::endl;
	std::cout << "-----" << std::endl;
  /*
	counter += 1;
  if (counter == 10)
  {
    std::cout << "count: " << counter << " size: " << centroid_holder.size() << std::endl;
    counter = 0;
		pub.publish(msg);

  } else {
  	for (int i=0; i < 2; i++)
  	{
  		centroid_holder.push_back(Point(msg.points[i].x, msg.points[i].y, msg.points[i].z, msg.points[i].r));
  	}
  }
	*/
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "obstacle_processor");

  ros::NodeHandle n;

  counter = 0;

  ros::Subscriber sub = n.subscribe("centroids", 1000, centroid_callback);
	pub = n.advertise<NASA_ARMS::PointIndicesArray>("obstacles", 1);

  ros::spin();

  return 0;
}
