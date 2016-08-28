#include <ros/ros.h>
#include <iostream>
#include <obstacleAvoidance.h>
using namespace std;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "obstacleAvoidance_node");
	ros::NodeHandle nh;
	ObstacleAvoidance CObstacleAvoidance(nh);
	return 0;
}
