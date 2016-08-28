#ifndef OBSTACLEAVOIDANCE_H
#define OBSTACLEAVOIDANCE_H
#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/LaserScan.h>
using namespace std;
class ObstacleAvoidance
{
public:
	ros::NodeHandle nh_;
	ros::Subscriber laserScan_sub;
	sensor_msgs::LaserScan laserScan_msg;
	vector<float> laserAng;
	vector<float> laserRange;
	ObstacleAvoidance(ros::NodeHandle nh);
	~ObstacleAvoidance();
	void initSubscriber();
	
	//Callbacks
	void LaserScanCallback(const sensor_msgs::LaserScanConstPtr& msg);
  
};


#endif