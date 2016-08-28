#include <ros/ros.h>
#include <obstacleAvoidance.h>
#include <iostream>

void ObstacleAvoidance::LaserScanCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
 	laserScan_msg.angle_increment = msg->angle_increment;
 	laserScan_msg.angle_max = msg->angle_max;
 	laserScan_msg.angle_min = msg->angle_min;
 	laserScan_msg.ranges = msg->ranges;
 	int lengthOfRanges = laserScan_msg.ranges.size();
	laserAng.reserve(lengthOfRanges);
	laserRange.reserve(lengthOfRanges);
	for(int i = 0;i != lengthOfRanges;i++)
	{
		laserRange.push_back(laserScan_msg.ranges[i]);
		laserAng.push_back(laserScan_msg.angle_min + i*laserScan_msg.angle_increment);
	}
	
}
ObstacleAvoidance::ObstacleAvoidance(ros::NodeHandle nh):nh_(nh)
{
	laserAng.clear();
	laserRange.clear();
	initSubscriber();
	
	
}

ObstacleAvoidance::~ObstacleAvoidance()
{

}



void ObstacleAvoidance::initSubscriber()
{
	laserScan_sub = nh_.subscribe("/scan", 2, &ObstacleAvoidance::LaserScanCallback, this);
}


