#ifndef OBSTACLEAVOIDANCE_H
#define OBSTACLEAVOIDANCE_H
#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include<stereo_msgs/DisparityImage.h>
#include <opencv2/opencv.hpp>
#include <dji_sdk/LocalPosition.h>
using namespace std;
class ObstacleAvoidance
{
public:
	ros::NodeHandle nh_;
	ros::Subscriber laserScan_sub;
	ros::Publisher hokuyo_data_pub;
	ros::Publisher guidance_data_pub;
	//--wxd--
	ros::Subscriber quadrotorPosNED_sub;
	ros::Subscriber obstacle_distance_sub;
	ros::Subscriber obstacle_disparity_sub;
	sensor_msgs::LaserScan laserScan_msg;
	sensor_msgs::LaserScan guidance_msg;
	stereo_msgs::DisparityImage disparity_msg;
    std_msgs::Float32MultiArray hokuyo_range_array;
 	std_msgs::Float32MultiArray hokuyo_angle_array;
 	std_msgs::Int16 hokuyo_obstacle_num;
 	std_msgs::Float32MultiArray guidance_range_array;
 	std_msgs::Int16MultiArray guidance_flag_array;
	vector<float> laserAng;
	vector<float> laserRange;
	cv::Mat_<cv::Vec3f> dense_points_;
	
	ObstacleAvoidance(ros::NodeHandle nh);
	~ObstacleAvoidance();
	void initSubscriber();
	
	//Callbacks
	void LaserScanCallback(const sensor_msgs::LaserScanConstPtr& msg);
	void guidanceObstacleCallback(const sensor_msgs::LaserScanConstPtr& g_oa);
	void guidancePointsCallback(const sensor_msgs::ImageConstPtr& disparity_img);
	void quadrotorPosNEDCallback(const dji_sdk::LocalPosition::ConstPtr &msg);
	//-whd- test
	float Object_N,Object_E;
	dji_sdk::LocalPosition quadrotorPosNED;
};
enum obstacle_state {none, guidance, hokuyo, hokuyo_and_guidance};
struct guidances{
	int obstacle_dir_flag[4];
	//float obstacleRange;
	//float obstacleAng;
};
struct hokuyos{
	int obstacle_Num;
	double obstacle_Range[2000];
	double obstacle_Ang[2000];
};

#define ARRAY_NUM 3
#endif