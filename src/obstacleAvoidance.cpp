#include <ros/ros.h>
#include <obstacleAvoidance.h>
#include <iostream>
#include<obstacle_avoidance/Hokuyo.h>
#include<stdio.h>
int laser_count = 0;
#define THRESHOLD_VALUE 3
void ObstacleAvoidance::LaserScanCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
	hokuyos obstacle_dis_hok;  //定义hokuyo输出的信息结构体变量
	obstacle_dis_hok.obstacle_Num = 0;   //障碍物的个数
 	laserScan_msg.angle_increment = msg->angle_increment;
 	laserScan_msg.angle_max = msg->angle_max;
 	laserScan_msg.angle_min = msg->angle_min;
 	laserScan_msg.ranges = msg->ranges;
	const unsigned int data_sz = 5;
	//最多五个障碍物，定义长度为5的数组
	obstacle_avoidance::Hokuyo hokuyo_data;
	hokuyo_data.ranges.resize(data_sz);
	hokuyo_data.angles.resize(data_sz);
  	hokuyo_data.ranges[0] = 999;
	hokuyo_data.ranges[1] = 999;
	hokuyo_data.ranges[2] = 999;
	hokuyo_data.ranges[3] = 999;
	hokuyo_data.ranges[4] = 999;
	
	hokuyo_data.angles[0] = 999;
	hokuyo_data.angles[1] = 999;
	hokuyo_data.angles[2] = 999;
	hokuyo_data.angles[3] = 999;
	hokuyo_data.angles[4] = 999;
	hokuyo_data.number = 0;
	int obtacle_count = 1, unobstacle_count = 0;
	int obstacle_flag = 0; //障碍物判断标志位
	int org_num = 0, end_num = 0; //障碍物第一个和最后一个位置
	float obstacle_rate = 0;
 	int lengthOfRanges = laserScan_msg.ranges.size();   //获取hokuyo节点发来的信息
	//定义数据数组
	double all_ranges[5][lengthOfRanges], filter_ranges[lengthOfRanges], laser_Ang[5][lengthOfRanges], ang_ranges[lengthOfRanges];
	for(int i = 0;i != lengthOfRanges;i++){
		//距离小于1.5米就表示有障碍物
		if (laserScan_msg.ranges[i] < THRESHOLD_VALUE){
			all_ranges [laser_count][i] = laserScan_msg.ranges[i];
			laser_Ang[laser_count][i] = laserScan_msg.angle_min + i*laserScan_msg.angle_increment;
		}
		//没障碍物否则赋值999
		else{
			all_ranges [laser_count][i] = 999;
			laser_Ang[laser_count][i] = laserScan_msg.angle_min + i*laserScan_msg.angle_increment;
		}
	}
	//中值滤波
	if(laser_count == ARRAY_NUM -1){
		laser_count = 0;
		//对三组数据的每个元素进行排序得到三组按测得距离顺序排的数组
 		for(int i = 0;i < lengthOfRanges; i++){
			double range_temp, ang_temp;
			for(int j = 0; j < ARRAY_NUM-1; j++) //冒泡排序法
				for(int k = 0; k <ARRAY_NUM-1-j; k++){
					if( all_ranges[k][i] > all_ranges[k+1][i]){
						range_temp = all_ranges[k][i];
						//ROS_INFO("I am 4!");
						all_ranges[k][i] = all_ranges[k+1][i];
						all_ranges[k+1][i] = range_temp;
						
						ang_temp = laser_Ang[k][i];
						laser_Ang[k][i] = laser_Ang[k+1][i];
						laser_Ang[k+1][i] = ang_temp;
				}
			}
			//选取中间一组作为最终的数据
			filter_ranges[i] = all_ranges[1][i];
			ang_ranges[i] = laser_Ang[1][i];
			
			//有障碍物，开始障碍物点计数
			if (filter_ranges[i] < THRESHOLD_VALUE){  //为了便于测试，将距离改为10厘米
				if(obtacle_count == 1)
					org_num = i;
				else if(obtacle_count > 10){
					obstacle_flag = 1;  //障碍物点大于10个点才有效
				}  //当小于点数目大于10个点时候就表示有效的障碍物
				if(unobstacle_count != 0){
					unobstacle_count = 0;
				}  //如果非障碍物点不为0且又进入障碍物点的计数说明是噪声点
				obtacle_count = obtacle_count + 1;
			}
			//当障碍物在扫描周期的末端时，非障碍物的点数为0，这时候强制障碍物赋值条件有效即可
			if(lengthOfRanges - i < 20 && obstacle_flag)
				obstacle_rate = 1; 
			//障碍物点标志位有效，且距离突变为999
		    if(filter_ranges[i] == 999  && obstacle_flag ){
				unobstacle_count = unobstacle_count + 1;
				obstacle_rate = unobstacle_count / (obtacle_count * 1.0); 
				if(obstacle_rate >= 1){  //当非障碍物点与障碍物点相同时，认为上一轮障碍物点有效
					end_num = (obtacle_count + 2 * org_num) / 2;  //取障碍物中间的值
					while(filter_ranges[end_num] > THRESHOLD_VALUE){
						end_num = end_num + 1;
					}//如果中间值是个噪声点就取旁边的值
					obstacle_dis_hok.obstacle_Range[obstacle_dis_hok.obstacle_Num] = filter_ranges[end_num];
					obstacle_dis_hok.obstacle_Ang[obstacle_dis_hok.obstacle_Num] = ang_ranges[end_num];
					hokuyo_data.ranges[obstacle_dis_hok.obstacle_Num]  =  obstacle_dis_hok.obstacle_Range[obstacle_dis_hok.obstacle_Num];
				    hokuyo_data.angles[obstacle_dis_hok.obstacle_Num] = obstacle_dis_hok.obstacle_Ang[obstacle_dis_hok.obstacle_Num];
					obstacle_dis_hok.obstacle_Num = obstacle_dis_hok.obstacle_Num + 1; //障碍物个数，可用于作为标志位
					hokuyo_data.number = obstacle_dis_hok.obstacle_Num;
					obstacle_flag = 0;
					obtacle_count = 1; 
					org_num = 0;
					unobstacle_count = 0; //第一个障碍物已判断完成，标志位和计算点均清零。
					//ROS_INFO("I am here!!!");
					//printf("obstacle_dis_hok.obstacle_Num=%d\n", obstacle_dis_hok.obstacle_Num); 
				}
			}
		}
		//ROS_INFO("number %d",hokuyo_data.number);
		//-whd-for test
		hokuyo_data.number = 1;
		hokuyo_data.ranges[0] = 1.5;
		hokuyo_data.angles[0] = 3.1415926/2.0;
		//hokuyo_data_pub.publish(hokuyo_data);
	}
	else
		//ROS_INFO("whd_test3");
		laser_count++;
}
//-whd_testuse-
void ObstacleAvoidance::quadrotorPosNEDCallback(const dji_sdk::LocalPosition::ConstPtr &msg)
{
	quadrotorPosNED.x = msg->x;
	quadrotorPosNED.y = msg->y;
	quadrotorPosNED.z = msg->z;
	obstacle_avoidance::Hokuyo hokuyo_data;
	hokuyo_data.number = 0;
	hokuyo_data.ranges.resize(5);
	hokuyo_data.angles.resize(5);
	for(int i=0;i<5;i++)
	{
		hokuyo_data.ranges[i] = 999;
		hokuyo_data.angles[i] = 999;
	}
	//ROS_INFO("OB: %3.1f,%3.1f",Object_N, Object_E);
	float range = sqrt((Object_N - quadrotorPosNED.x)*(Object_N - quadrotorPosNED.x)+(Object_E - quadrotorPosNED.y)*(Object_E - quadrotorPosNED.y));
	float angle = 3.1415926 - atan2((Object_E - quadrotorPosNED.y),(Object_N - quadrotorPosNED.x));
	if (range <3.0)
	{
		hokuyo_data.number = 1;
		hokuyo_data.ranges[0] = range;
		hokuyo_data.angles[0] = angle;
	}
	hokuyo_data_pub.publish(hokuyo_data);

}



//--wxd,guidance data--
void ObstacleAvoidance::guidanceObstacleCallback(const sensor_msgs::LaserScanConstPtr& g_oa)
{	
	guidances obstacle_dis_gui;
	obstacle_avoidance::Hokuyo guidance_data;
	const int guidance_sz = 4;
	int obstacle_number = 0;
	guidance_data.ranges.resize(guidance_sz);
	guidance_data.angles.resize(guidance_sz);
	obstacle_dis_gui.obstacle_dir_flag[0] = 0;
	obstacle_dis_gui.obstacle_dir_flag[1] = 0;
	obstacle_dis_gui.obstacle_dir_flag[2] = 0;
	obstacle_dis_gui.obstacle_dir_flag[3] = 0;//判断那个guindance有障碍物
	guidance_msg.ranges = g_oa->ranges;
	guidance_msg.angle_increment = g_oa->angle_increment;
	if (guidance_msg.ranges[1] < 2)
	{
		obstacle_dis_gui.obstacle_dir_flag[0] = 1;
		obstacle_number++;
	}
	else
		guidance_msg.ranges[1] = 999;
	
	if (guidance_msg.ranges[2] < 2)
	{
		obstacle_dis_gui.obstacle_dir_flag[1] = 1;
		obstacle_number++;
	}
	else
		guidance_msg.ranges[2] = 999;
	
	if (guidance_msg.ranges[3] <2 )
	{
		obstacle_dis_gui.obstacle_dir_flag[2] = 1;
		obstacle_number++;
	}
	else
		guidance_msg.ranges[3] = 999;
	
	if (guidance_msg.ranges[4] < 2)
	{
		obstacle_dis_gui.obstacle_dir_flag[3] = 1;
		obstacle_number++;
	}
	else
		guidance_msg.ranges[4] = 999;

	guidance_data.number = obstacle_number;

	guidance_data.angles[0] = obstacle_dis_gui.obstacle_dir_flag[0];
	guidance_data.angles[1] = obstacle_dis_gui.obstacle_dir_flag[1];
	guidance_data.angles[2] = obstacle_dis_gui.obstacle_dir_flag[2];
	guidance_data.angles[3] = obstacle_dis_gui.obstacle_dir_flag[3];
	
	guidance_data.ranges[0] = guidance_msg.ranges[1];
	guidance_data.ranges[1] = guidance_msg.ranges[2];
	guidance_data.ranges[2] = guidance_msg.ranges[3];
	guidance_data.ranges[3] = guidance_msg.ranges[4];
	
	guidance_data_pub.publish(guidance_data);
	
}
//--wxd--
void ObstacleAvoidance::guidancePointsCallback(const sensor_msgs::ImageConstPtr& disparity_img){
	//const sensor_msgs::Image& dimage = disparity_img;
	//const cv::Mat_<float> dmat(dimage.height, dimage.width, (float*)&dimage.data[0], dimage.step);
	//model.projectDisparityImageTo3d(dmat, dense_points_, true);
	
}
ObstacleAvoidance::ObstacleAvoidance(ros::NodeHandle nh):nh_(nh),nh_param("~")
{
	if(!nh_param.getParam("Object_N", Object_N))Object_N = 10.0;
	if(!nh_param.getParam("Object_E", Object_E))Object_E = 2.0;
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
	obstacle_distance_sub = nh_.subscribe("/guidance/obstacle_distance", 1, &ObstacleAvoidance::guidanceObstacleCallback, this);
	obstacle_disparity_sub = nh_.subscribe("/guidance/disparity", 1, &ObstacleAvoidance::guidancePointsCallback, this);
	quadrotorPosNED_sub = nh_.subscribe("/dji_sdk/local_position", 10, &ObstacleAvoidance::quadrotorPosNEDCallback, this);
	hokuyo_data_pub = nh_.advertise<obstacle_avoidance::Hokuyo>("/hokuyo/obstacle_data", 10);
	guidance_data_pub = nh_.advertise<obstacle_avoidance::Hokuyo>("/guidance/obstacle_data", 10);
} 