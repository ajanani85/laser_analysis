#include<ros/ros.h>
#include<geometry_msgs/Point32.h>
#include<sensor_msgs/LaserScan.h>
#include<cluster_finder/Clusters.h>
#include<cluster_finder/cluster.h>
#include<laser_convertor/laser_convertor.h>
#include<sensor_msgs/PointCloud.h>
#include<iostream>
#include<utility>
#include<vector>
#include<string>

using namespace std;

string NODE_NAME = "cluster_finder";
string LASER_RAW_TOPIC = "/laser";
string CLUSTER_PUB_TOPIC = "/clusters";
int MIN_CLUSTER_SIZE = 0;
double MAX_CLUSTER_DISTANCE = 0.30;

ros::Publisher cluster_pub_;

void laserCB(const sensor_msgs::LaserScan::ConstPtr &scan)
{
	
}
int main(int argc, char** argv)
{
	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle nh;
	ros::NodeHandle nhp("~");
	
	nhp.getParam("min_cluster_size", MIN_CLUSTER_SIZE);
	ROS_INFO(CYAN "MIN_CLUSTER_SIZE = %d" COLOR_RESET, MIN_CLUSTER_SIZE);	
	
	nhp.getParam("max_cluster_distance", MAX_CLUSTER_DISTANCE);
	ROS_INFO(CYAN "MAX_CLUSTER_DISTANCE = %f" COLOR_RESET, MAX_CLUSTER_DISTANCE);
	
	nhp.getParam("cluster_pub_topic", CLUSTER_PUB_TOPIC);
	ROS_INFO(CYAN "CLUSTER_PUB_TOPIC = %s" COLOR_RESET, CLUSTER_PUB_TOPIC.c_str());
	
	nhp.getParam("laser_raw_topic", LASER_RAW_TOPIC);
	ROS_INFO(CYAN "LASER_RAW_TOPIC = %s" COLOR_RESET, LASER_RAW_TOPIC.c_str());

	ros::Subscriber laser_sub_ = nh.subscribe<sensor_msgs::LaserScan>(LASER_RAW_TOPIC.c_str(), 100, &laserCB);
	cluster_pub_ = nh.advertise<cluster_finder::Clusters>(CLUSTER_PUB_TOPIC.c_str(), 100);
		
	ros::spin();
	return 0;
}
