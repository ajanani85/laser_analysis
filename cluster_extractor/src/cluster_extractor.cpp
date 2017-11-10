#include<cluster_extractor/cluster.h>
#include<cluster_extractor/laser_analysis.h>
#include<ros/ros.h>
#include<sensor_msgs/PointCloud.h>
#include<sensor_msgs/LaserScan.h>
#include<cluster_extractor/LaserClusters.h>
#include<string>
#include<vector>

using namespace std;

string NODE_NAME = "cluster_extractor";
string LASER_TOPIC = "/scan";
string CLUSTER_TOPIC = "/clusters";
double MAX_CLUSTER_DISTANCE = 0.30; //meters
int MIN_CLUSTER_SIZE = 50;

ros::Publisher cluster_pub_ ;

void laserCB(const sensor_msgs::LaserScan::ConstPtr &scan)
{
	ros::Time start = ros::Time::now();
	
	vector<Cluster> clusters;
	
	sensor_msgs::PointCloud pc;
	laser_analysis::convertLaserScanToPointCloud(*scan, pc);
	laser_analysis::getClusters(pc, clusters, MAX_CLUSTER_DISTANCE, MIN_CLUSTER_SIZE);
	laser_analysis::publishClusters(clusters, cluster_pub_, scan->header.frame_id, scan->header.seq, MAX_CLUSTER_DISTANCE, MIN_CLUSTER_SIZE);
	
	//ROS_INFO(GREEN "Laser size: %d, PointCloud size: %d" COLOR_RESET, (int)scan->ranges.size(), (int)pc.points.size());
	
	ROS_INFO(RED "Processing Time: %f ms" COLOR_RESET, (float)(ros::Time::now().toNSec() - start.toNSec()) / 1000000.00);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, NODE_NAME.c_str());
	ros::NodeHandle nh_;
	ros::NodeHandle nhp_("~");
	
	nhp_.getParam("laser_topic", LASER_TOPIC);
	ROS_INFO(CYAN "LASER_TOPIC: %s" COLOR_RESET, LASER_TOPIC.c_str());
	
	nhp_.getParam("cluster_topic", CLUSTER_TOPIC);
	ROS_INFO(CYAN "CLUSTER_TOPIC: %s" COLOR_RESET, CLUSTER_TOPIC.c_str());
	
	nhp_.getParam("max_cluster_distance", MAX_CLUSTER_DISTANCE);
	ROS_INFO(CYAN "MAX_CLUSTER_DISTANCE: %f" COLOR_RESET, MAX_CLUSTER_DISTANCE);
	
	nhp_.getParam("min_cluster_size", MIN_CLUSTER_SIZE);
	ROS_INFO(CYAN "MIN_CLUSTER_SIZE: %d" COLOR_RESET, MIN_CLUSTER_SIZE);
	
	ros::Subscriber laser_sub_ = nh_.subscribe<sensor_msgs::LaserScan>(LASER_TOPIC.c_str(), 100, &laserCB);
	cluster_pub_ = nh_.advertise<cluster_extractor::LaserClusters>(CLUSTER_TOPIC.c_str(), 100);
	
	ros::spin();
	return 0;
}
