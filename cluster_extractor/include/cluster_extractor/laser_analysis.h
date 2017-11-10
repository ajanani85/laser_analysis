#ifndef LASER_ANALYSIS_H_
#define LASER_ANALYSIS_H_

#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>
#include<sensor_msgs/PointCloud.h>
#include<geometry_msgs/Point32.h>
#include<cluster_extractor/LaserClusters.h>
#include<cluster_extractor/cluster.h>
#include<cluster_extractor/local.h>
#include<math.h>
#include<utility>
using namespace std;

namespace laser_analysis
{
	void rectify(const sensor_msgs::LaserScan &src, sensor_msgs::LaserScan &des);
	int getIndex(double angular_res, double beam_angle);
	double getAngle(double angular_res, int beam_index);
	pair<int, int> getBoundaries(const sensor_msgs::LaserScan &src, double frontFieldOfViewAngle);
	void convertLaserScanToPointCloud(const sensor_msgs::LaserScan &l, sensor_msgs::PointCloud &pc, bool rangeFilterIsOn, double maxRange, double front_field_of_view);
	void convertLaserScanToPointCloud(const sensor_msgs::LaserScan &l, sensor_msgs::PointCloud &pc);
	void getClusters(const sensor_msgs::PointCloud &src, vector<Cluster> &res, double max_cluster_distance, int min_cluster_size);
	void clustersSizeFilter(vector<Cluster> &src, vector<Cluster> &des, int size_th);
	void getLargestCluster(vector<Cluster> &src, Cluster &des);
	void publishClusters( vector<Cluster> &src, ros::Publisher &pub, const string &frame_id, int seq, double max_cluster_distance, int min_cluster_size);
	void searchForLocals(const sensor_msgs::PointCloud &src, vector<Local> &des, int n_search, bool searchForMinima);
	void searchForLocals(Cluster &src, vector<Local> &des, int n_search, bool searchForMinima);
	void AverageMagnitudeFilter( vector<Local> &src, vector<Local> &res, double avg_coef_limit);
	void neighbourDistanceFilter(vector<Local> &src, vector<Local> &res, double ThresholdDistance, double Error_Th);
	template <class T>
	inline vector<T> &getSubVector(vector<T> &src, int start_index, int end_index)
	{
		vector<T> result;
		for(int i = start_index; i <= end_index; i++)
		{
			result.push_back(src[i]);
		}
		return result;
	}
}
#endif

