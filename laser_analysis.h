#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>
#include<sensor_msgs/PointCloud.h>
#include<geometry_msgs/Point32.h>
#include<brain_box_msgs/LaserClusters.h>
#include<lidar/cluster.h>
#include<math.h>
using namespace std;

namespace laser_analysis
{
	inline void rectify(const sensor_msgs::LaserScan &src, sensor_msgs::LaserScan &des)
	{
		des = src;
		for(int i = 0; i < des.ranges.size(); i++)
		{
			if(des.ranges[i] < des.range_min)
			{
				des.ranges[i] = des.range_min;
			} 
			else if(des.ranges[i] > des.range_max)
			{
				des.ranges[i] = des.range_max;
			}
		}
	}
	inline void convertLaserScanToPointCloud(const sensor_msgs::LaserScan &l, sensor_msgs::PointCloud &pc)
	{
		pc.header = l.header;
	
		int range_size = (int)l.ranges.size();
		double angle_increment = l.angle_increment;
		double angle_min = l.angle_min;
	
		for (int i = 0; i < range_size; i++)
		{
			if(l.ranges[i] >= l.range_min && l.ranges[i] <= l.range_max)
			{
				geometry_msgs::Point32 p;
				p.x = l.ranges[i] * cos((i * angle_increment) + angle_min);
				p.y = l.ranges[i] * sin((i * angle_increment) + angle_min);
				p.z = 0.0;
			
				pc.points.push_back(p);
			}
		}		
	}

	inline void getClusters(const sensor_msgs::PointCloud &src, vector<Cluster> &res, double max_cluster_distance, int min_cluster_size)
	{
		Cluster l_cluster;
		l_cluster.distanceToRightCluster = 0.0;
		for(int i = 0; i < src.points.size() - 1; i++)
		{
			geometry_msgs::Point32 p = src.points[i];
			geometry_msgs::Point32 pn = src.points[i+1];
			double dx = p.x - pn.x;
			double dy = p.y - pn.y;
			double distanceToNeighbour = sqrt(dx*dx + dy*dy);
			if(distanceToNeighbour < max_cluster_distance)
			{
				//This is the right index
				if(l_cluster.size() == 0)
				{
					l_cluster.rightBound = i;
					l_cluster.addMember(p);
				}
				if(i == src.points.size() - 2 )
				{
					l_cluster.leftBound = i;
					l_cluster.distanceToLeftCluster = 0.0;
					l_cluster.getTrend();
					l_cluster.addMember(p);
					if(l_cluster.size() >= min_cluster_size)
					{
						res.push_back(l_cluster);
					}
				
				}
				else
				{
					l_cluster.addMember(p);
				}
			}
			else
			{
				l_cluster.leftBound = i;
				l_cluster.addMember(p);
				l_cluster.distanceToLeftCluster = distanceToNeighbour;
				l_cluster.getTrend();
			
				if(l_cluster.size() >= min_cluster_size)
				{
					res.push_back(l_cluster);
				}
				//reseting the local cluster
				l_cluster.clear();
				l_cluster.rightBound = i+1;
				l_cluster.distanceToRightCluster = distanceToNeighbour;
			}
		}
	}
	
	inline void publishClusters( vector<Cluster> &src, ros::Publisher &pub, const string &frame_id, int seq)
	{
		brain_box_msgs::LaserClusters lcs;
		lcs.header.frame_id =frame_id;
		lcs.header.seq = seq;
		lcs.header.stamp = ros::Time::now();
		
		for(int i = 0; i < src.size(); i++)
		{
			brain_box_msgs::LaserCluster lc;
			sensor_msgs::PointCloud p; 
			src[i].getMembers(p);
			lc.points = p.points;
			lc.rightBound = src[i].rightBound;
			lc.leftBound = src[i].leftBound;
			lc.distanceToLeftCluster = src[i].distanceToLeftCluster;
			lc.distanceToRightCluster = src[i].distanceToRightCluster;
			lc.slope = src[i].getSlope();
			lc.y_intercept = src[i].getYIntercept();	
			
			lcs.clusters.push_back(lc);				
		}
		pub.publish(lcs);
	}
}


