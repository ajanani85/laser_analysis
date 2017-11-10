#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>
#include<sensor_msgs/PointCloud.h>
#include<geometry_msgs/Point32.h>
#include<cluster_extractor/LaserClusters.h>

#include<math.h>
#include<utility>

#include<cluster_extractor/cluster.h>
#include<cluster_extractor/local.h>
#include<cluster_extractor/laser_analysis.h>
using namespace std;

//namespace laser_analysis
//{
	void laser_analysis::rectify(const sensor_msgs::LaserScan &src, sensor_msgs::LaserScan &des)
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
	
	int laser_analysis::getIndex(double angular_res, double beam_angle)
	{
		return ((beam_angle  +  45.0) / angular_res);
	}
	double laser_analysis::getAngle(double angular_res, int beam_index)
	{
		return ((beam_index * angular_res )- 45.0);
	}
	pair<int, int> laser_analysis::getBoundaries(const sensor_msgs::LaserScan &src, double frontFieldOfViewAngle)
	{
		double min_angle = (src.angle_min * 180.0 / M_PI);
		double max_angle = (src.angle_max * 180.0 / M_PI);
		double angle_increment = src.angle_increment * 180.0 / M_PI;
		double correctAng = (max_angle - min_angle) - 180.0;
		
		return make_pair(getIndex(angle_increment, (correctAng + (frontFieldOfViewAngle / 2 ))), getIndex(angle_increment, (correctAng - (frontFieldOfViewAngle / 2))));
	}
	void laser_analysis::convertLaserScanToPointCloud(const sensor_msgs::LaserScan &l, sensor_msgs::PointCloud &pc)
	{
		pc.header = l.header;
		
		int range_size = (int)l.ranges.size();
		double angle_increment = l.angle_increment;
		double angle_min = l.angle_min;
		double max_range =l.range_max;	
		double min_range = l.range_min;
		
		for (int i = 0; i < l.ranges.size(); i++)
		{
			if(l.ranges[i] >= min_range && l.ranges[i] <= max_range)
			{
				geometry_msgs::Point32 p;
				p.x = l.ranges[i] * cos((i * angle_increment) + angle_min);
				p.y = l.ranges[i] * sin((i * angle_increment) + angle_min);
				p.z = 0.0;
			
				pc.points.push_back(p);
			}
		}
	}
	void laser_analysis::convertLaserScanToPointCloud(const sensor_msgs::LaserScan &l, sensor_msgs::PointCloud &pc, bool rangeFilterIsOn, double maxRange, double front_field_of_view)
	{
		pc.header = l.header;
	
		pair<int, int> boundaries = getBoundaries(l, front_field_of_view);
		
		int range_size = (int)l.ranges.size();
		double angle_increment = l.angle_increment;
		double angle_min = l.angle_min;
		
		double max_range = 0.0;	
		double min_range = l.range_min;
		if(rangeFilterIsOn)
		{
			max_range = maxRange;
		} 
		else 
		{
			max_range =  l.range_max;
		}

		for (int i = boundaries.second; i <= boundaries.first; i++)
		{
			if(l.ranges[i] >= min_range && l.ranges[i] <= max_range)
			{
				geometry_msgs::Point32 p;
				p.x = l.ranges[i] * cos((i * angle_increment) + angle_min);
				p.y = l.ranges[i] * sin((i * angle_increment) + angle_min);
				p.z = 0.0;
			
				pc.points.push_back(p);
			}
		}		
	}
	
	void laser_analysis::getClusters(const sensor_msgs::PointCloud &src, vector<Cluster> &res, double max_cluster_distance, int min_cluster_size)
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
	void laser_analysis::clustersSizeFilter(vector<Cluster> &src, vector<Cluster> &des, int size_th)
	{
		for(int i = 0; i < src.size(); i++)
		{
			if((int)src[i].size() >= size_th)
			{
				des.push_back(src[i]);
			}
		}
	}
	void laser_analysis::getLargestCluster(vector<Cluster> &src, Cluster &des)
	{
		int size = 0;
		int index = 0;
		for(int i = 0; i < src.size(); i++)
		{
			if((int)src[i].size() > size)
			{
				index = i;
			}
		}
		des = src[index];
	}
	void laser_analysis::publishClusters( vector<Cluster> &src, ros::Publisher &pub, const string &frame_id, int seq, double max_cluster_distance, int min_cluster_size)
	{
		cluster_extractor::LaserClusters lcs;
		lcs.header.frame_id =frame_id;
		lcs.header.seq = seq;
		lcs.header.stamp = ros::Time::now();
		lcs.max_cluster_distance = max_cluster_distance;
		lcs.min_cluster_size = min_cluster_size;		
		
		for(int i = 0; i < src.size(); i++)
		{
			cluster_extractor::LaserCluster lc;
			sensor_msgs::PointCloud p; 
			src[i].getMembers(p);
			lc.points = p.points;
			lc.rightBound = src[i].rightBound;
			lc.leftBound = src[i].leftBound;
			lc.distanceToLeftCluster = src[i].distanceToLeftCluster;
			lc.distanceToRightCluster = src[i].distanceToRightCluster;
			lc.slope = src[i].getSlope();
			lc.y_intercept = src[i].getYIntercept();	
			lc.clusterIndex = i;
			lc.angle_with_x_axis = src[i].getAngleWithXAxis();
						
			lcs.clusters.push_back(lc);				
		}
		pub.publish(lcs);
	}
	void laser_analysis::searchForLocals(const sensor_msgs::PointCloud &src, vector<Local> &des, int n_search, bool searchForMinima)
	{
		for(int cnt = n_search + 1; cnt < src.points.size() - n_search; cnt++)
		{
				vector<geometry_msgs::Point32> temp;
				bool isItLocal = true;
				geometry_msgs::Point32 p = src.points[cnt];
				double mag = sqrt((p.x)*(p.x)		+		(p.y)*(p.y));
			
				int lowBound = cnt - (n_search / 2);
				int highBound = cnt + (n_search / 2);
			
				for(int i = lowBound; i <= highBound; i++)
				{
					if(i == cnt) continue;
					
					geometry_msgs::Point32 p_i = src.points[i];
					temp.push_back(p_i);
					double nmag = sqrt((p_i.x)*(p_i.x)		+		(p_i.y)*(p_i.y));
					if(searchForMinima && mag > nmag)
					{
						isItLocal = false;
						break;
					}
					if(!searchForMinima && mag < nmag)
					{
						isItLocal = false;
						break;
					}
				}
				if(isItLocal )
				{
					Local lm;
					lm.setLocal(temp, searchForMinima);
					des.push_back(lm);
				}
		}
	}
	void laser_analysis::searchForLocals(Cluster &src, vector<Local> &des, int n_search, bool searchForMinima)
	{
		sensor_msgs::PointCloud members;
		src.getMembers(members);
		for(int cnt = n_search + 1; cnt < (int) src.size() - n_search; cnt++)
		{
			vector<geometry_msgs::Point32> temp;
			bool isItLocal = true;
			geometry_msgs::Point32 p = members.points[cnt];
			double mag = sqrt((p.x)*(p.x)		+		(p.y)*(p.y));
			
			
			int lowBound = cnt - (n_search / 2);
			int highBound = cnt + (n_search / 2);
			for(int i = lowBound; i <= highBound; i++)
			{
				if(i == cnt) continue;
				geometry_msgs::Point32 p_i = members.points[i];
				temp.push_back(p_i);
				double nmag = sqrt((p_i.x)*(p_i.x)		+		(p_i.y)*(p_i.y));
				if(searchForMinima && mag > nmag)
				{
					isItLocal = false;
					break;
				}
				if(!searchForMinima && mag < nmag)
				{
					isItLocal = false;
					break;
				}
			}
			if(isItLocal )
			{
				Local lm;
				lm.setLocal(temp, searchForMinima);
				des.push_back(lm);
			}
		}
	}
	void laser_analysis::AverageMagnitudeFilter( vector<Local> &src, vector<Local> &res, double avg_coef_limit)
	{
		double avg = 0.0;
		for(Local local : src)
		{
			avg += local.getMagnitude();
		}
		avg = avg / src.size();
	
		for(int i = 0; i < src.size(); i++)
		{
			if(src[i].getMagnitude() >= avg *avg_coef_limit)
			{
				res.push_back(src[i]);
			}
		}
	}
	void laser_analysis::neighbourDistanceFilter(vector<Local> &src, vector<Local> &res, double ThresholdDistance, double Error_Th)
	{
		double sm_th = 0.1;
		for (int i = 0; i < src.size(); i++)
		{
			int inRangeNeighbours = 0;
			for(int j = 0; j < src.size(); j++)
			{
				if(i == j) continue;
			
				double n_dist = abs(src[i].centre.y - src[j].centre.y);
				//double n_dist = abs(p.points[src[i].pc_index].y - p.points[src[j].pc_index].y);		
				if((n_dist <= ThresholdDistance + Error_Th &&  n_dist >= ThresholdDistance - Error_Th) ||
					 (n_dist <= 2*(ThresholdDistance + Error_Th) &&  n_dist >= 2*(ThresholdDistance - Error_Th)))	
				{
					inRangeNeighbours++;
				}
			}
			if(inRangeNeighbours >= 1) 
			{
				src[i].inRangeNeighbours = inRangeNeighbours;
				res.push_back(src[i]);
			}
		}
	}
//}


