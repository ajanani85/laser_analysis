#include<ros/ros.h>
#include<cluster_finder/cluster.h>
#include<utility>
#include<vector>

using namespace std;


	Cluster::Cluster()
	{
		
	}
	Cluster::~Cluster()
	{
	
	}
	void Cluster::addMember(geometry_msgs::Point32 p)
	{
		members.push_back(p);
		sum_x += p.x;
		sum_y += p.y;
		sum_x2 += p.x * p.x;
		sum_y2 += p.y * p.y;
		sum_xy += p.x * p.y;
	}
	vector<geometry_msgs::Point32> &Cluster::getMembers()
	{
		return members;
	}
	size_t Cluster::size()
	{
		return members.size();
	}
	pair<double, double> Cluster::getTrend()
	{
		x_bar = sum_x / members.size();
		y_bar = sum_y / members.size();

		slope = (sum_xy - y_bar * sum_x - x_bar * sum_y + members.size() * x_bar * y_bar)		/	(sum_x2 - 2 * x_bar * sum_x + members.size() * x_bar * x_bar);	
		y_intercept = y_bar - slope * x_bar;
		
//		slope = (sum_xy - x_bar * sum_y - y_bar * sum_x + members.size() * y_bar * x_bar)		/	(sum_y2 - 2 * y_bar * sum_y + members.size() * y_bar * y_bar);	
//		y_intercept = x_bar - slope * y_bar;
		
		return std::make_pair(slope, y_intercept);   		
	}
	void Cluster::clear()
	{
		members.clear();
		slope = 0.0;
		y_intercept = 0.0;
		x_bar = 0.0;
		y_bar = 0.0;
		sum_x = 0.0;
		sum_y = 0.0;
		sum_x2 = 0.0;
		sum_xy = 0.0;
		leftBound = 0;
		rightBound = 0;
		distanceToLeftCluster = 0.0;
		distanceToRightCluster = 0.0;
	}
	double Cluster::getSlope()
	{
		return slope;
	}
	double Cluster::getYIntercept()
	{
		return y_intercept;
	}
