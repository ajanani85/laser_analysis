#include<ros/ros.h>
#include<cluster_finder/cluster.h>
#include<utility>
#include<vector>

using namespace std;

namespace laser_analysis
{
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
		sum_xy += p.x * p.y;
	}
	vector<geometry_msgs::Point32> &Cluster::getMembers()
	{
		return members;
	}
	int Cluster::memberSize()
	{
		return (int)(members.size());
	}
	pair<double, double> Cluster::getTrend()
	{
		x_bar = sum_x / members.size();
		y_bar = sum_y / members.size();

		slope = (sum_xy - y_bar * sum_x - x_bar * sum_y + members.size() * x_bar * y_bar)		/	(sum_x2 - 2 * x_bar * sum_x + members.size() * x_bar * x_bar);	
		y_intercept = y_bar - slope * x_bar;
		
		return std::make_pair(slope, y_intercept);   		
	}
}
