#include<ros/ros.h>
#include<cluster_extractor/cluster.h>
#include<sensor_msgs/PointCloud.h>
#include<utility>
#include<math.h>
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
		members.points.push_back(p);
		sum_x += p.x;
		sum_y += p.y;
		sum_x2 += p.x * p.x;
		sum_y2 += p.y * p.y;
		sum_xy += p.x * p.y;
	}
	void Cluster::getMembers(sensor_msgs::PointCloud &p)
	{
		p = members;
	}
	size_t Cluster::size()
	{
		return members.points.size();
	}
	pair<double, double> Cluster::getTrend()
	{
		x_bar = sum_x / members.points.size();
		y_bar = sum_y / members.points.size();

		slope = (sum_xy - y_bar * sum_x - x_bar * sum_y + members.points.size() * x_bar * y_bar)		/	(sum_x2 - 2 * x_bar * sum_x + members.points.size() * x_bar * x_bar);	
		y_intercept = y_bar - slope * x_bar;
		
		
//		slope = (sum_xy - x_bar * sum_y - y_bar * sum_x + members.size() * y_bar * x_bar)		/	(sum_y2 - 2 * y_bar * sum_y + members.size() * y_bar * y_bar);	
//		y_intercept = x_bar - slope * y_bar;
		angle_with_x_axis = (atan(slope) * 180.0 / M_PI) + 90.0;
		return std::make_pair(slope, y_intercept);   		
	}
	void Cluster::clear()
	{
		members.points.clear();
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
	double Cluster::getAngleWithXAxis()
	{
		return angle_with_x_axis;
	}
