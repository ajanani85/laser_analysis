#include<sensor_msgs/LaserScan.h>
#include<sensor_msgs/PointCloud.h>
#include<geometry_msgs/Point32.h>
#include<math.h>
using namespace std;

namespace laser_analysis
{
	void convertLaserScanToPointCloudRF(const sensor_msgs::LaserScan &l, sensor_msgs::PointCloud &pc)
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
	void convertLaserScanToPointCloudFL(const sensor_msgs::LaserScan &l, sensor_msgs::PointCloud &pc)
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
				p.y = l.ranges[i] * cos((i * angle_increment) + angle_min);
				p.x = l.ranges[i] * sin((i * angle_increment) + angle_min);
				p.z = 0.0;
				
				pc.points.push_back(p);
			}
		}		
	}
};
