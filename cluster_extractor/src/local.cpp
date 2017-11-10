#include<vector>
#include<sensor_msgs/PointCloud.h>
#include<geometry_msgs/Point32.h>
#include<cluster_extractor/local.h>

using namespace std;

namespace laser_analysis
{
	Local::Local()
	{

	}
	Local::~Local()
	{
		
	}
	void Local::setLocal(geometry_msgs::Point32 &p , bool  is_minima)
	{
			isMinima = is_minima;
			centre = p;
			setMaxMin(centre);
	}	
	void Local::setLocal(vector<geometry_msgs::Point32> &src, bool is_minima)
	{
		isMinima = is_minima;
		centre = src[(src.size() / 2) + 1];
		setMaxMin(centre);
		for(int i = 0; i < src.size(); i++)
		{
			if (i == (src.size() / 2) + 1 ) continue;
			
			members.push_back(src[i]);
			
			double mag = sqrt(src[i].x * src[i].x + src[i].y * src[i].y);
			if(isMinima)
			{
				if(mag > max) 
				{
					max = mag;
				}
			}
			else
			{
				if(mag < min) 
				{
					min = mag;
				}
			}
		}
		
		magnitude = max - min;
		
	}
	void Local::addMembers(vector<geometry_msgs::Point32> &src, bool is_minima)
	{
		isMinima = is_minima;
		members = src;
		n_search = src.size() / 2;
		for(int i = 0; i < members.size(); i++)
		{
			double mag = sqrt(members[i].x * members[i].x + members[i].y * members[i].y);
			if(isMinima)
			{
				if(mag > max) 
				{
					max = mag;
				}
			}
			else
			{
				if(mag < min) 
				{
					min = mag;
				}
			}
		}
		
		magnitude = max - min;
	}
	
	void Local::setMaxMin(geometry_msgs::Point32 &p)
	{
		double mag = sqrt(p.x*p.x + p.y*p.y);
		if(isMinima)
		{
			min = mag;
		}
		else
		{
			max = mag;
		}
	}
	double Local::getMagnitude()
	{
		return magnitude;
	}
}
