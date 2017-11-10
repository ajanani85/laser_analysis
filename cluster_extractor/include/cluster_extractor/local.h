#ifndef LOCAL_H_
#define LOCAL_H_

#include<vector>
#include<geometry_msgs/Point32.h>
#include<sensor_msgs/PointCloud.h>

using namespace std;

namespace laser_analysis
{
	class Local
	{
	private:
		double magnitude;
		double max = 0.0;
		double min = 100.00;
		int n_search;
		vector<geometry_msgs::Point32> members;
		bool isMinima = false;
		
		void setMaxMin(geometry_msgs::Point32 &p);
	public:
		int index;
		int inRangeNeighbours;
		geometry_msgs::Point32 centre;
		
		Local();
		~Local();
		void addMembers(vector<geometry_msgs::Point32> &src, bool is_minima);
		void setLocal(geometry_msgs::Point32 &p , bool  is_minima);
		void setLocal(vector<geometry_msgs::Point32> &src, bool is_minima);
		double getMagnitude();
	};
}
#endif
