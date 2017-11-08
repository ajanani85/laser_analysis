#ifndef CLUSTER_H_
#define CLUSTER_H_

#include <geometry_msgs/Point32.h>
#include <vector>
#include <utility>

using namespace std;

#define RED "\x1b[31m"
#define GREEN "\x1b[32m"
#define YELLOW "\x1b[33m"
#define BLUE "\x1b[34m"
#define MAGENTA "\x1b[35m"
#define CYAN "\x1b[36m"
#define COLOR_RESET "\x1b[0m"

namespace laser_analysis
{
	class Cluster
	{
	private:
		vector<geometry_msgs::Point32> members;
				
		double slope;
		double y_intercept;
		
		double x_bar;
		double y_bar;
		double sum_x;
		double sum_y;
		double sum_x2;
		double sum_xy;
		
	public:
		int leftBound;
		int rightBound;
		double distanceToLeftCluster;
		double distanceToRightCluster;
		
		Cluster();
		~Cluster();
		
		void addMember(geometry_msgs::Point32 p);
		vector<geometry_msgs::Point32> &getMembers();
		int memberSize();
		pair<double, double> getTrend();
	};
}

#endif
