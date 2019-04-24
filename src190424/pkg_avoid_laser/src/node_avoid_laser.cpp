#include "ros/ros.h"
#include "std_msgs/String.h"
//#include "sensor_msgs/LaserScan.h"
#include "pkg_msgs/LaserScanNew.h"

const int cnt = 121;//381
float prev_dis[cnt];
int pit_tbl[cnt];
int obs_tbl[cnt];
int is_obstacle = 0;
int is_first_time = 1;
//void printCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
void printCallback(const pkg_msgs::LaserScanNew::ConstPtr& msg)
{
    ROS_INFO("RECEIVE!  \n");
    /*for (int i = 0; i < cnt; i++)
        std::cout<<msg->ranges[i]<<" ";
    std::cout<<std::endl;*/
    if (is_first_time)
    {
        for (int i = 0; i < cnt; i++)
	    prev_dis[i] = msg->ranges[i];
	is_first_time = 0;
    }
    else
    {
	for (int i = 0; i < cnt; i++)
	{
	    if (msg->ranges[i] >= prev_dis[i] + 0.05)
	    {
		ROS_INFO("Pit Front!   \n");
		pit_tbl[i] = 1;
		obs_tbl[i] = 0;
	    }
	    else if (msg->ranges[i] <= prev_dis[i] - 0.05)
	    {
		ROS_INFO("Obstacle Front!    \n");
		pit_tbl[i] = 0;
		obs_tbl[i] = 1;
	    }
	    else
	    {
		ROS_INFO("As Usual!   \n");
		pit_tbl[i] = 0;
		obs_tbl[i] = 0;
	    }
	    prev_dis[i] = msg->ranges[i];
	}
    }
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "avoid_laser");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("scannew", 1000, printCallback);
    ros::spin();
    return 0;
}
