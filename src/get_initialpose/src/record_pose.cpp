#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>	
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <math.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include<sys/time.h>
using namespace std;

double x; 
double y;
double th;
double lx; 
double ly;
double lth;
bool flag=true;

int pose_count=0;

void RobotPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    if(pose_count>10)
    {
        x = msg->pose.pose.position.x;
        y = msg->pose.pose.position.y;
        th = tf::getYaw(msg->pose.pose.orientation);
        if (flag)
        {
            lx=x;
            ly=y;
            lth=th;
            flag=false;
        }

        // ROS_INFO("jjjj");
        if(fabs(lx-x)<1&&fabs(ly-y)<1)
        {
            ofstream fos("/home/user/.nav/initPose.txt");
            fos << x << endl;
            fos << y << endl;
            fos << th << endl;
            time_t t;
            time(&t);
            fos<<ctime(&t)<<endl;
            fos.close();

            lx=x;
            ly=y;
            lth=th;
        }
    }
    else
        pose_count++;
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "record_pose");
    ros::NodeHandle nodeHandle;
    ros::Subscriber robotPose= nodeHandle.subscribe("topic_robot_pose",10,RobotPose);  //订阅修正信息
    ros::spin();

    return 0;
}
