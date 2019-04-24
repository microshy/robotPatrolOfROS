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

using namespace std;
char initPose[] = "/home/robot/.nav/initPose.txt";
geometry_msgs::PoseWithCovarianceStamped initialPose;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "amcl_initialpose");
    ros::NodeHandle n;
    ros::Publisher initialposePublisher = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);

    double x,y,th;
    ifstream fis(initPose);
    fis >> x;
    fis >> y;
    fis >> th;
    fis.close();
    //x=0.0;
    //y=0.0;
    //th=0.0;
    int count=0;
    while (ros::ok()&&count<2)
    {
        initialPose.pose.pose.position.x=x;
        initialPose.pose.pose.position.y=y;
        initialPose.pose.pose.orientation=tf::createQuaternionMsgFromYaw(th);

        initialposePublisher.publish(initialPose);
        ROS_INFO("HAHA");

        count++;
        ros::spinOnce();
        ros::Duration(1).sleep();
    }
    ROS_INFO("HAHAHAHAHA");

    return 0;
}
