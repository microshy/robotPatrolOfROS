#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <cmath>
#include <tf/transform_broadcaster.h>
#include <string>
#include <pkg_msgs/MsgOdometrySensor.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <math.h>

using namespace std;

double rightOdom=0,leftOdom=0,lf=0,lb=0,rf=0,rb=0;

void getOdom(const pkg_msgs::MsgOdometrySensor::ConstPtr& msg)
{
    double dlf = msg->dlf;
    double dlb = msg->dlb;

    double drf = msg->drf;
    double drb = msg->drb;

    double dl = (dlf+dlb)/2;
    double dr = (drf+drb)/2;
    double d = (dl+dr)/2;
    lf+=dlf;
    lb+=dlb;
    rf+=drf;
    rb+=drb;

    //debug
    leftOdom+=dl;
    rightOdom+=dr;
    ROS_INFO("get odom data:lf%f,lb:%f,rf%f,rb%f",lf,lb,rf,rb);
    ROS_INFO("get odom data:rightOdom%f,leftOdom%f",rightOdom,leftOdom);
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"node_odom_odom");
    ros::NodeHandle nodeHandle;

    ros::Subscriber odometrySubscriber;
    odometrySubscriber = nodeHandle.subscribe("topic_odometry_sensor",1000,getOdom);
    ros::spin();
}
