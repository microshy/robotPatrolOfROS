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

double ldlf=0,ldlb=0,ldrf=0,ldrb=0;
double ELF=0,ELB=0,ERF=0,ERB=0;

void getOdom(const pkg_msgs::MsgOdometrySensor::ConstPtr& msg)
{
    double dlf = msg->dlf;
    double dlb = msg->dlb;

    double drf = msg->drf;
    double drb = msg->drb;

    ELF=fabs(dlf-ldlf);
    ELB=fabs(dlb-ldlb);
    ERF=fabs(drf-ldrf);
    ERB=fabs(drb-ldrb);

    if(ELF>0.1||ELB>0.1||ERF>0.1||ERB>0.1)
    {
        ROS_INFO("ERROR");
        ROS_INFO("lf%f,lb%f,rf%f,rb%f",dlf,dlb,drf,drb);
        ROS_INFO("llf%f,llb%f,lrf%f,lrb%f",ldlf,ldlb,ldrf,ldrb);
    }
    else
    {
        ldlf=dlf;
        ldlb=dlb;
        ldrf=drf;
        ldrb=drb;
    }

}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"node_odom_printer");
    ros::NodeHandle nodeHandle;

    ros::Subscriber odometrySubscriber;
    odometrySubscriber = nodeHandle.subscribe("topic_odometry_sensor",1000,getOdom);
    ros::spin();
}
