#include <string>
#include <ros/ros.h>
#include <pkg_msgs/MsgOdometrySensor.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <math.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <processor/isTurn.h>

using namespace std;
//debug

// /debug
// position
double x; 
double y;
double th;

char initPoseCfg[] = "/home/user/.nav/initPoseCfg.txt";

// velocity
double vx;
double vy;
double vth;

//spin
bool flag;

//
double LX;
double LY;
double EX;
double EY;

ros::Time lastOdomTime;
ros::Time curOdomTime;

const double axisDis = 0.55;
const double wheelDis = 0.55;
const double opAngDis = sqrt(axisDis*axisDis + wheelDis*wheelDis);
const double spinFactor = wheelDis/opAngDis/opAngDis;

void reviseOdom(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    th = tf::getYaw(msg->pose.pose.orientation);
}

void spinAround(const processor::isTurn::ConstPtr& msg)
{
    flag=msg->turnFlag;
}

void updateData(const pkg_msgs::MsgOdometrySensor::ConstPtr& msg)
{
    //读取、判断、更新最新数据
    if(msg->type == "ODOMETER")
    {
        curOdomTime = ros::Time::now();
        double dt = (curOdomTime - lastOdomTime).toSec();

        double vlf = msg->vlf;
        double vlb = msg->vlb;
        double vrf = msg->vrf;
        double vrb = msg->vrb;
        double vl = (vlf+vlb)/2;
        double vr = (vrf+vrb)/2;
        double v = (vl+vr)/2;
        vx = cos(th) * v;
        vy = sin(th) * v;
        vth = (vr-vl)*spinFactor;

        //置0
        vx=0;
        vy=0;
        vth=0;

        LX=x;
        LY=y;

        //ROS_INFO("LX%f,   LY%f,  LTH%f",x,y,th);

        double dlf = msg->dlf;
        double dlb = msg->dlb;
        //double dlb = msg->dlf;
        double drf = msg->drf;
        double drb = msg->drb;
        //double drb = msg->drf;
        double dl = (dlf+dlb)/2;
        double dr = (drf+drb)/2;
        double d = (dl+dr)/2;

        x += cos(th) * d;
        y += sin(th) * d;

        if(flag)
        {//jiaodu
            double dlth=dl/opAngDis;
            double drth=dr/opAngDis;
            double dth=fabs(dlth)+fabs(drth);
            if(dr<0)
            {
                dth=-dth;
            }
            th+=dth;
        }
        else
        {
            th += (dr-dl)*spinFactor;
        }

        EX=x-LX;
        EY=y-LY;

        if(EX>0.1||EY>0.1)
        {
            ROS_INFO("ERROR");
            ROS_INFO("LX%f,   LY%f",LX,LY);
            ROS_INFO("LX%f,   LY%f",x,y);
        }

        //ROS_INFO("X%f,  Y%f,  TH%f",x,y,th);

        lastOdomTime = ros::Time::now();

    }
    else if(msg->type == "COMPASS")
    {
        //TO DO
    }
    else if(msg->type == "IMU")
    {
        //TO DO
    }
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "node_odom_robot");
    ros::NodeHandle nodeHandle;
    ros::Subscriber odometryReviseSubscriber = nodeHandle.subscribe("topic_revise_odometry",1000,reviseOdom);	//订阅修正信息
    ros::Subscriber odometrySensorSubscriber = nodeHandle.subscribe("topic_odometry_sensor",1000,updateData);	//订阅量测传感器信息
    ros::Subscriber justAroundSubscriber = nodeHandle.subscribe("processor/turn",1000,spinAround);      //订阅转弯信息
    ros::Publisher odom_pub = nodeHandle.advertise<nav_msgs::Odometry>("odommmm", 10);

    // initial position
    ifstream fis(initPoseCfg);
    fis >> x;
    fis >> y;
    fis >> th;
    fis.close();

    // velocity
    vx = 0.0;
    vy = 0.0;
    vth = 0.0;

    curOdomTime = ros::Time::now();
    lastOdomTime = ros::Time::now();
    //	ros::Time curTime = ros::Time::now();
    //	ros::Time lastTime = ros::Time::now();

    //tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(50);

    const double arc2deg = 180/M_PI;

    // message declarations
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    while (ros::ok()) {
        //周期性航迹推算
        //	curTime = ros::Time::now(); 	//周期开始，更新当前时间
        ros::spinOnce();		//检查跟新外部数据
        //角度范围-pi - pi，弧度制
        while(th < -M_PI)
        {
            th += 2*M_PI;
        }
        while(th > M_PI)
        {
            th -= 2*M_PI;
        }

        //	lastTime = curTime;		//航迹推算处理结束，当前时间作为上一周期时间。
        //虽然一次循环没有结束，但位姿的新周期已经开始了。
        //std::cout << "x: " << x << ", y: " << y <<", th " << th * arc2deg << std::endl;
        //std::cout << "vx: " << vx << ", vy: " << vy << std::endl;

        //更新base_link->odom坐标变换
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(th);

        //更新odom消息
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        //位姿
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(th);
        //速度
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.linear.z = 0.0;
        odom.twist.twist.angular.x = 0.0;
        odom.twist.twist.angular.y = 0.0;
        odom.twist.twist.angular.z = vth;

        //发布odom消息和baselink->odom坐标变换
        //broadcaster.sendTransform(odom_trans);
        odom_pub.publish(odom);
        loop_rate.sleep();
    }
    return 0;
}
