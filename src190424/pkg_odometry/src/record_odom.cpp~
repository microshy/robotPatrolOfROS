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
#include <pkg_odometry/record.h>	

using namespace std;

ofstream OsWrite;
ofstream OsWrite1;
ofstream OsWrite2;
//debug

// /debug
// position
double x; 
double y;
double th;

char initPoseCfg[] = "/home/user/.nav/initPoseCfg.txt";
const char* file   = "/home/user/paper/turn_odom.csv";
const char* file1  = "/home/user/paper/straight_odom.csv";
const char* file2  = "/home/user/paper/all_odom.csv";
// velocity
double vx;
double vy;
double vth;

//spin
bool flag;
bool first=false;

ros::Time lastOdomTime;
ros::Time curOdomTime;

const double axisDis = 0.55;
const double wheelDis = 0.55;
const double opAngDis = sqrt(axisDis*axisDis + wheelDis*wheelDis);
const double spinFactor = wheelDis/opAngDis/opAngDis;
const double leftfFix = 0.9784;
const double leftbFix = 0.9805;
const double rightfFix = 0.9889;
const double rightbFix = 0.9775;

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
        
		
        double dlf = msg->dlf;
        double dlb = msg->dlb;
        //double dlb = msg->dlf;
        double drf = msg->drf;
        double drb = msg->drb;
        //double drb = msg->drf;
		
		if(first&&!flag)
		{
			first=false;
			OsWrite<<endl;
		}
		if(flag)
		{
			first=true;
		}
		
		if(flag)
		{
			OsWrite<<dlf<<","<<dlb<<","<<drf<<","<<drb<<"," ;
        }
		else
		{
			OsWrite1<<dlf<<","<<dlb<<","<<drf<<","<<drb<<"," ;
		}
		
			OsWrite2<<dlf<<","<<dlb<<","<<drf<<","<<drb<<"," ;
			
		
		if(fabs(dlf)>0.1||fabs(dlb)>0.1||fabs(drf)>0.1||fabs(drb)>0.1)
        {
            if(dlf>0) dlf=0.05;	 else dlf=-0.05;
            if(dlb>0) dlf=0.05;	 else dlf=-0.05;
            if(drf>0) dlf=0.05;	 else dlf=-0.05;
            if(drb>0) dlf=0.05;	 else dlf=-0.05;
        }

        dlf =dlf*leftfFix;
        dlb =dlb*leftbFix;
        drf =drf*rightfFix;
        drb =drb*rightbFix;
		
		if(flag)
		{
			OsWrite<<dlf<<","<<dlb<<","<<drf<<","<<drb<<"," ;
        }
		else
		{
			OsWrite1<<dlf<<","<<dlb<<","<<drf<<","<<drb<<"," ;
		}
		OsWrite2<<dlf<<","<<dlb<<","<<drf<<","<<drb<<"," ;
        
		double dl = (dlf+dlb)/2;
        double dr = (drf+drb)/2;
        double d = (dl+dr)/2;

		if(flag)
		{
			
			OsWrite<<dl<<","<<dr<<","<<d<<"," ;
        }
		else
		{
			OsWrite1<<dl<<","<<dr<<","<<d<<"," ;
		}
		OsWrite2<<dl<<","<<dr<<","<<d<<"," ;
		
        x += cos(th) * d;
        y += sin(th) * d;

		if(!flag)
		{
			OsWrite1<<x<<","<<y<<",";
		}
		OsWrite2<<x<<","<<y<<",";
        if(flag)
        {
           
            //jiaodu
            double dlth=dl/opAngDis;
            double drth=dr/opAngDis;
            double dth=fabs(dlth)+fabs(drth);
            if(dr<0)
            {
                dth=-dth;
                vth= 0;
            }
            th+=dth;
			OsWrite<<dlth<<","<<drth<<","<<dth<<","<<th<<"," ;
			record_time(OsWrite);
        }
        
        else
        {
            th += (dr-dl)*spinFactor;
            vth = (vr-vl)*spinFactor;
			OsWrite1<<th<<"," ;
			record_time(OsWrite1);
        }
		OsWrite2<<th<<",";
		record_time(OsWrite2);
        lastOdomTime = ros::Time::now();
    }
   
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle nodeHandle;
    ros::Subscriber odometryReviseSubscriber = nodeHandle.subscribe("topic_revise_odometry",1000,reviseOdom);	//订阅修正信息
    ros::Subscriber odometrySensorSubscriber = nodeHandle.subscribe("topic_odometry_sensor",1000,updateData);	//订阅量测传感器信息
    ros::Subscriber justAroundSubscriber = nodeHandle.subscribe("processor/turn",1000,spinAround);      //订阅转弯信息
  
    //文本记录
    openFile(file,OsWrite);
    record_string("dlf dlb drf drb dlf dlb drf drb dl dr d dlth drth dth th",OsWrite);

    //文本记录
    openFile(file1,OsWrite1);
    record_string("dlf dlb drf drb dlf dlb drf drb dl dr x y th ",OsWrite1);
	//文本记录
    openFile(file2,OsWrite2);
    record_string("dlf dlb drf drb dlf dlb drf drb dl dr x y th ",OsWrite2);

  

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
  
    ros::Rate loop_rate(50);
    const double arc2deg = 180/M_PI;

    while (ros::ok()) {
        //周期性航迹推算
        //	curTime = ros::Time::now(); 	//周期开始，更新当前时间
        ros::spinOnce();		        //检查更新外部数据
        //角度范围-pi - pi，弧度制
        while(th < -M_PI)
        {
            th += 2*M_PI;
        }
        while(th > M_PI)
        {
            th -= 2*M_PI;
        }


        loop_rate.sleep();
    }
    closeFile(OsWrite); //关闭文本
    closeFile(OsWrite1);//关闭文本
	closeFile(OsWrite2);//关闭文本
    return 0;
}
