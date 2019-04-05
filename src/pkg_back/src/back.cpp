#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sensor_msgs/LaserScan.h>
#include <pkg_back/MsgScan.h>
#include <stdio.h>
#include <math.h>

using namespace std;

class Scan2{
public:
    Scan2();
private:
    ros::NodeHandle n;
    ros::Publisher scan_pub;
    ros::Subscriber scan_sub;
    void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan2);
};

Scan2::Scan2()
{
    scan_pub = n.advertise<pkg_back::MsgScan>("topic_scan_back",1);
    scan_sub = n.subscribe<sensor_msgs::LaserScan>("/scan",1, &Scan2::scanCallBack, this);
}

void Scan2::scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan2)
{
    int ranges = scan2->ranges.size();
    pkg_back::MsgScan backscan;

    backscan.dlf=scan2->ranges[10];    //左側 90
    backscan.dlr=scan2->ranges[370];   //右側 90
    backscan.dis=scan2->ranges[190];   //正前 0

    float l0,lr,lf,ch;

    l0=scan2->ranges[190];    //正前 0
    lf=(scan2->ranges[130]+scan2->ranges[140]+scan2->ranges[150]+scan2->ranges[160]+scan2->ranges[170])/5; // 左5
    lr=(scan2->ranges[250]+scan2->ranges[240]+scan2->ranges[230]+scan2->ranges[220]+scan2->ranges[210])/5; // 右5

    float clf,clr;
    clf=(scan2->ranges[180]+scan2->ranges[182]+scan2->ranges[184])/3;  //左3
    clr=(scan2->ranges[200]+scan2->ranges[198]+scan2->ranges[196])/3;  //右3

    ch=(scan2->ranges[157]+scan2->ranges[156]+scan2->ranges[158])/3;
    backscan.ch=ch;
    backscan.lf=lf;
    backscan.lr=lr;
    backscan.clf=clf;
    backscan.clr=clr;

    scan_pub.publish(backscan);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "back");
    Scan2 scan2;
    ros::spin();
}
