#include "ros/ros.h"
#include "pkg_msgs/MsgPointInfo.h"
#include "pkg_srvs/SrvReturn.h"
#include <stack>
#include <string>
#include <vector>
#include<fstream>
using namespace std;
/*
version 2.0
author ydd
modify: save the main route 
keep the subRoad point when not find the mainRoad ever
find the mainRoad point in main route once got first mainRoad point
*/
struct PointInfo
{
    string location;
    string speed;
    int pointlevel;
};

struct Point
{
    int x;
    int y;
};

stack<PointInfo> routeBak;    //original data, clear when recieve clear
vector<Point> readPoint;
const char* file="/home/robot/.nav/route.txt";

vector<Point> createRoute(ifstream &is)
{
    vector<Point> tempPoint;
    int id,x,  y;
    while (is >> id >> x >> y)
    {
        Point temp;
        temp.x = x;
        temp.y = y;
        tempPoint.push_back(temp);
        //cout<<x<<" "<<y<<endl;
        ROS_INFO("Node_return:The main road Point: %d：%d, %d",id,x,y);
    }
    return tempPoint;
}

stack<PointInfo> searchRoute(Point &a, const vector<Point> &myRoute)
{
    stack<PointInfo> result;
    int route_size=myRoute.size();
    int i=0;
    for (; i < route_size; i++)
    {
        int _x = myRoute[i].x, _y = myRoute[i].y;

        //record current mainRoad point
        PointInfo mypoint;
        //transform to hex form
        mypoint.location.push_back(_x>>24);
        mypoint.location.push_back(_x>>16);
        mypoint.location.push_back(_x>>8);
        mypoint.location.push_back(_x);

        mypoint.location.push_back(_y >> 24);
        mypoint.location.push_back(_y >> 16);
        mypoint.location.push_back(_y >> 8);
        mypoint.location.push_back(_y);
        result.push(mypoint);

        // if find target point , stop
        if ((_x==a.x)  &&  (_y==a.y))
        {
            //cout<<"find "<<a.x<<" "<<a.y<<endl;
            //cout<<"main route"<<_x<<" "<<_y<<endl;
            //cout<<"same"<< endl;
            ROS_INFO("Node_return:Point(%d,%d) in the Main Road.",a.x,a.y);
            break;
        }
    }

    //never find
    if(i==route_size )
    {
        //cout<<"no find route"<<endl;
        ROS_INFO("Node_return:No road was found!");
        stack<PointInfo> nothing;
        return nothing;
    }
    else
        return result;
}

void record(const pkg_msgs::MsgPointInfo::ConstPtr& msg)
{
    PointInfo temp;
    temp.location = msg->location;
    temp.speed = msg->speed;
    temp.pointlevel = msg->pointlevel;
    routeBak.push(temp);
    //cout<<"get new point:level "<<msg->pointlevel<<endl;
    ROS_INFO("Node_return:Get new point level: %d",msg->pointlevel);
}

bool returnRoute(pkg_srvs::SrvReturn::Request  &req,
                 pkg_srvs::SrvReturn::Response &res)
{
    stack<PointInfo> mainStack;

    bool clear = req.clear;
    if(clear)
    {
        while(!routeBak.empty())
        {
            routeBak.pop();
        }
        return true;
    }
    stack<PointInfo> routeStack=routeBak;  //temp data

    if(routeStack.size()<=1)
    {
        return false;
    }

    if(routeStack.top().pointlevel!=1)
    {
        routeStack.pop();  //dismiss the top point ,cause maybe not arrived
    }

    string localSeq;
    string speedSeq;

    while(!routeStack.empty())
    {
        if(routeStack.top().pointlevel==1)
        {
            //cout<<"get main point"<<endl;
            ROS_INFO("Node_return:Get main point!");
            string temp=routeStack.top().location;

            int _x = ((unsigned char)temp[0] << 24) + ((unsigned char)temp[1] << 16) + ((unsigned char)temp[2] << 8) + (unsigned char)temp[3];
            int _y = ((unsigned char)temp[4] << 24) + ((unsigned char)temp[5] << 16) + ((unsigned char)temp[6] << 8) + (unsigned char)temp[7];
            //cout<<"need find "<<_x<<" "<<_y<<endl;
            ROS_INFO("Node_return:Need to find Point (%d,%d) in Main Road",_x,_y);

            Point a;
            a.x=_x;
            a.y=_y;
            //cout<<a.x<<" "<<a.y<<endl;
            mainStack=searchRoute(a,readPoint);

            if(mainStack.empty())
                return false;

            break;
        }
        else
        {
            localSeq += routeStack.top().location;
            speedSeq += routeStack.top().speed;
            routeStack.pop();
            //cout<<"支路 ，add "<<endl;
            ROS_INFO("Node_route:Add subroad!");
        }
    }

    //add point to road
    while (!mainStack.empty())
    {
        localSeq += mainStack.top().location;
        mainStack.pop();
    }

    //add original point
    char tmp=0x00;
    for(int i=0;i<8;i++)
    {
        localSeq += tmp;
    }
    speedSeq += "0";
    res.location = localSeq;
    res.speed = speedSeq;
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"node_return");
    ros::NodeHandle nodeHandle;
    ros::Subscriber sub = nodeHandle.subscribe("topic_point_info",10,record);
    ros::ServiceServer returnService = nodeHandle.advertiseService("srv_return",returnRoute);

    ifstream readfile;
    readfile.open(file);

    if (readfile.is_open())
    {
        readPoint=createRoute(readfile);
        ROS_INFO("Node_return: Read main route from /home/user/.nav/route.txt");
    }

    readfile.close();
    ros::spin();
    return 0;
}
