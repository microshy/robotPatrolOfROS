#include "ros/ros.h"
#include "pkg_msgs/MsgPointInfo.h"
#include "pkg_srvs/SrvReturn.h"
#include <iostream>
#include <string>
#include <fstream>
#include "geometry_msgs/PoseWithCovarianceStamped.h"

using namespace std;

unsigned char locationX[4]={0x00,0x00,0x00,0x00};
unsigned char locationY[4]={0,0,0,0};
int pose_x;
int pose_y;

void HexDump(char *buf,int len,int addr)
{
    int i,j,k;
    char binstr[80];

    for (i=0;i<len;i++)
    {
        if (0==(i%16))
        {
            sprintf(binstr,"%08x -",i+addr);
            sprintf(binstr,"%s %02x",binstr,(unsigned char)buf[i]);
        }
        else if (15==(i%16))
        {
            sprintf(binstr,"%s %02x",binstr,(unsigned char)buf[i]);
            sprintf(binstr,"%s  ",binstr);
            for (j=i-15;j<=i;j++)
            {
                //       sprintf(binstr,"%s%c",binstr,('!'<buf[j]&&buf[j]<='~')?buf[j]:'.');
            }
            printf("%s\n",binstr);
        }
        else
        {
            sprintf(binstr,"%s %02x",binstr,(unsigned char)buf[i]);
        }
    }
    if (0!=(i%16))
    {
        k=16-(i%16);
        for (j=0;j<k;j++)
        {
            sprintf(binstr,"%s   ",binstr);
        }
        sprintf(binstr,"%s  ",binstr);
        k=16-k;
        for (j=i-k;j<i;j++)
        {
            //     sprintf(binstr,"%s%c",binstr,('!'<buf[j]&&buf[j]<='~')?buf[j]:'.');
        }
        printf("%s\n",binstr);
    }
}

void statecallback1(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg1)  //获取机器人坐标以及姿态角度；
{    
    //   ROS_INFO("receive msg from nav\n");
    double X,Y,th;

    X = msg1->pose.pose.position.x;
    Y = msg1->pose.pose.position.y;

    pose_x=X*100;
    pose_y=Y*100;
}

void showRoute(string &a)
{   
    char data[1000]={0};
    memcpy(data,a.c_str(),a.size()+1);

    ofstream fos("/home/user/share/myroute.txt");
    int pn=0;
    pn=a.size()/8;
    int i = 0;
    do
    {
        locationX[0]=data[8*i+0];//读出对应点的x坐标
        locationX[1]=data[8*i+1];
        locationX[2]=data[8*i+2];
        locationX[3]=data[8*i+3];
        locationY[0]=data[8*i+4];//读出对应点的y坐标
        locationY[1]=data[8*i+5];
        locationY[2]=data[8*i+6];
        locationY[3]=data[8*i+7];

        //将目标点坐标转换成double型 单位m
        int ddpointX = 0;
        int ddpointY = 0;//目标点

        ddpointX =(((((((ddpointX | locationX[0]) << 8) | locationX[1]) << 8 ) |locationX[2]) << 8) | locationX[3]);
        ddpointY =(((((((ddpointY | locationY[0]) << 8) | locationY[1]) << 8 ) |locationY[2]) << 8) | locationY[3]);

        cout<<i+1<<": ("<<ddpointX<<","<<ddpointY<<")"<<endl;

        fos << ddpointX <<" "<< ddpointY<<endl;
        i++ ;
    }while(i<pn);

    fos.close();
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"return_route");
    ros::NodeHandle nodeHandle;
    ros::ServiceClient clientback = nodeHandle.serviceClient<pkg_srvs::SrvReturn>("srv_return"); //调用自主返航服务

    ros::Subscriber sub1 = nodeHandle.subscribe("topic_robot_pose",1, &statecallback1);
    pkg_srvs::SrvReturn backhome;
    backhome.request.clear = false;

    if(clientback.call(backhome))
    {
        string route=backhome.response.location;

        printf("back home success\n");
        char *sendBuf=(char *)route.c_str();
        int out_len=route.size();

        //调试用
        cout<<"route :";
        HexDump(sendBuf,out_len,0);
        showRoute(route);
    }
    //clear
    if(argc>1 && strcmp(argv[1],"clear")==0)
    {
        pkg_srvs::SrvReturn backhome;
        backhome.request.clear = true;
        if(clientback.call(backhome))
        {cout<<"clear back route"<<endl;}
    }
    return 0;
}
