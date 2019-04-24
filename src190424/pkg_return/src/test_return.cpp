#include "ros/ros.h"
#include <fstream>
#include "string.h"
using namespace std;
/*
version 2.0
author ydd
modify: save the main route 
keep the subRoad point when not find the mainRoad ever
find the mainRoad point in main route once got first mainRoad point
*/

//const char* file="/robot/src/pkg_return/src/udp_node.txt";
const char* file="/home/robot/catkin_ws/src/pkg_return/src/udp_node.txt";
//const char* absolutefile="/home/livic/robot/src/pkg_return/src/absolute_test_return.txt";
const char* absolutefile="/home/robot/log/record/absolute_test_return.txt";

int getFileSize(char * strFileName)
{
    FILE * fp = fopen(strFileName, "r");
    fseek(fp, 0L, SEEK_END);
    int size = ftell(fp);
    fclose(fp);
    return size;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"test_return");
    ros::NodeHandle nodeHandle;
    //ros::Subscriber sub = nodeHandle.subscribe("topic_point_info",10,record);
    //ros::ServiceServer returnService = nodeHandle.advertiseService("srv_return",returnRoute);
    int filesize;
    char *relatefile;
    relatefile = getenv("HOME");
    strcat(relatefile,file);
    filesize = getFileSize(file);
    cout<<filesize<<endl;

    /*
    ofstream fos(relatefile);
    ofstream absolutefos(absolutefile);

    if (fos.is_open())
    {
        fos << relatefile <<endl;
    }
    else
    {
        ROS_INFO("FILE OPEN FAIL!");
    }

    if (absolutefos.is_open())
    {
        absolutefos << relatefile <<endl;
    }
    else
    {
        ROS_INFO("ABSOLUTE FILE OPEN FAIL!");
    }

    fos.close();
    absolutefos.close();
    */
    return 0;
}
