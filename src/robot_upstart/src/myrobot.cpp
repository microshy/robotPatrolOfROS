#include <ros/ros.h>

#define WAIT_TIME   20  //s

int main(int argc,char**argv)
{
    ros::init(argc,argv,"start");
    ros::NodeHandle nh;
    
    system("echo  $(date):start >> /home/user/log/start.log");
    system("sh /home/user/newFile.sh");
    ros::Duration(WAIT_TIME).sleep();
    //  system("echo  $(date):launch main process >> /home/user/log/start.log");
    system("roslaunch communication 1.launch");

    return 0;
}
