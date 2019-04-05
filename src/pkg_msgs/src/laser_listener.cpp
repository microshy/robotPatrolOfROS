#include "ros/ros.h"
#include "pkg_msgs/MsgServerCmd.h"

void openLaser(const pkg_msgs::MsgServerCmd::ConstPtr& msg)
{
    if(msg->cmd == "LASER_ON")
    {
        ROS_INFO("node_laser_listener:get LASER_ON command");
        system("roslaunch pkg_msgs laser_on.launch");
        //printf("get the open command\n");
    }
    if(msg->cmd == "OPEN_LASER")
    {
        ROS_INFO("node_laser_listener:get OPEN_LASER command");
        system("roslaunch pkg_msgs open_laser.launch");
    }
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"node_laser_listener");
    ros::NodeHandle nodeHandle;
    ros::Subscriber subscriber = nodeHandle.subscribe("topic_server_cmd",1,openLaser);
    ros::spin();
}
