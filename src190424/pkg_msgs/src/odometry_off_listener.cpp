#include "ros/ros.h"
#include "pkg_msgs/MsgServerCmd.h"

void openOdometry(const pkg_msgs::MsgServerCmd::ConstPtr& msg)
{
    if(msg->cmd == "ODOM_OFF")
    {
        ROS_INFO("node_odometry_off_listener: get ODOM_OFF command!");
        system("roslaunch pkg_msgs odom_on.launch");
    }
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"node_odometry_listener");
    ros::NodeHandle nodeHandle;
    ros::Subscriber subscriber = nodeHandle.subscribe("topic_server_cmd",1000,openOdometry);
    ros::spin();
}
