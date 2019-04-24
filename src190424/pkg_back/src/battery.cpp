#include "ros/ros.h"
#include "communication/state.h"
#include "pkg_back/state.h"
#include <string>
using namespace std;

#define VOL_TARGET 28

bool fullFlag=false;
bool positive=true;
double last_curr=99999;

void callback(const communication::state &msg)
{
    string data=msg.data;
    double vol=data[28]*256+(unsigned char)data[29];
    vol/=100.0;
    double curr=(char)data[30]*256+(unsigned char)data[31];
    curr/=100.0;

    if(vol>VOL_TARGET && curr>-0.25)
        fullFlag=true;
    else
        fullFlag=false;

    if( last_curr == 99999)
        last_curr=curr;

    if(curr>0 && last_curr>0)
        positive=true;
    else
        positive=false;


    //cout<<"vol "<<vol<<" current "<<curr<<" fullFlag:"<<fullFlag<<" currpositive: "<<positive <<" last_curr "<<last_curr<<endl;
    //cout<<"vol "<<vol<<" current "<<curr<<endl;
    ROS_INFO("Battery_node:Charge Voltage: %.1f, Charge Cur:%.1f, fullFlag: %d, Last Cur: %.1f",vol,curr,fullFlag,last_curr);
    last_curr=curr;

}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"node_battery");
    ros::NodeHandle nodeHandle;
    ros::Subscriber sub = nodeHandle.subscribe("communication/state_battery",10,callback);
    ros::Publisher  pub = nodeHandle.advertise<pkg_back::state>("pkg_back/chargeFull", 1000);

    ros::Rate loop_rate(1);
    while(ros::ok())
    {
        pkg_back::state msg;
        msg.fullFlag=fullFlag;
        msg.positive_curr=positive;
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
