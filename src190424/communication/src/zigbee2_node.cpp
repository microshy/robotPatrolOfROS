//与充电屋通信：开门指令，查询指令（开门是否到位，接触是否到位），充电器接触指令
/* (1) 提供服务：下发指令给充电屋
   (2) 发布消息：开门状态，接触状态
*/
#include <string>
#include <ros/ros.h>
#include <fcntl.h>
#include <communication/sendCmd.h>
#include <communication/state.h>
#include <communication/zigbee.h>
#include "processor/parkingOrder.h"

using namespace std;
using namespace boost::asio;           //定义一个命名空间，用于后面的读写操作'''

#define PORTNAME    "/dev/ttyS2"
#define BAUD        115200
#define WAIT_TIME   0.5

char ASK_TYPE = 0X00;
char ASK_DATA = 0X01;
char ROOM_ID  = 0x03; 

Zigbee my_zige(PORTNAME,ROOM_ID);
bool startFlag=false;

bool sendCallback(communication::sendCmd::Request  &req,
                  communication::sendCmd::Response &res) ;

void callback(const processor::parkingOrder &msg)
{
    startFlag = msg.zigbeeComFlag;
}  

int main(int argc,char ** argv)
{
    ros::init(argc,argv,"command_to_room");
    ros::NodeHandle nh;
    ros::ServiceServer send_Cmd = nh.advertiseService("send_room_cmd", sendCallback);
    ros::Publisher pub_state=nh.advertise<communication::state>("communication/state_room",10);
    ros::Subscriber sub=nh.subscribe("parkingOrder",10,&callback);

    communication::state msg;
    my_zige.setSeial(115200,1,'N',8);

    string t;
    string d;
    t=ASK_TYPE;
    d=ASK_DATA;

    return 0;
}

//提供服务
bool sendCallback(communication::sendCmd::Request  &req,
                  communication::sendCmd::Response &res)
{
    char buff[20];
    my_zige.sendInfo(req.type,req.data);
    my_zige.waitBack(buff,9);

    cout<<endl;

    res.receive=my_zige.receFlag;
    res.state=my_zige.data_in;//接受正常与否

    return true;
}

