//与充电屋通信：开门指令，查询指令（开门是否到位，接触是否到位），充电器接触指令
/* (1) 提供服务：下发指令给充电屋
   (2) 发布消息：开门状态，接触状态
*/
#include <string>
#include <ros/ros.h>
#include <fcntl.h>
#include <communication/sendCmd.h>

#include <communication/state.h>
#include "processor/parkingOrder.h"
#include "communication/SerialPort.h"
#include <communication/record.h>

using namespace std;

#define  PORTNAME       "/dev/ttyS2"
#define  BAUD           115200
#define  TIME_OUT       1  //s
#define  WAIT_TIME      0.5//frequence of ask commmand
#define  ASK_LENTH      34
#define  COMMON_LENTH   9

char ASK_TYPE = 0X20;
char ASK_DATA = 0X01;
char ROOM_ID  = 0x10; 

char CONNECT_TYPE = 0x00;
char CONNECT_DATA = 0X01;

bool startFlag=false;
SerialPort basePort = *(new SerialPort(PORTNAME));

bool sendCallback(communication::sendCmd::Request  &req,
                  communication::sendCmd::Response &res) ;

void callback(const processor::parkingOrder &msg)
{
    startFlag = msg.zigbeeComFlag; // 如果为true则建立链接
}  

int main(int argc,char ** argv)
{
    ros::init(argc,argv,"zigbee_node");
    ros::NodeHandle nh;
    ros::ServiceServer send_Cmd = nh.advertiseService("send_room_cmd", sendCallback); //发送给充电屋
    ros::Publisher pub_state=nh.advertise<communication::state>("communication/state_room",10); //发布充电屋状态
    ros::Subscriber sub=nh.subscribe("parkingOrder",10,&callback); //订阅入库指令

    //配置串口
    basePort.setID(ROOM_ID);
    basePort.openPort();
    basePort.setPort(BAUD,8,1,'n');
    basePort.setInMode('r');
    basePort.setOutMode('r');

    //状态查询
    communication::state msg;
    int conn_time=0;
    ROS_INFO("Zigbee_node:start communicate with room!");

    while(ros::ok())
    {
        // if(startFlag) //true 则建立连接
        if(true)
        {
            //cout<<"send"<<endl;
            if(!basePort.connect) // disconnected
            {
                //printf("connect command\n");
                string t;
                string d;
                t.push_back(CONNECT_TYPE);
                d.push_back(CONNECT_DATA);
                basePort.sendInfo(t,d);
                ROS_INFO("Zigbee_node:send connect command to ROOM!");
                char buff[20];
                int readBytes;
                readBytes = basePort.waitBack(buff,9 , TIME_OUT);
                conn_time++;

                if(conn_time>10)
                {
                    msg.wrong_flag=true;  //通信故障
                    msg.break_flag=true;
                    msg.data="";
                    pub_state.publish(msg);
                    //printf("communication failed\n");
                    ROS_INFO("Zigbee_node:communication with ROOM failed!");
                    //system("echo  $(date): zigbee communication failed>> ~/log/zigbee.log");
                }
                else if(!basePort.connect&&conn_time>2)
                {
                    msg.wrong_flag=true;   //通信超时
                    msg.break_flag=false;
                    msg.data="";
                    pub_state.publish(msg);
                    //printf("communication wrong \n");
                    ROS_INFO("Zigbee_node:communication with ROOM failed!");
                }
            }
            else
            {
                conn_time=0;
                //printf("ask data\n");
                ROS_INFO("Zigbee_node:ASK ROOM STATE!");
                string t;
                string d;
                t.push_back(ASK_TYPE);
                d.push_back(ASK_DATA);

                basePort.sendInfo(t,d);
                ros::Duration(0.5).sleep();
                char buff[46];
                int readBytes;
                readBytes = basePort.waitBack(buff,46, TIME_OUT);
                //发布消息
                if(basePort.receFlag) //correct data,pub out
                {
                    msg.wrong_flag=false;
                    msg.break_flag=false;
                    msg.type=ASK_TYPE;
                    msg.data=basePort.data_in;
                    msg.lenth=basePort.len+1;//data lenth and type lenth
                    pub_state.publish(msg);
                }
            }
        }
        ros::Duration(WAIT_TIME).sleep();
        ros::spinOnce();
    }
    basePort.closePort();
    return 0;
}

//提供服务
bool sendCallback(communication::sendCmd::Request  &req,
                  communication::sendCmd::Response &res)
{
    char buff[20];
    basePort.sendInfo(req.type,req.data);

    int readBytes=basePort.waitBack(buff,9, TIME_OUT);
    if(readBytes)
    {
        res.receive=basePort.receFlag;
        res.state=basePort.data_in;//接受正常与否
        return true;
    }
    else
    {
        return false;
    }
    ros::Duration(0.5).sleep();
}
