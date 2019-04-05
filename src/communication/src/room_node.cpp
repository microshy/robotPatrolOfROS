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
#include <communication/udp_client.h>

using namespace std;

#define	UDP_TEST_PORT    1031
#define UDP_SERVER_IP    "192.168.0.250"
#define MAXSIZE          1000
#define TIME_WRONG       300*1000//300ms

char ASK_TYPE = 0X20;
char ASK_DATA = 0X01;
char ROOM_ID  = 0x10; 
char CONNECT_TYPE = 0x00;
char CONNECT_DATA = 0X01;
bool startFlag=false;

bool sendCallback(communication::sendCmd::Request  &req,
                  communication::sendCmd::Response &res) ;

void callback(const processor::parkingOrder &msg)
{
    startFlag = msg.zigbeeComFlag; // 如果为true则建立链接
}  

UDP_Client client;

int main(int argc,char ** argv)
{
    ros::init(argc,argv,"zigbee_node");
    ros::NodeHandle nh;
    ros::ServiceServer send_Cmd = nh.advertiseService("send_room_cmd", sendCallback); //发送给充电屋
    ros::Publisher pub_state=nh.advertise<communication::state>("communication/state_room",10); //发布充电屋状态
    ros::Subscriber sub=nh.subscribe("parkingOrder",10,&callback); //订阅入库指令

    communication::state msg;

    char rec[MAXSIZE];
    client.open();
    string dev;
    dev=ROOM_ID;
    client.init(UDP_TEST_PORT,UDP_SERVER_IP,rec,dev,"/home/user/log/record/zigbee.txt");

    int conn_time=0;
    ros::Rate rate(2);

    string t;
    string d;
    while(ros::ok())
    {
        if(!client.connect) //
        {
            printf("connect command\n");
            t=CONNECT_TYPE;
            d=CONNECT_DATA;
            client.sendInfo(t,d);
            client.wait_back(MAXSIZE,0,TIME_WRONG);

            cout<<endl;
            system("echo  $(date):[time out]communication  ,reconnect >> ~/log/moveRound_communication.log");
            conn_time++;
            if(conn_time>10)
            {
                msg.wrong_flag=true;  //通信故障
                msg.break_flag=true;
                msg.data="";
                pub_state.publish(msg);
                printf("communication failed\n");
                system("echo  $(date):[failed]communication  ,reconnect >> ~/log/moveRound_communication.log");
            }
            else if(!client.connect&&conn_time>4)
            {
                msg.wrong_flag=true;   //通信超时
                msg.break_flag=false;
                msg.data="";
                pub_state.publish(msg);
                printf(" communication wrong \n");
            }
        }
        else
        {
            printf("ask command\n");
            conn_time=0;
            t=ASK_TYPE;
            d=ASK_DATA;
            client.sendInfo(t,d);
            client.wait_back(MAXSIZE,0,TIME_WRONG);

            cout<<endl;
            if(client.receFlag)
            {
                msg.wrong_flag=false;
                msg.break_flag=false;
                msg.type=ASK_TYPE;
                msg.data=client.data_in;
                msg.lenth=client.len+1;//data lenth and type lenth
                pub_state.publish(msg);
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

bool sendCallback(communication::sendCmd::Request  &req,
                  communication::sendCmd::Response &res)
{
    printf("other command\n");
    printf("AAAAAAAAAAA\n");
    // send command and return somethin
    client.sendInfo(req.type,req.data);
    client.wait_back(MAXSIZE,0,TIME_WRONG);

    cout<<endl;

    res.receive=client.receFlag;
    res.state=client.data_in;//接受正常与否
    return true;
}
