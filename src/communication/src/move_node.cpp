/*通信模块: 与运动卡通信节点
 节点名: command_to_move
 订阅话题:无
 发布话题:communication/state_move
 服务:send_move_cmd
 功能:1.若未连接,则发送连接指令
          2.定时发布查询指令,并发布运动状态消息
          3.接受主控服务调用,发送所要求的指令
*/

#include <ros/ros.h>
#include <communication/udp_client.h>
#include <communication/sendCmd.h>
#include <communication/state.h>

#define	UDP_TEST_PORT    1032
#define UDP_SERVER_IP    "192.168.0.239"
#define MAXSIZE          1000
#define TIME_WRONG       200*1000//uSec

char MOVE_ID      = 0x01;
char CONNECT_TYPE = 0x00;
char CONNECT_DATA = 0X01;
char STATE_TYPE   = 0X20;
char STATE_DATA   = 0X01;

bool sendCallback(communication::sendCmd::Request  &req,
                  communication::sendCmd::Response &res) ;

UDP_Client client;

int main(int argc,char ** argv)
{
    ros::init(argc,argv,"move_node");
    ros::NodeHandle nh;
    ros::ServiceServer send_Cmd = nh.advertiseService("send_move_cmd", sendCallback);
    ros::Publisher pub_state=nh.advertise<communication::state>("communication/state_move",10);
    communication::state msg;

    char rec[MAXSIZE];
    client.open();
    string dev;
    dev=MOVE_ID;
    client.init(UDP_TEST_PORT,UDP_SERVER_IP,rec,dev,"/home/user/log/record/move.txt");

    int conn_time=0;
    ros::Rate rate(10);

    string t;
    string d;

    while(ros::ok())
    {
        if(!client.connect) //
        {
            ROS_INFO("move_node:send connect command\n");
            t=CONNECT_TYPE;
            d=CONNECT_DATA;
            client.sendInfo(t,d);
            client.wait_back(MAXSIZE,0,TIME_WRONG);

            conn_time++;
            if(conn_time>10)
            {
                msg.wrong_flag=true;  //通信故障
                msg.break_flag=true;
                msg.data.clear();
                pub_state.publish(msg);
            }
            else if(!client.connect&&conn_time>5)
            {
                msg.wrong_flag=true;   //通信超时
                msg.break_flag=false;
                msg.data.clear();
                pub_state.publish(msg);
            }
        }
        else
        {
            conn_time=0;
            t=STATE_TYPE;
            d=STATE_DATA;
            client.sendInfo(t,d);
            client.wait_back(MAXSIZE,0,TIME_WRONG);

            if(client.receFlag)
            {
                msg.wrong_flag=false;
                msg.break_flag=false;
                msg.type=STATE_TYPE;
                msg.data=client.data_in;
                msg.lenth=client.len+1;//data lenth and type lenth
                pub_state.publish(msg);
            }
            else
            {
                //cout<<"np send"<<endl;
                ROS_INFO("move_node:np send!");
            }
        }
        ros::spinOnce();
        rate.sleep();
        //cout<<"send time!"<<count<<endl;
        ROS_INFO("move_node:send time!");
    }
    return 0;
}

bool sendCallback(communication::sendCmd::Request  &req,
                  communication::sendCmd::Response &res)
{
    // send command and return somethin
    client.sendInfo(req.type,req.data);
    client.wait_back(MAXSIZE,0,TIME_WRONG);

    cout<<endl;

    res.receive = client.receFlag;
    res.state = client.data_in;//接受正常与否
    return true;
}
