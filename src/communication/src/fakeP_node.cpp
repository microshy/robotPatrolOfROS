/*模拟主控
调用plc服务
调用move服务

*/
#include "ros/ros.h"
#include <communication/sendCmd.h>
#include <communication/state.h>
#include <processor/lifterHeight.h>
#include <cstdlib>
using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fakeP_node");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<communication::sendCmd>("send_plc_cmd");
    ros::ServiceClient client1 = n.serviceClient<communication::sendCmd>("send_moveRound_cmd");
    ros::ServiceClient clientRoom = n.serviceClient<communication::sendCmd>("send_room_cmd");

    ros::Publisher pub=n.advertise<communication::state>("processor/state",10);
    ros::Publisher pub2 = n.advertise<processor::lifterHeight>("lifterHeight", 1);//发布给升降机模块

    communication::sendCmd srv;
    communication::sendCmd srv_plc;
    srv_plc.request.type+=0x60;
    srv_plc.request.lenth=1;
    srv_plc.request.data.push_back(0x01);

    communication::sendCmd srv_room;
    char gate=0x01;
    srv_room.request.type=0x50;
    srv_room.request.lenth=1;
    srv_room.request.data="";
    srv_room.request.data.push_back(gate);

    communication::state  msg;

    char origin[23]={0x41,0x01,0x42,0x00,0x43,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                     0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x44,0x00};//回原位

    char stop[23]={0x41,0x00,0x42,0x00,0x43,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                   0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x44,0x01};//急停

    char hold[23]={0x41,0x00,0x42,0x00,0x43,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                   0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x44,0x00};//抱闸

    char zero[23]={0x41,0x00,0x42,0x00,0x43,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                   0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x44,0x00}; //回0度

    char turn[23]={0x41,0x00,0x42,0x00,0x43,0x00,0x00,0x11,0x94,0x00,0x00,0x11,0x94,
                   0x00,0x00,0x11,0x94,0x00,0x00,0x11,0x94,0x44,0x00}; //旋转45度

    char turn1[23]={0x41,0x00,0x42,0x00,0x43,0xFF,0xFF,0x73,0x60,0xFF,0xFF,0x73,0x60,
                    0xFF,0xFF,0x73,0x60,0xFF,0xFF,0x73,0x60,0x44,0x00}; //旋转45度

    char turn60[23]={0x41,0x00,0x42,0x00,0x43,0x00,0x00,0x17,0x70,0x00,0x00,0x17,0x70,
                     0x00,0x00,0x17,0x70,0x00,0x00,0x17,0x70,0x44,0x00}; //旋转45度

    char turn3[23]={0x41,0x00,0x42,0x00,0x43,0xFF,0xFF,0x73,0x60,0xFF,0xFF,0x73,0x60,
                    0xFF,0xFF,0x73,0x60,0xFF,0xFF,0x73,0x60,0x44,0x00}; //旋转45度

    int result = turn[5]*16^3+turn[6]*16^2+turn[7]*16+turn[8];
    printf("%d",result);
    char cmd_type=0x40 ;
    
    srv.request.type+=cmd_type;
    for(int i=0;i<23;i++)
    {
        if(strcmp(argv[2],"origin")==0)
        { srv.request.data.push_back(origin[i]);}
        if(strcmp(argv[2],"turn")==0)
        { srv.request.data.push_back(turn[i]);}
        if(strcmp(argv[2],"zero")==0)
        { srv.request.data.push_back(zero[i]);}
        if(strcmp(argv[2],"60")==0)
        { srv.request.data.push_back(turn60[i]);}
        if(strcmp(argv[2],"hold")==0)
        { srv.request.data.push_back(hold[i]);}
    }

    int i=0;
    ros::Rate rate(2);
    if(strcmp(argv[1],"move")==0)
    {
        printf("move serve\n");
        if (client1.call(srv)&&srv.response.receive)
        {
            ROS_INFO("sucess");
        }
        else
        {
            ROS_ERROR("Failed ");
            return 1;
        }
    }
    else if(strcmp(argv[1],"pub")==0)
    {
        while (ros::ok())
        {
            printf("pub state to shangweiji\n");
            msg.data="";
            msg.data+=0x51;
            msg.data+=0x01;msg.data+=0x01;msg.data+=0x01;msg.data+=0x01;
            msg.data+=0x01;msg.data+=0x01;msg.data+=0x01;msg.data+=0x01;
            msg.data+=0x52;
            msg.data+=0x01;msg.data+=0x01;
            msg.data+=0x53;
            msg.data+=0x01;msg.data+=0x01;
            msg.data+=0x54;
            msg.data+=0x01;msg.data+=0x01;   msg.data+=0x01;msg.data+=0x01;
            msg.data+=0x55;
            msg.data+=0x01;msg.data+=0x01;
            msg.data+=0x56;
            msg.data+=0x01;
            msg.data+=0x57;
            msg.data+=0x01;
            msg.data+=0x58;
            msg.data+=0x01;
            msg.data+=0x59;
            msg.data+=0x01;
            msg.data+=0x5A;
            msg.data+=0x01;
            msg.data+=0x5B;
            msg.data+=0x01;
            msg.data+=0x01 ;
            msg.data+=0x5C;
            msg.data+=0x01;

            pub.publish(msg);
            i++;
            rate.sleep();
        }
    }
    else if(strcmp(argv[1],"lift")==0)
    {
        int height=0;
        printf(" lift serve\n");
        processor::lifterHeight msg;
        msg.height = height;
        printf("pub");
        pub2.publish(msg);
        ros::Duration(1).sleep();
        msg.height = height;
        printf("pub");
        pub2.publish(msg);
        ros::Duration(1).sleep();
        msg.height = height;
        printf("pub");
        pub2.publish(msg);
    }
    else if(strcmp(argv[1],"room")==0)
    {
        printf("room serve\n");
        if (clientRoom.call(srv_room)&&srv_room.response.receive)
        {
            ROS_INFO("room sucess");
        }
        else
        {
            ROS_ERROR("Failed ");
            return 1;
        }
    }
    else
    {
        printf("plc serve\n");
        if (client.call(srv_plc)&&srv_plc.response.receive)
        {
            ROS_INFO("plc sucess");
        }
        else
        {
            ROS_ERROR("Failed ");
            return 1;
        }
    }
    return 0;
}
