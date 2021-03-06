#include "ros/ros.h"
#include "communication/command.h"
#include "processor/moveorder.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "pkg_srvs/SrvMode.h"
#include "pkg_srvs/SrvReturn.h"
#include "communication/state.h"
#include "communication/sendCmd.h"
#include "processor/isTurn.h"

#include "processor/parkingOrder.h"
#include <sstream>
#include <string.h>
#include <iostream>

using namespace std;
//指令类型
unsigned char msgtype2=0x10;  //启动配置指令
unsigned char msgtype3=0x20;  //巡检任务指令
unsigned char msgtype4=0x30;  //自由遥控指令
unsigned char msgtype5=0x40;  //路径遥控指令
unsigned char msgtype7=0x60;  //重启指令
unsigned char msgtype8=0x70;  //停车指令
unsigned char msgtype9=0x80;  //模式切换指令
unsigned char msgtype10=0x90; //返航指令
unsigned char msgtype11=0xA0; //照明设备指令
unsigned char msgtype12=0xB0; //出入库指令
unsigned char msgtype13=0xC0; //电池信息指令
unsigned char msgtype14=0xD0; //升降杆运动指令
unsigned char msgtype15=0xE0; //清除驱动器报警指令
unsigned char msgtype16=0xF0; //扬声器指令
//遥控指令分类
unsigned char ordtype1=0x31;  //遥控机器人运动指令
unsigned char ordtype2=0x32;  //调整机器人速度指令
unsigned char ordtype5=0x35;  //设置云台高度指令
unsigned char ordtype8=0x38;  //开启构图指令
unsigned char ordtype9=0x39;  //开启激光量测指令
unsigned char ordtypeA=0x3A;  //开启激光定位指令
unsigned char ordtypeB=0x3B;  //原地旋转度数
unsigned char ordtypeC=0x3C;  //原地旋转

//标号0x31消息（机器人遥控移动）内容分类
unsigned char movtype0=0x00;  //停止
unsigned char movtype1=0x01;  //前进
unsigned char movtype2=0x02;  //后退
unsigned char movtype3=0x04;  //左转
unsigned char movtype4=0x08;  //右转
unsigned char movtype5=0x05;  //前左
unsigned char movtype6=0x09;  //前右
unsigned char movtype7=0x06;  //后左
unsigned char movtype8=0x0A;  //后右

//机器人遥控模式下子设定的线速度与角速度
#define Vmax 120              //机器人最大线速度 单位：cm/s
//#define Vup 50              //机器人自设定基准前进线速度 单位：cm/s
#define Wlft 600              //机器人自设定基准左转角速度 单位：0.01rad/s
//#define Vdw -50             //机器人自设定基准后退线速度 单位：cm/s
#define Wrt -600              //机器人子设定基准右转角速度 单位：0.01rad/s

//指示灯颜色，状态 标号定义
#define RED      0x52         
#define GREEN    0x53
#define BLUE     0x54
#define SOUND    0x55
#define ON       0x01
#define OFF      0x00

// 旋转电机运动控制指令
char origin[23]={0x41,0x01,0x42,0x00,0x43,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x44,0x00};//回原位

char stop[23]={0x41,0x00,0x42,0x00,0x43,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
               0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x44,0x01};//急停

char hold[23]={0x41,0x00,0x42,0x01,0x43,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
               0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x44,0x00};//抱闸

char zero[23]={0x41,0x00,0x42,0x00,0x43,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
               0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x44,0x00}; //回0度

char turn[23]={0x41,0x00,0x42,0x00,0x43,0xFF,0xFF,0xEE,0x6C,0x00,0x00,0x11,0x94,
               0x00,0x00,0x11,0x94,0xFF,0xFF,0xEE,0x6C,0x44,0x00}; //旋转45度

class Processor
{
public:
    bool lightFlag;
    bool driveFlag;
    int  chargeGate;//需要充电的阈值
    bool roundFlag;

    Processor()
    {
        pub  = n.advertise<processor::moveorder>("moveorder", 1000);         //发布给运动控制模块
        pub2 = n.advertise<processor::isTurn>("processor/turn", 1000);       //发送导航模块，是否处于旋转状态
        pub3 = n.advertise<processor::parkingOrder>("parkingOrder", 1000);   //发送给出入库充电模块

        //Topic to subscribe
        sub1 = n.subscribe("communication/state_move",1, &Processor::callback1,this);          //接受机器人运控板使能信息
        sub2 = n.subscribe("communication/cmd", 5, &Processor::callback2, this);               //接收通讯模块信息
        sub3 = n.subscribe("communication/state_moveRound",10, &Processor::roundcallback,this);//接受机器人旋转运控板使能信息

        lightFlag=false;
        driveFlag=true;

        roundFlag=false;//旋转到达标志位
    }

    string strtf(int lenth, char *tfdata)  //数组转化为字符串；
    {
        string b="";
        int i=0;
        for(i=0;i<lenth;i++)
        {
            b+=tfdata[i];
        }

        return b;
    }

    string strtf(int lenth,unsigned char *tfdata)  //数组转化为字符串；
    {
        string b="";
        int i=0;
        for(i=0;i<lenth;i++)
        {
            b+=tfdata[i];
        }
        return b;
    }

    void sendCmdtoMove(ros::ServiceClient client, communication::sendCmd srv)
    {
        int count = 0;

        while(count<5)
        {
            if(client.call(srv)&&srv.response.receive)
            {
                //ROS_INFO("Processor:send command to Yunkongban success!");
                count = 5;
            }
            else
            {
                ROS_INFO("Processor:send command to Yunkongban again! %d", count);
                count++;
            }
        }
    }

    void sendCmdtoPLC(ros::ServiceClient client, communication::sendCmd srv)
    {
        int count = 0;

        while(count<5)
        {
            if(client.call(srv)&&srv.response.receive)
            {
                //ROS_INFO("Processor:send command to PLC success!");
                count = 5;
            }
            else
            {
                ROS_INFO("Processor:send command to PLC again! %d", count);
                count++;
            }
        }
    }

    void sendCmdtoRound(ros::ServiceClient client, communication::sendCmd srv)
    {
        int count = 0;

        while(count<5)
        {
            if(client.call(srv)&&srv.response.receive)
            {
                //ROS_INFO("Processor:send command to Xuanzhuanban success!");
                count = 5;
            }
            else
            {
                ROS_INFO("Processor:send command to Xuanzhuanban again! %d", count);
                count++;
            }
        }
    }

    void Light(char lightC,char light_state,ros::ServiceClient &client) //打开或者关闭报警灯
    {
        communication::sendCmd srvp;//发送给plc
        char io[2];
        io[0]=lightC;
        io[1]=light_state;
        srvp.request.type=0x50;
        srvp.request.lenth=2;
        srvp.request.data="";
        srvp.request.data+=strtf(srvp.request.lenth, io);
        sendCmdtoPLC(client, srvp);//重复发送未做
    }

    void HexDump(char *buf,int len,int addr) {
        int i,j,k;
        char binstr[80];

        for (i=0;i<len;i++) {
            if (0==(i%16)) {
                sprintf(binstr,"%08x -",i+addr);
                sprintf(binstr,"%s %02x",binstr,(unsigned char)buf[i]);
            } else if (15==(i%16)) {
                sprintf(binstr,"%s %02x",binstr,(unsigned char)buf[i]);
                sprintf(binstr,"%s  ",binstr);
                for (j=i-15;j<=i;j++) {
                    //       sprintf(binstr,"%s%c",binstr,('!'<buf[j]&&buf[j]<='~')?buf[j]:'.');
                }
                //printf("%s\n",binstr);
                ROS_INFO("Processor: Route: %s",binstr);
            } else {
                sprintf(binstr,"%s %02x",binstr,(unsigned char)buf[i]);
            }
        }
        if (0!=(i%16)) {
            k=16-(i%16);
            for (j=0;j<k;j++) {
                sprintf(binstr,"%s   ",binstr);
            }
            sprintf(binstr,"%s  ",binstr);
            k=16-k;
            for (j=i-k;j<i;j++) {
                //     sprintf(binstr,"%s%c",binstr,('!'<buf[j]&&buf[j]<='~')?buf[j]:'.');
            }
            //printf("%s\n",binstr);
            ROS_INFO("Processor: Route: %s",binstr);
        }
    }

    void roundcallback(const communication::state::ConstPtr& msg_moveRound)//接受机器人旋转运控板使能信息
    {
        //更新旋转电机状态
        unsigned char movstate[100];
        memcpy(movstate,msg_moveRound->data.c_str(),msg_moveRound->data.size()+1);
        if(movstate[32]==0x0f)//旋转是否到位
        {
            roundFlag=true;
        }
        else
            roundFlag=false;
    }

    void sendStop( ros::ServiceClient client )  //停止指令
    {
        char ctrmov[10];
        ctrmov[0]=0x41;//运动控制处理逻辑指令
        ctrmov[1]=0x00;
        ctrmov[2]=0x00;
        ctrmov[3]=0x42;
        ctrmov[4]=0x00;
        ctrmov[5]=0x00;
        ctrmov[6]=0x43;
        ctrmov[7]=0x01;
        ctrmov[8]=0x44;
        ctrmov[9]=0x00;
        communication::sendCmd srvm;//发送给运控板
        srvm.request.type=0x40;
        srvm.request.lenth=10;
        srvm.request.data="";
        srvm.request.data+=strtf(srvm.request.lenth, ctrmov);
        sendCmdtoMove(client, srvm);
        ROS_INFO("Processor:send STOP to move.");
    }
    void callback1(const communication::state::ConstPtr& msg_move)//接受机器人运控板使能信息
    {
        //更新机器人驱动器使能信息
        unsigned char movstate[100];
        memcpy(movstate,msg_move->data.c_str(),msg_move->data.size()+1);
		//宋凯
		//if(movstate[9]==0x0E || movstate[9]==0x0F)
		if(movstate[9]==0x07 || movstate[9]==0x0F)
		{
			driveFlag = true;
		}
		else
		{
			driveFlag = false;
		}
    }	
	
	
	void EnableMotor(char state,ros::ServiceClient clientm)
	{
		communication::sendCmd srvm;//发送给运控板
		srvm.request.type=0x70;
		srvm.request.lenth=1;
		srvm.request.data=state;
		sendCmdtoMove(clientm, srvm);
	}
	

    void callback2(const communication::command::ConstPtr& msg)//接收通讯模块信息
    {
        ros::ServiceClient clientp      = n.serviceClient<communication::sendCmd>("send_plc_cmd");//给PLC的发送消息的服务
        ros::ServiceClient clientm      = n.serviceClient<communication::sendCmd>("send_move_cmd");//给运动控制板发送消息的服务
        ros::ServiceClient clientmRound = n.serviceClient<communication::sendCmd>("send_moveRound_cmd");//给旋转控制板发送消息的服务
        ros::ServiceClient clientn      = n.serviceClient<pkg_srvs::SrvMode>("srv_mode");//调用导航模块服务
        ros::ServiceClient clientback   = n.serviceClient<pkg_srvs::SrvReturn>("srv_return"); //调用自主返航服务
	//190319测试时注释
        /*if (msg->break_flag)//通讯故障，返回充电屋,待处理
        {
            //报警处理
            ROS_INFO("Processor:communication fail with shangweiji!");
            Light(GREEN,OFF,clientp); //关闭绿色报警灯
            Light(RED,ON,clientp);    //打开红色报警灯
            lightFlag=false;

            processor::moveorder moveorder;
            moveorder.stopflag=true;
            pub.publish(moveorder);
            ros::Duration(1).sleep();

            //停止运动
            sendStop(clientm);
            //旋转电机回0度
            communication::sendCmd srvmR;
            srvmR.request.type=0x40;
            srvmR.request.lenth=23;
            srvmR.request.data="";
            srvmR.request.data+=strtf(srvmR.request.lenth, zero);
            sendCmdtoRound(clientmRound,srvmR);
            ROS_INFO("Processor:Go back to ROOM, because communication fault!");
            //printf("通讯故障，返回充电屋\n");

            //自主返航指令
            pkg_srvs::SrvReturn backhome;
            backhome.request.clear = false;
            moveorder.state="keep";
            moveorder.location="";
            moveorder.pose="";
            moveorder.speed="";
            moveorder.height="";
            moveorder.pointlevel="";
            moveorder.stopflag=false;
            moveorder.returnflag=1;

            if(clientback.call(backhome))
            {
                moveorder.location+=backhome.response.location;

                //printf("back home success\n");
                ROS_INFO("Processor:Go back to ROOM success!");
                char *sendBuf=(char *)moveorder.location.c_str();
                int out_len=moveorder.location.size();

                //调试用
                //cout<<"route :";
                HexDump(sendBuf,out_len,0);
            }

            moveorder.pointtype="";
            int i=0;
            for(;i<moveorder.location.size()/8;i++)
            {
                moveorder.pointtype+=0x01;
            }
            if(i>1)
            {
                pub.publish(moveorder);
            }
        }
        else if (msg->wrong_flag&&lightFlag)   //超时
        {
            ROS_INFO("Processor: waiting for communication reconnecting!\n");

            Light(GREEN,ON,clientp);//打开绿色灯
            Light(RED,ON,clientp);//打开红色灯，组合形成黄色
            lightFlag=false;
            //printf("open the yellow\n ");
            ROS_INFO("Processor:open the yellow");
        }
        else*/   //通信正常
        {ROS_INFO("Processor: \n");
            if(lightFlag==false)
            {
                Light(RED,OFF,clientp);   //关闭红色报警灯
                Light(SOUND,OFF,clientp); //关闭蜂鸣器
                Light(GREEN,ON,clientp);  //打开
                lightFlag=true;
            }

            char data[1000]={0};
            unsigned char data_lifter[10];
            memcpy(data,msg->data.c_str(),msg->data.size()+1);

            unsigned char testtype=msg->type[0];

            if(testtype==msgtype2)//启动参数配置//
            {
                char setp[11]={0};
                setp[0]=0x11;
                setp[1]=data[1];//电量阈值
                chargeGate=(int)data[1];

                setp[2]=0x12;
                setp[3]=data[3];//电量报警阈值
                setp[4]=0x13;
                setp[5]=data[5];//通信中断时间阈值
                setp[6]=0x14;
                setp[7]=data[7];//通讯故障时间阈值
                setp[8]=0x15;

                int tmp=0;
                tmp=(data[9]<<8|data[10])*10;
                setp[9]=tmp>>8;//机体温度
                setp[10]=tmp;//机体温度
                communication::sendCmd srvp;//发送给plc
                srvp.request.type=0x10;
                srvp.request.lenth=11;
                srvp.request.data="";
                srvp.request.data+=strtf(srvp.request.lenth, setp);
                sendCmdtoPLC(clientp, srvp);
                //配置指令给运控板
                char setmov[7];
                setmov[0]=0x11;
                setmov[1]=data[5];//通讯中断阈值
                setmov[2]=0x12;
                setmov[3]=data[7];//通讯故障阈值
                setmov[4]=0x13;
                setmov[5]=data[12];//超声波报警距离
                setmov[6]=data[13];//超声波报警距离
                communication::sendCmd srvm;//发送给运控板
                srvm.request.type=0x10;
                srvm.request.lenth=7;
                srvm.request.data="";
                srvm.request.data+=strtf(srvm.request.lenth, setmov);
                sendCmdtoMove(clientm,srvm);
            }
            else if(testtype==msgtype3||testtype==msgtype5||testtype==msgtype10)//巡检任务指令//
            {
                /******停止运动**********/
                sendStop(clientm);
                /******旋转电机回0度*****/
                communication::sendCmd srvmR;
                srvmR.request.type=0x40;
                srvmR.request.lenth=23;
                srvmR.request.data="";
                srvmR.request.data+=strtf(srvmR.request.lenth, zero);
                sendCmdtoRound(clientmRound,srvmR);

                ROS_INFO("Processor:Patrol and Inspect Order");
                processor::moveorder moveorder;
                moveorder.stopflag=true;
                pub.publish(moveorder);

                ros::Duration(3).sleep();

                moveorder.routeid=data[0];
                
                char datak[1000]={0};

                for(int dn=0;dn<999;dn++)
                {
                    datak[dn]=data[dn+1];
                }

                int i=0;
                char type[100];
                char *ty=type;
                char location[999];
                char *lo=location;
                char pose[999];
                char *po=pose;
                char speed[100];
                char *sp=speed;
                char height[100];
                char *he=height;
                char level[100];
                char *le=level;
                char *p;
                int k=msg->command_lenth/26;

                for(i=0;i<k;i++)
                {
                    p=datak+i*26;
                    *ty=*p;ty++;
                    for(int j=0;j<8;j++)
                    {
                        *lo=*(p+2);
                        lo++;
                        p++;
                    }
                    *po=*(p+3);po++;p++;
                    *po=*(p+3);po++;p++;
                    *he=*(p+12);he++;p++;
                    *he=*(p+12);he++;p++;
                    *sp=*(p+8);sp++;
                    *le=*(p+13);le++;
                }

                moveorder.state="keep";
                moveorder.pointtype="";
                moveorder.location="";
                moveorder.pose="";
                moveorder.speed="";
                moveorder.height="";
                moveorder.pointlevel="";
                
                moveorder.pointtype+=strtf(k, type);
                moveorder.location+=strtf(8*k, location);
                moveorder.pose+=strtf(2*k,pose);
                moveorder.speed+=strtf(k, speed);
                moveorder.height+=strtf(2*k, height);
                moveorder.pointlevel+=strtf(k, level);
                moveorder.stopflag = false;
                if(k>=1) //至少有一个点
                {
                    //cout<<"get "<<k<<"point\n";
                    ROS_INFO("Processor:Get %d Points and Publish them to move!", k);
                    pub.publish(moveorder);
                }
            }
            else if(testtype==msgtype4)//遥控指令//
            {
                char ctrmov[10];
                ctrmov[0]=0x41;//机器人方向
                ctrmov[3]=0x42;//机器人速度
                ctrmov[6]=0x43;//机器人急停指令
                ctrmov[8]=0x44;//机器人转弯

                int vec=0;
                vec = (int)data[3]*10;
                int dvec=0;
                dvec = (-1)*vec;

                //旋转电机控制
                communication::sendCmd srvmR;
                srvmR.request.type=0x40;
                srvmR.request.lenth=23;
                srvmR.request.data="";

                if(data[0]==ordtype1)//控制机器人移动
                {
                    if(data[1]==movtype0)//遥控下停止（00）
                    {
                        ctrmov[1]=0x00;
                        ctrmov[2]=0x00;
                        ctrmov[4]=0x00;
                        ctrmov[5]=0x00;
                        ctrmov[7]=0x00;
                        //printf("stop\n");
                        ROS_INFO("Processor: STOP by Remote control");
                    }
                    else if(data[1]==movtype1)//遥控下前进（01）
                    {
                        ctrmov[1]=vec>>8;
                        ctrmov[2]=vec;
                        ctrmov[4]=0x00;
                        ctrmov[5]=0x00;
                        ctrmov[7]=0x00;
                        ctrmov[9]=0x00;

                        //printf("go\n");
                        ROS_INFO("Processor: GO by Remote control");
                        /******旋转电机抱闸****/
                        srvmR.request.data+=strtf(srvmR.request.lenth, hold);
                        sendCmdtoRound(clientmRound,srvmR);//发送给旋转控制板
                    }
                    else if(data[1]==movtype2)//遥控下后退（02）
                    {
                        ctrmov[1]=dvec>>8;
                        ctrmov[2]=dvec;
                        ctrmov[4]=0x00;
                        ctrmov[5]=0x00;
                        ctrmov[7]=0x00;
                        ctrmov[9]=0x00;
                        /******旋转电机抱闸****/
                        ROS_INFO("Processor: GO BACK by Remote control");
                        srvmR.request.data+=strtf(srvmR.request.lenth, hold);
                        sendCmdtoRound(clientmRound,srvmR);//发送给旋转控制板
                    }
                    else if(data[1]==movtype3)//遥控下左转（04）
                    {
                        dvec=-80;
                        ctrmov[1]=dvec>>8;
                        ctrmov[2]=dvec;
                        ctrmov[4]=0x00;
                        ctrmov[5]=0x00;
                        ctrmov[7]=0x00;
                        ctrmov[9]=0x01;
                        //printf("left\n");
                        ROS_INFO("Processor: Turn Left by Remote control");
                    }
                    else if(data[1]==movtype4)//遥控下右转（08）
                    {
                        vec=80;
                        ctrmov[1]=vec>>8;
                        ctrmov[2]=vec;
                        ctrmov[4]=0x00;
                        ctrmov[5]=0x00;
                        ctrmov[7]=0x00;
                        ctrmov[9]=0x01;
                        ROS_INFO("Processor: Turn Right by Remote control");
                    }
                    else if(data[1]==movtype5)//遥控下前左（05）
                    {}
                    else if(data[1]==movtype6)//遥控下前右（09）
                    {}
                    else if(data[1]==movtype7)//遥控下后左（06）
                    {}
                    else if(data[1]==movtype8)//遥控下后右（0A）
                    {}
			//190319测试时注释
                    //if(roundFlag)
                    {
                        communication::sendCmd srvm;//发送给运控板
                        srvm.request.type=0x40;
                        srvm.request.lenth=10;
                        srvm.request.data="";
                        srvm.request.data+=strtf(srvm.request.lenth, ctrmov);
                        sendCmdtoMove(clientm, srvm);
                    }
                    /*else//190319测试时注释
                    {
                        //printf("turn not finish\n");
                        ROS_INFO("Processor: Turn NOT STOP by Remote control");
                    }*/
                }
                else if(data[0]==ordtypeC)//原地旋转
                {
                    //ROS_INFO("turn around!");
                    ROS_INFO("Processor: Turn around by Remote control");
                    communication::sendCmd srvmR;
                    srvmR.request.type=0x40;
                    srvmR.request.lenth=23;
                    srvmR.request.data="";
                    /******旋转电机旋转****/
                    if(data[1]==0x01)
                    {
						//190319测试时注释
						//宋
						//关闭左前电机使能
						//char state=0x06;
						//EnableMotor(state,clientm);
						//ROS_INFO("Move:Close motor");
                        
						srvmR.request.data+=strtf(srvmR.request.lenth, turn);
                        processor::isTurn msg;
                        msg.turnFlag=true;
                        pub2.publish(msg);
                        //printf("now in turn\n");
                        ROS_INFO("Processor: to spinTurn by Remote control");
                    }
                    else
                    {
						//190319测试时注释
						//打开左前右后电机使能
						//char state=0x0F;
						//EnableMotor(state,clientm);
						//ROS_INFO("Move:open motor");
						
                        srvmR.request.data+=strtf(srvmR.request.lenth, zero);
                        processor::isTurn msg;
                        msg.turnFlag=false;
                        pub2.publish(msg);
                        //printf("now go straight\n");
                        ROS_INFO("Processor: to StraightGO by Remote control");
                    }
                    sendCmdtoRound(clientmRound,srvmR);//发送给旋转控制板
                }
                else if(data[0]==ordtypeB)//原地旋转度数
                {
                    //ROS_INFO("turn around random!\n");
                    communication::sendCmd srvmR;
                    srvmR.request.type=0x40;
                    srvmR.request.lenth=23;
                    srvmR.request.data="";
                    /******旋转电机旋转****/
                    int angle=data[1]*100;
                    int neg_angle=0;

                    neg_angle=-1*(180-data[1])*100;

                    ROS_INFO("Processor: to spinTurn with specified angle %d by Remote control.", angle);
                    unsigned  char turnRandom[23]={0x41,0x00,0x42,0x00,0x43,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                                                   0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x44,0x00}; //任意度數

                    turnRandom[5]=angle>>24;turnRandom[6]=angle>>16;turnRandom[7]=angle>>8;turnRandom[8]=angle;
                    turnRandom[9]=angle>>24;turnRandom[10]=angle>>16;turnRandom[11]=angle>>8;turnRandom[12]=angle;
                    turnRandom[13]=angle>>24;turnRandom[14]=angle>>16;turnRandom[15]=angle>>8;turnRandom[16]=angle;
                    turnRandom[17]=angle>>24;turnRandom[18]=angle>>16;turnRandom[19]=angle>>8;turnRandom[20]=angle;

                    //  turnRandom[5]=neg_angle>>24;turnRandom[6]=neg_angle>>16;turnRandom[7]=neg_angle>>8;turnRandom[8]=neg_angle;
                    //  turnRandom[9]=angle>>24;turnRandom[10]=angle>>16;turnRandom[11]=angle>>8;turnRandom[12]=angle;
                    //  turnRandom[13]=angle>>24;turnRandom[14]=angle>>16;turnRandom[15]=angle>>8;turnRandom[16]=angle;
                    //  turnRandom[17]=neg_angle>>24;turnRandom[18]=neg_angle>>16;turnRandom[19]=neg_angle>>8;turnRandom[20]=neg_angle;

                    srvmR.request.data+=strtf(srvmR.request.lenth, turnRandom);
                    sendCmdtoRound(clientmRound,srvmR);//发送给旋转控制板
                }
                else if(data[0]==ordtype5)//操控云台
                {
                    communication::sendCmd srvmR;
                    srvmR.request.type=0x70;
                    srvmR.request.lenth=1;
                    srvmR.request.data="";

                    char io=0x00;
                    if(data[1]==0x00){io=0x00;cout<<"stop the yuntai"<<endl;}
                    else if(data[1]==0x01){io=0x01;cout<<"up the yuntai"<<endl;}
                    else if(data[1]==0x02){io=0x02;cout<<"down the yuntai"<<endl;}
                    srvmR.request.data.push_back(io);
                    sendCmdtoRound(clientmRound,srvmR);//发送给旋转控制板
                }
                else if(data[0]==ordtype8)//开启构图
                {
                    //导航模块
                    pkg_srvs::SrvMode srvn;

                    if(data[1]==0x01)
                    {
                        srvn.request.cmd="MAP_ON";
                        //printf("start  mapping\n");
                        ROS_INFO("Processor: Start Mapping");
                    }
                    else if(data[1]==0x00)
                    {
                        srvn.request.cmd="MAP_OFF";
                        //printf("close  mapping\n");
                        ROS_INFO("Processor:Close Mapping");
                    }
                    if(clientn.call(srvn)&&srvn.response.state)
                    {
                        ROS_INFO("Processor:send the up command about MAPPING to srv_mode success!");
                    }
                    else
                    {
                        ROS_INFO("Processor:send the up command about MAPPING to srv_mode again!");
                    }
                }
                else if(data[0]==ordtype9)//开启量测定位
                {
                    pkg_srvs::SrvMode srvn;
                    if(data[1]==0x01)
                    {
                        srvn.request.cmd="ODOM_ON";
                        //printf("start  odom\n");
                        ROS_INFO("Processor:Start Odometry.");
                    }
                    else if(data[1]==0x00)
                    {
                        srvn.request.cmd="ODOM_OFF";
                        //printf("close odom\n");
                        ROS_INFO("Processor:Close Odometry.");
                    }
                    if(clientn.call(srvn)&&srvn.response.state)
                    {
                        ROS_INFO("Processor:send the up command about ODOM to srv_mode success!");
                    }
                    else
                    {
                        ROS_INFO("Processor:send the up command about ODOM to srv_mode again!");
                    }
                }
                else if(data[0]==ordtypeA)//开启激光定位
                {
                    pkg_srvs::SrvMode srvn;
                    if(data[1]==0x01)
                    {
                        srvn.request.cmd="LASER_ON";
                        //printf("start laser \n");
                        ROS_INFO("Processor:Start Laser.");
                    }
                    else if(data[1]==0x00)
                    {
                        srvn.request.cmd="LASER_OFF";
                        //printf("close laser \n");
                        ROS_INFO("Processor:Close Laser.");
                    }
                    if(clientn.call(srvn)&&srvn.response.state)
                    {
                        ROS_INFO("Processor:send the up command about LASER to srv_mode success!");
                    }
                    else
                    {
                        ROS_INFO("Processor:send the up command about LASER to srv_mode again!");
                    }
                }
            }
            else if(testtype==msgtype7)//重启指令//
            {
                ROS_INFO("Processor: Restart!\n");
                communication::sendCmd srvm;//发送给运控板
                srvm.request.type=0x30;
                srvm.request.lenth=2;
                srvm.request.data=0x01;
                sendCmdtoMove(clientm, srvm);

                communication::sendCmd s;//发送给旋转运控板
                s.request.type=0x30;
                s.request.lenth=2;
                s.request.data=0x01;
                sendCmdtoRound(clientmRound,s);//发送给旋转控制板

                system("echo  $(date):restart the processor >> /home/user/log/processor.log");
                //system("sh /home/user/robot/src/shell/reboot.sh");
                system("echo 666666 | sudo -S reboot");

            }
            else if(testtype==msgtype8)//停车指令
            {
                ROS_INFO("Processor: STOP!");
                /******停止运动**********/
                sendStop(clientm);
                /******旋转电机急停****？
                communication::sendCmd srvmR;
                srvmR.request.type=0x40;
                srvmR.request.lenth=23;
                srvmR.request.data="";
                srvmR.request.data+=strtf(srvmR.request.lenth, stop);
                sendCmdtoRound(clientmRound,srvmR);
                */
                processor::moveorder move;
                move.stopflag=true;
                pub.publish(move);
                processor::parkingOrder  parkOrder;
                parkOrder.stopFlag = true ;
                pub3.publish(parkOrder);
            }
            else if(testtype==msgtype9)//模式切换指令
            {
                char data3=data[0];
                char data4=data[1];
                string modela="automatic";
                string modelb="controlled";
                char testa;
                testa=0x00;
                processor::moveorder moveorder;
                if(data3==0x00)
                {
                    moveorder.state+=modela;
                }
                else if(data3==0x01)
                {
                    moveorder.state+=modelb;
                }
                //printf("model change in------------------%s\n",moveorder.state.c_str());
                ROS_INFO("ROS_INFO:Control Mode change to %s.",moveorder.state.c_str());
                moveorder.pointtype+=testa;
                moveorder.location+=testa;
                moveorder.pose+=testa;
                moveorder.speed="";
                moveorder.height+=testa;
                moveorder.pointlevel+=testa;
                moveorder.stopflag = true;
                pub.publish(moveorder);

                /******停止运动**********/
                sendStop(clientm);
                /******旋转电机回0度*****/
                communication::sendCmd srvmR;
                srvmR.request.type=0x40;
                srvmR.request.lenth=23;
                srvmR.request.data="";
                srvmR.request.data+=strtf(srvmR.request.lenth, zero);
                sendCmdtoRound(clientmRound,srvmR);
            }
            else if(testtype==msgtype11)//照明设备指令
            {
                ROS_INFO("Processor:Open LED Light!\n");
                communication::sendCmd srvp;//发送给plc
                char io[3];
                io[0]=0x51;
                io[1]=data[0];
                srvp.request.type=0x50;
                srvp.request.lenth=2;
                srvp.request.data="";
                srvp.request.data+=strtf(srvp.request.lenth, io);
                sendCmdtoPLC(clientp, srvp);
            }
            else if(testtype==msgtype16)//扬声器设备指令
            {
                ROS_INFO("Processor:Open speaker!\n");
                communication::sendCmd srvp;//发送给plc
                char io[3];
                io[0]=0x31;
                io[1]=data[0];
                srvp.request.type=0x30;
                srvp.request.lenth=2;
                srvp.request.data="";
                srvp.request.data+=strtf(srvp.request.lenth, io);
                sendCmdtoPLC(clientp, srvp);
            }
            else if(testtype==msgtype12)//出入库指令
            {
                processor::parkingOrder  parkOrder; //多次按出入库
                parkOrder.stopFlag = true ;
                pub3.publish(parkOrder);

                ros::Duration(3).sleep();
                if(data[0]==0x01) //入库
                {
                    /******停止运动**********/
                    sendStop(clientm);
                    /******旋转电机回0度****/
                    communication::sendCmd srvmR;
                    srvmR.request.type=0x40;
                    srvmR.request.lenth=23;
                    srvmR.request.data="";
                    srvmR.request.data+=strtf(srvmR.request.lenth, zero);
                    sendCmdtoRound(clientmRound,srvmR);
                    //cout<<"in"<<endl;
                    pkg_srvs::SrvReturn backhome;
                    backhome.request.clear = true;
                    if(clientback.call(backhome))
                    {
                        //cout<<"clear back route"<<endl;
                        ROS_INFO("Processor:publish PARKIN order!");
                    }
                }
                else if(data[0]==0x00)//出库
                {
                    ros::Duration(30).sleep();//防止与充电机控制指令冲突；
                    ROS_INFO("Processor:send PARKOUT cmd!");
                    //cout<<"out"<<endl;
                }
                parkOrder.parkingOrder="";
                parkOrder.parkingOrder+=data[0];
                parkOrder.zigbeeComFlag=true;
                parkOrder.stopFlag=false;
                parkOrder.chargeGate=chargeGate;
                pub3.publish(parkOrder);
            }
            else if(testtype==msgtype14)//云台高度指令
            {
                memcpy(data_lifter,msg->data.c_str(),msg->data.size()+1);
                communication::sendCmd s;//发送给旋转运控板
                s.request.type=0x60;
                s.request.lenth=2;
                s.request.data.push_back(data_lifter[1]);
                s.request.data.push_back(data_lifter[2]);

                if(clientmRound.call(s)&&s.response.receive)
                {
                    ROS_INFO("Processor:send Yuntai High to moveRound success!");
                }
                else
                {
                    ROS_INFO("Processor:send Yuntai High to moveRound fail!");
                }
            }
            else if(testtype==msgtype15)//清除驱动器报警信息指令
            {
                communication::sendCmd srvm;//发送给运控板
                srvm.request.type=0x50;
                srvm.request.lenth=1;
                srvm.request.data=0x01;
                sendCmdtoMove(clientm, srvm);

                /****旋转电机清除报警*******/
                communication::sendCmd srvmR;
                srvmR.request.type=0x50;
                srvmR.request.lenth=1;
                srvmR.request.data=0x01;
                sendCmdtoRound(clientmRound,srvmR);

                //printf("clear alarm-state in drivers!\n");
                ROS_INFO("Processor:clear alarm-state in drivers!");
            }
        }
    }
private:
    ros::NodeHandle n;

    ros::Publisher pub;
    ros::Publisher pub2;
    ros::Publisher pub3;

    ros::Subscriber sub1;
    ros::Subscriber sub2;
    ros::Subscriber sub3;
};

int main(int argc, char**argv)
{
    ros::init(argc, argv, "processor");

    Processor SAPObject;
    ros::MultiThreadedSpinner s(4);//待测试
    ros::spin(s);

    return 0;
}
