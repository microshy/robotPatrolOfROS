#include <ros/ros.h>
#include <sstream>
#include <cmath>
#include "tf/transform_broadcaster.h"
#include "communication/sendCmd.h"
#include "communication/state.h"
#include "processor/parkingOrder.h"
#include "processor/moveorder.h"                      //主控制器提供的消息
#include "geometry_msgs/PoseWithCovarianceStamped.h"  //导航模块的位姿消息
#include "geometry_msgs/PoseWithCovariance.h"         //服务里的数据类型
#include "pkg_srvs/SrvGetLine.h"                      //两点求直线服务
#include "pkg_srvs/SrvGetYawBias.h"                   //求偏行角服务
#include "processor/isTurn.h"                         //是否旋转
#include "pkg_srvs/SrvMode.h"
#include "stateandalarm/state.h"                      //发送给状态模块的消息
#include "move/charge.h"                              //充放电信息 放电为false
#include "pkg_back/MsgScan.h"

#include <string.h>
#include <iostream>
#include <fstream>
#include "move/inifile.h"
#include <time.h>
#include "pkg_back/state.h"

using namespace std;
using namespace inifile;

#define DRIVERENABLE 0x0C                             //定义驱动器使能状态
#define Vin -100.0                                    //定义机器人运行速度 mm/s
#define V 100.0
#define W 150.0                                       //定义机器人运行角速度 0.001rad/s
#define T 0.02                                        //定义每次发送速度指令的时间周期
#define movecount 1                                  
#define a 0.5
#define b 0.5 
#define Kx 1
#define Ky 5
#define K 1

#define CONNECTED 0x01
#define CONNECTING 0x00

#define GATEOPEN 0x01
#define GATECLOSED 0x00

#define PARKIN 0x01
#define PARKOUT 0x00

#define STOPTH 0.02                                   //旋转允许误差角度（开始停止角度）  单位 rad
#define SLOWTH 0.3 

#define CHARGEDIS 0.1
#define STOPD 0.02                                    //停车距离    单位 m
#define SLOWD1 1.0                                    //第一次减速距离   单位 m
#define SLOWDTEST 0.5

#define WAIT_TIME    90                               //等待云台自检时间90s
#define ON           0x01
#define OFF          0x00

const string filepath = "/home/robot/.nav/roomConfig.ini";

struct point
{
    double x;
    double y;
    point(double _x,double _y):x(_x),y(_y){}
    point(){}
};

struct room
{
    point room_in;
    point room_out;
    room(point _in,point _out):room_in(_in),room_out(_out)
    { }
    room(){}
};

class Parking
{
public:

    bool fullFlag;
    bool positive_curr;
    bool stopFlag;
    bool chargeFlag;
    bool driveFlag;
    bool connectFlag;
    bool gateOpenFlag;
    bool gateCloseFlag;
    bool inroomFlag;
    bool justParkoutFlag;//刚刚出库，尚未巡检标志
    bool enterfixFlag;//刚出库就入库补偿标志
    bool roundFlag;//旋转到达标志位
    bool to90angle=false; //转到90度标记
    bool to45angle=false; //转到45度标记
    bool to0angle=false;  //转到0度标记
    string arvstate; //说明：此数据发送给状态备份模块,为是否到达目标点;
    double x0;//表示机器人实时的x坐标
    double y0;//表示机器人实时y坐标
    double th0;//表示机器人实时姿态角

    double error_theta;//机器人调整角度
    double error_theta1;//机器人调整角度
    double distance_r; //距离右边的距离
    double distance_m; //距离正前方距离
    double distance_real;//斜边距离
    int chargeGate;
    int dianliang;//current electric quantity

    room my_room;
    point room_point; //屋内点激光距离参数配置

    double DISTANCE_OUT_ROOM; //出库调整的两个参数
    double DISTANCE_OUT;
    char motorState;
    
    Parking()
    {
        x0=0.0;
        y0=0.0;
        th0=0.0;
        chargeGate=95;  //充电阈值，低于该值开始充电
        dianliang=0;    //实时电量
        chargeFlag=false;
        driveFlag=true;
        gateOpenFlag=false;
        gateCloseFlag=false;
        connectFlag=false;
        
        fullFlag=false;         //是否充满
        positive_curr=true;     //电流是否为正
        justParkoutFlag = false;//刚出库，尚未巡检标志，初始值为false;
        enterfixFlag = false;   //刚出库，就入库补偿标志，初始值为false;
        stopFlag=false;
        roundFlag=false;
        inroomFlag=false;

        //加载配置
        IniFile ini;
        ini.load(filepath);
        int ret = 0;
        point in(ini.getDoubleValue("POINT", "ROOM_IN_X", ret),ini.getDoubleValue("POINT", "ROOM_IN_Y", ret));
        point out(ini.getDoubleValue("POINT", "ROOM_OUT_X", ret),ini.getDoubleValue("POINT", "ROOM_OUT_Y", ret));

        my_room.room_in = in;
        my_room.room_out = out;

        room_point.x=ini.getDoubleValue("DISTANCE", "DISTANCE_IN_X", ret);
        room_point.y=ini.getDoubleValue("DISTANCE", "DISTANCE_IN_Y", ret);

        DISTANCE_OUT=ini.getDoubleValue("DISTANCE", "DISTANCE_OUT", ret);
        DISTANCE_OUT_ROOM=ini.getDoubleValue("DISTANCE", "DISTANCE_OUT_ROOM", ret);

        //cout<<DISTANCE_OUT<<" "<<DISTANCE_OUT_ROOM<<endl;

        //topic to publish
        pub1 = n.advertise<processor::parkingOrder>("parkingOrder", 1000);  //发送给出入库充电模块
        pub_turn = n.advertise<processor::isTurn>("processor/turn", 1000);  //发送导航模块，是否处于旋转状态
        pub_state = n.advertise<stateandalarm::state>("state_route", 1000);//发送给状态模块
        pub_chargeState = n.advertise<move::charge>("parking/chargeState", 1000);//发送给状态模块 是否处于充电状态
        //topic to subscribe
        sub1 = n.subscribe("parkingOrder",5,&Parking::parkingcallback,this);       //接收主控板消息
        sub2 = n.subscribe("topic_robot_pose",5,&Parking::posecallback,this);      //接收导航模块实时位姿
        sub3 = n.subscribe("communication/state_move",1, &Parking::driverFlagcallback,this);//接受机器人运控板使能信息
        sub4 = n.subscribe("communication/state_room",1, &Parking::chargeFlagcallback,this);//接受充电屋状态信息
        sub5 = n.subscribe("communication/state_moveRound",10, &Parking::callback2,this);//接受机器人运控板使能信息
        sub6 = n.subscribe("communication/state_plc", 1, &Parking::statecallback3,this); //plc信息
        sub7 = n.subscribe("topic_scan_back", 1, &Parking::laser_callback,this);         //激光数据
        sub8 = n.subscribe("pkg_back/chargeFull",10,&Parking::callback3,this);
        //service to be used
        client1 = n.serviceClient<pkg_srvs::SrvGetLine>("srv_get_line");    //求直线及角度
        clientm = n.serviceClient<communication::sendCmd>("send_move_cmd"); //给运动控制板发送消息的服务
        clientp = n.serviceClient<communication::sendCmd>("send_plc_cmd");  //给PLC的发送消息的服务
        clientr = n.serviceClient<communication::sendCmd>("send_room_cmd"); //给充电屋发指令
        clientmRound = n.serviceClient<communication::sendCmd>("send_moveRound_cmd"); //给旋转运动控制板发送消息的服务
        clientn = n.serviceClient<pkg_srvs::SrvMode>("srv_mode");//调用导航模块服务
    }

    void sendCmdtoPLC(ros::ServiceClient client, communication::sendCmd srv)
    {
        int count = 0;

        while(count<10)
        {

            if(client.call(srv)&&srv.response.receive)
            {
                //ROS_INFO("Park:send cmd to plc success.");
                count = 10;
            }
            else
            {
                ROS_INFO("Park:send cmd to plc again! %d",count);
                count++;
            }
        }
    }

    void sendCmdtoRoom(ros::ServiceClient client, communication::sendCmd srv)
    {
        int count = 0;

        while(count<5)
        {
            if(client.call(srv)&&srv.response.receive)
            {
                //ROS_INFO("Park:send cmd to ROOM success");
                count = 5;
            }
            else
            {
                ROS_INFO("Park:send cmd to ROOM again! %d",count);
                count++;
            }
        }
    }

    void sendCmdtoLaser(ros::ServiceClient client, pkg_srvs::SrvMode srvn)
    {
        int count = 0;
        while(count<5)
        {
            if(client.call(srvn)&&srvn.response.state)
            {
                //ROS_INFO("Park:send cmd to laser_node success");
                count = 5;
            }
            else
            {
                ROS_INFO("Park:send cmd to laser_node again! %d",count);
                count++;
            }
        }
    }

    void sendCmdtoMove(ros::ServiceClient client, communication::sendCmd srv)
    {
        int count = 0;

        while(count<5)
        {
            if(driveFlag)
            {
                if(client.call(srv)&&srv.response.receive)
                {
                    //ROS_INFO("Park:connect with move_node success");
                    count = 5;
                }
                else
                {
                    ROS_INFO("Park:send cmd to move_node again! %d",count);
                    count++;
                }
            }
            else
            {
                ROS_INFO("Park:move driver is unabled!\n");
                char ctrmov[10];
                ctrmov[0]=0x41;//运动控制处理逻辑指令
                ctrmov[1]=0x00;
                ctrmov[2]=0x00;
                ctrmov[3]=0x42;
                ctrmov[4]=0x00;
                ctrmov[5]=0x00;
                ctrmov[6]=0x43;
                ctrmov[7]=0x00;
                ctrmov[8]=0x44;
                ctrmov[9]=0x00;

                srv.request.type=0x40;
                srv.request.lenth=10;
                srv.request.data="";
                srv.request.data+=strtf(srv.request.lenth, ctrmov);
                if(client.call(srv)&&srv.response.receive)
                {
                    count = 5;
                }
                else
                {
                    ROS_INFO("Park:send cmd to move fail! %d",count);
                    count++;
                }
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
                //ROS_INFO("Parking_node:connect with moveround_node success");
                count = 5;
            }
            else
            {
                ROS_INFO("Park:send cmd to moveround_node again! %d",count);
                count++;
            }
        }
    }

    string strtf(int lenth, char *aa)//用于将字符数组的数据赋给string类型变量
    {
        string bb="";
        int i=0;
        for(i=0;i<lenth;i++)
        {
            bb+=aa[i];
        }
        return bb;
    }

    void pubstate()
    {
        stateandalarm::state statepub;
        statepub.type=0x50;
        statepub.lenth=1;
        statepub.data="";
        statepub.data+=arvstate;
        pub_state.publish(statepub);//发布信息给状态模块
    }

    void statecallback3(const communication::state::ConstPtr& msg3)//获取PLC板信息；
    {
        dianliang=(int)msg3->data[22];
    }

    void driverFlagcallback(const communication::state::ConstPtr& msg_move)
    {
        //更新机器人驱动器使能信息
        unsigned char movstate[100];

        memcpy(movstate,msg_move->data.c_str(),msg_move->data.size()+1);

		 //宋
		if(movstate[9]==0x07 || movstate[9]==0x0F)
		{
			driveFlag = true;
		}
		else
		{
			driveFlag = false;
		}
		motorState=movstate[9];
    }

    void chargeFlagcallback(const communication::state::ConstPtr& msg_room)
    {
        //更新充电屋插头接触状态
        unsigned char roomstate[100];

        memcpy(roomstate,msg_room->data.c_str(),msg_room->data.size()+1);

        if(roomstate[12]==CONNECTED)  //插头接触状态
        {
            connectFlag = true;
        }
        else if(roomstate[12]==CONNECTING)
        {
            connectFlag = false;
        }

        if(roomstate[8]==GATEOPEN)  //门状态
        {
            gateOpenFlag = true;
        }
        else if(roomstate[8]==GATECLOSED)
        {
            gateOpenFlag = false;
        }
        if(roomstate[10]==GATEOPEN)  //门状态
        {
            gateCloseFlag = true;
        }
        else if(roomstate[10]==GATECLOSED)
        {
            gateCloseFlag = false;
        }
        if(roomstate[16]==0x01)  //充电继电器状态 01在充电
        {
            chargeFlag = true;
        }
        else if(roomstate[16]==0x00)
        {
            chargeFlag = false;
        }
    }

    void callback2(const communication::state::ConstPtr& msg_moveRound)
    {

        //更新旋转电机状态
        unsigned char movstate[100];
        memcpy(movstate,msg_moveRound->data.c_str(),msg_moveRound->data.size()+1);
        int angle1=(movstate[34]<<24)+(movstate[35]<<16)+(movstate[36]<<8)+movstate[37]; //计算左前旋转电机角度，45度或者0度
        int angle2=(movstate[38]<<24)+(movstate[39]<<16)+(movstate[40]<<8)+movstate[41]; //计算右前旋转电机角度，45度或者0度
        int angle3=(movstate[42]<<24)+(movstate[43]<<16)+(movstate[44]<<8)+movstate[45]; //计算左后旋转电机角度，45度或者0度
        int angle4=(movstate[46]<<24)+(movstate[47]<<16)+(movstate[48]<<8)+movstate[49]; //计算右后旋转电机角度，45度或者0度
        int angle=(abs(angle1)+abs(angle2)+abs(angle3)+abs(angle4))/4;

        bool flag=false;
        if(to90angle)
        {
            if(abs(angle-9000)<100)
                flag=true;

            if(movstate[32]==0x0f && flag)//旋转是否到位
            {
                roundFlag=true;
                to90angle=false;
            }
            else
                roundFlag=false;
        }
        else if(to45angle)
        {
            if(abs(angle-4500)<100)
                flag=true;

            if(movstate[32]==0x0f && flag)//旋转是否到位
            {
                roundFlag=true;
                to45angle=false;
            }
            else
                roundFlag=false;
        }
        else if(to0angle)
        {
            if(abs(angle)<100)
                flag=true;

            if(movstate[32]==0x0f && flag)//旋转是否到位
            {
                roundFlag=true;
                to0angle=false;
            }
            else
                roundFlag=false;
        }
    }

    void posecallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg1)//位姿消息的回调函数,返回实时位姿//需要设定订阅频率
    {
        x0 = msg1->pose.pose.position.x ;//机器人实时x坐标
        y0 = msg1->pose.pose.position.y ;//机器人实时y坐标
        th0 = tf::getYaw(msg1->pose.pose.orientation);//机器人实时姿态角
        if((x0>0.1)||(y0>0.1))
            justParkoutFlag = false;
    }

    double getYawBias(double aimtheta) // 求去偏航角 单位 rad 从机器人角度指向目标角度 逆时针方向为正
    {
        double yawbias=0;
        double yawb=0;

        yawb=th0-aimtheta;
        if(((-2*M_PI)<=yawb)&&(yawb<(-1*M_PI))){yawbias=-2*M_PI-yawb;}
        else if(((-1*M_PI)<=yawb)&&(yawb<0)){yawbias=-1*yawb;}
        else if((0<=yawb)&&(yawb<M_PI)){yawbias=-1*yawb;}
        else if((M_PI<=yawb)&&(yawb<=2*M_PI)){yawbias=2*M_PI-yawb;}

        //ROS_INFO("Park:current:%f,target :%f ,thetatoturn :%f",th0,aimtheta,yawbias);
        return yawbias;
    }

    double getTHe(double targettheta,double realth)
    {
        double TH1=0.0;
        double TH2=0.0;

        TH1=targettheta-realth;
        if(((-2*M_PI<=TH1))&&(TH1<(-M_PI)))  {TH2=TH1+2*M_PI;}
        else if((-M_PI<=TH1)&&(TH1<=M_PI))   {TH2=TH1;}
        else if((M_PI<TH1)&&(TH1<=(2*M_PI))) {TH2=-2*M_PI+TH1;}

        return TH2;
    }

	void EnableMotor(char state)
	{
		communication::sendCmd srvm;//发送给运控板
		srvm.request.type=0x70;
		srvm.request.lenth=1;
		srvm.request.data=state;
		sendCmdtoMove(clientm, srvm);
	}
	
    void spinfun(double targetth) //控制机器人旋转
    {
        communication::sendCmd automov;
        char ctrmov2[10]={0};//转动停止
        ctrmov2[0]=0x41;     //运动控制处理逻辑指令
        ctrmov2[1]=0x00;
        ctrmov2[2]=0x00;
        ctrmov2[3]=0x42;
        ctrmov2[4]=0x00;
        ctrmov2[5]=0x00;
        ctrmov2[6]=0x43;
        ctrmov2[7]=0x00;
        ctrmov2[8]=0x44;
        ctrmov2[9]=0x00;

        automov.request.type=0x40;
        automov.request.lenth=10;
        automov.request.data="";
        automov.request.data+=strtf(automov.request.lenth,ctrmov2);
        sendCmdtoMove(clientm, automov);
        ros::Duration(2).sleep();

        double thetatoturn=0;
        thetatoturn=getYawBias(targetth);//偏航角

        if(fabs(thetatoturn)<=0.02)   //先旋转到正确角度
        {
            ROS_INFO("Park:There is no need turn\n");
            ros::Duration(2).sleep();//待定
            return;
        }

        char hold[23]={0x41,0x00,0x42,0x01,0x43,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                       0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x44,0x00};//抱闸

        char zero[23]={0x41,0x00,0x42,0x00,0x43,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                       0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x44,0x00}; //回0度

        char turn[23]={0x41,0x00,0x42,0x00,0x43,0xFF,0xFF,0xEE,0x6C,0x00,0x00,0x11,0x94,
                       0x00,0x00,0x11,0x94,0xFF,0xFF,0xEE,0x6C,0x44,0x00}; //旋转45度

	/*//关闭左前右后电机使能
	char state=0x06;
	EnableMotor(state);
	ROS_INFO("Parking:Close motor");		   
	//等待电关闭
	while(motorState!=0x06)
	{   
	    ros::spinOnce();
            ros::Duration(0.02).sleep();
	}*/			   
        // 角度标记
        to90angle = false;
        to45angle = true;
        to0angle  = false;
        // 先调整轮子角度
        communication::sendCmd srvmR;
        srvmR.request.type=0x40;
        srvmR.request.lenth=23;
        srvmR.request.data.clear();
        srvmR.request.data+=strtf(srvmR.request.lenth, turn);
        sendCmdtoRound(clientmRound,srvmR);//电机旋转45度
        ros::Duration(2).sleep();//待定

        // 等待旋转到位
        while(!roundFlag)
        {
            ros::spinOnce();
            ros::Duration(0.02).sleep();
        }
        
        roundFlag = false; //等待旋转到位后旋转到位标记置0；
        processor::isTurn msg;
        msg.turnFlag=true;
        pub_turn.publish(msg);
        ROS_INFO("Park:Now robot in spinfun turn\n");

        while(thetatoturn>STOPTH||thetatoturn<(-STOPTH))   //先旋转到正确角度
        {
            ros::spinOnce();
            if(stopFlag){
                //char ctrmov2[10]={0};//转动停止
                ctrmov2[0]=0x41;//运动控制处理逻辑指令
                ctrmov2[1]=0x00;
                ctrmov2[2]=0x00;
                ctrmov2[3]=0x42;
                ctrmov2[4]=0x00;
                ctrmov2[5]=0x00;
                ctrmov2[6]=0x43;
                ctrmov2[7]=0x00;
                ctrmov2[8]=0x44;
                ctrmov2[9]=0x00;

                automov.request.type=0x40;
                automov.request.lenth=10;
                automov.request.data="";
                automov.request.data+=strtf(automov.request.lenth,ctrmov2);
                sendCmdtoMove(clientm, automov);
                break;
            }
            int wfirst=0;
            double spinslowrate=1.0;
            if(((thetatoturn>STOPTH)&&(thetatoturn<SLOWTH))||((thetatoturn<(-STOPTH))&&(thetatoturn>(-SLOWTH))))
            {
                spinslowrate=(0.8/0.38)*fabs(thetatoturn)+1-0.8/0.38*0.4;
                //ROS_INFO("Park:slow rate%f",spinslowrate);
            }

            //判断旋转翻方向
            if(thetatoturn>0){wfirst=(-1)*W*spinslowrate;}
            if(thetatoturn<0){wfirst=W*spinslowrate;}
            //待修改
            char ctrmov1[10]={0};
            ctrmov1[0]=0x41;//运动控制处理逻辑指令
            ctrmov1[1]=wfirst>>8;
            ctrmov1[2]=wfirst;
            ctrmov1[3]=0x42;
            ctrmov1[4]=0x00;
            ctrmov1[5]=0x00;
            ctrmov1[6]=0x43;
            ctrmov1[7]=0x00;
            ctrmov1[8]=0x44;
            ctrmov1[9]=0x01;

            automov.request.type=0x40;
            automov.request.lenth=10;
            automov.request.data="";
            automov.request.data+=strtf(automov.request.lenth,ctrmov1);
            sendCmdtoMove(clientm, automov);
            thetatoturn=getYawBias(targetth);//偏航角
        }
        //char ctrmov2[10]={0};//转动停止
        ctrmov2[0]=0x41;//运动控制处理逻辑指令
        ctrmov2[1]=0x00;
        ctrmov2[2]=0x00;
        ctrmov2[3]=0x42;
        ctrmov2[4]=0x00;
        ctrmov2[5]=0x00;
        ctrmov2[6]=0x43;
        ctrmov2[7]=0x00;
        ctrmov2[8]=0x44;
        ctrmov2[9]=0x00;

        automov.request.type=0x40;
        automov.request.lenth=10;
        automov.request.data="";
        automov.request.data+=strtf(automov.request.lenth,ctrmov2);
        sendCmdtoMove(clientm, automov);

	/*//打开左前右后电机使能
	state=0x0F;
	EnableMotor(state);
	ROS_INFO("Parking:open motor");
	//等待电机开启
	while(motorState!=0x0F)
	{   
	    ros::spinOnce();
            ros::Duration(0.02).sleep();
	}*/
		
        // 角度标记
        to90angle = false;
        to45angle = false;
        to0angle  = true;
        srvmR.request.data.clear();
        srvmR.request.data+=strtf(srvmR.request.lenth, zero);
        sendCmdtoRound(clientmRound,srvmR);//电机恢复直行状态

        ros::Duration(2).sleep();//待定
        // 等待旋转到位
        while(!roundFlag)
        {
            ros::spinOnce();
            ros::Duration(0.02).sleep();
        }
        roundFlag = false;  //等待旋转到位后旋转到位标记置0；

        //抱闸 直线
        srvmR.request.data.clear();
        srvmR.request.data+=strtf(srvmR.request.lenth, hold);
        sendCmdtoRound(clientmRound,srvmR);//电机抱闸

        ROS_INFO("Park:Spin fun success!");
        msg.turnFlag=false;
        pub_turn.publish(msg);
    }

    void laser_callback(const pkg_back::MsgScan::ConstPtr& msg)
    {
        error_theta=(msg->lf - msg->lr);//需调整角度
        error_theta1=(msg->clf - msg->clr);
        distance_r=msg->dlf;

        int count_set=5;
        static int count=0;
        static double sum_right=0.0;
        static double sum_mid=0.0;
        count++;

        sum_mid+=msg->dis;
        if(count==count_set)
        {
            count=0;
            distance_m=sum_mid/count_set;  //距离正前方的距离
            sum_mid=0;
            sum_right=0;
        }
        //cout<<"distance_r "<<distance_r<<"distance_m "<<distance_m<<endl;
        distance_real=msg->ch;  //出库时使用，距离屋子边沿的距离
    }

    void move_(int vvt)
    {
        communication::sendCmd automov;
        char ctrmov3[10]={0};
        ctrmov3[0]=0x41;//运动控制处理逻辑指令
        ctrmov3[1]=vvt>>8;
        ctrmov3[2]=vvt;
        ctrmov3[3]=0x42;
        ctrmov3[4]=0;
        ctrmov3[5]=0;
        ctrmov3[6]=0x43;
        ctrmov3[7]=0x00;
        ctrmov3[8]=0x44;
        ctrmov3[9]=0x00;
        automov.request.type=0x40;
        automov.request.lenth=10;
        automov.request.data="";
        automov.request.data+=strtf(automov.request.lenth,ctrmov3);
        sendCmdtoMove(clientm, automov);
    }

    void adjust_ydistance(double goal,int v)
    {
        //cout<<goal<<endl;
        double derror=goal-distance_m;

        while(1)   //调整y位置
        {
            if(stopFlag){break;}
            ros::spinOnce();
            if(distance_m>goal)
                break;
            move_(v);
        }
        move_(0);
        ROS_INFO("Park:adjust Y distance success!");
    }

    void out_adjust_ydistance_inroom(double goal,int v)
    {
        //cout<<goal<<endl;
        double derror=goal-distance_m;

        if(gateCloseFlag==false)//如果门关信号为真
        {
            char close=0x02;
            door(close);
        }
        if(distance_m>0.95)
        {
            ROS_INFO("Park:Robot may not be in ROOM!");
            inroomFlag = false;
        }
        else
        {
            inroomFlag = true;
            while(1)   //调整y位置
            {
                if(stopFlag){break;}
                ros::spinOnce();
                if(distance_m<goal)
                    break;
                move_(v);
            }
            move_(0);
            ROS_INFO("Park:Parkout adjust Y distance in ROOM success!");
        }
    }

    void out_adjust_ydistance(double goal,int v)
    {

        double derror;

        derror = goal-distance_m;
        while(1)   //调整y位置
        {
            if(stopFlag){break;}
            ros::spinOnce();
            derror = goal-distance_m;
            if(fabs(derror)<0.03)
                break;
            if(derror>0)
                move_(v);
            else
                move_(-v);
        }
        move_(0);
        ROS_INFO("Park:out_room adjust Y distance success");
    }

    void adjust_angle()
    {
        if(stopFlag) {return;}

        communication::sendCmd automov;
        double thetatoturn=0;
        thetatoturn=error_theta;

        char hold[23]={0x41,0x00,0x42,0x01,0x43,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                       0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x44,0x00};//抱闸
        char zero[23]={0x41,0x00,0x42,0x00,0x43,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                       0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x44,0x00}; //回0度
        char turn[23]={0x41,0x00,0x42,0x00,0x43,0xFF,0xFF,0xEE,0x6C,0x00,0x00,0x11,0x94,
                       0x00,0x00,0x11,0x94,0xFF,0xFF,0xEE,0x6C,0x44,0x00}; //旋转45度
	
	/*//关闭左前右后电机使能
	char state=0x06;
	EnableMotor(state);
	ROS_INFO("Parking:Close motor");		  
	
	//等待电关闭
	while(motorState!=0x06)
	{
	    ros::spinOnce();
            ros::Duration(0.02).sleep();
	}*/
				   
        // 角度标记
        to90angle = false;
        to45angle = true;
        to0angle  = false;
        // 先调整轮子角度
        communication::sendCmd srvmR;
        srvmR.request.type=0x40;
        srvmR.request.lenth=23;
        srvmR.request.data.clear();
        srvmR.request.data+=strtf(srvmR.request.lenth, turn);
        sendCmdtoRound(clientmRound,srvmR);
        ros::Duration(2).sleep();//待定

        // 等待旋转到位
        while(!roundFlag)
        {
            ros::spinOnce();
            ros::Duration(0.02).sleep();
        }
        roundFlag = false;  //等待旋转到位后旋转到位标记置0；
        processor::isTurn msg;
        msg.turnFlag=true;
        pub_turn.publish(msg);
        ROS_INFO("Park:Now in adjust_angle turn.");

        while(fabs(thetatoturn)>0.01)   //先旋转到正确角度
        {
            ros::spinOnce();
            if(stopFlag)
            {
                move_(0);
                break;
            }

            int wfirst=0;

            //判断旋转翻方向
            if(thetatoturn>0){wfirst=(-1)*50;}
            if(thetatoturn<0){wfirst=50;}
            //待修改
            char ctrmov1[10]={0};
            ctrmov1[0]=0x41;//运动控制处理逻辑指令
            ctrmov1[1]=wfirst>>8;
            ctrmov1[2]=wfirst;
            ctrmov1[3]=0x42;
            ctrmov1[4]=0x00;
            ctrmov1[5]=0x00;
            ctrmov1[6]=0x43;
            ctrmov1[7]=0x00;
            ctrmov1[8]=0x44;
            ctrmov1[9]=0x01;

            automov.request.type=0x40;
            automov.request.lenth=10;
            automov.request.data="";
            automov.request.data+=strtf(automov.request.lenth,ctrmov1);
            sendCmdtoMove(clientm, automov);

            thetatoturn=error_theta;
            //cout<<"error_theta "<<error_theta<<endl;
            ROS_INFO("Park:error_theta is %.3f",error_theta);

        }
		
	/*//打开左前右后电机使能
	state=0x0F;
	EnableMotor(state);
	ROS_INFO("Parking:open motor");
	
	//等待开启
	while(motorState!=0x0F)
	{   
	    ros::spinOnce();
            ros::Duration(0.02).sleep();
	}*/
        // 角度标记
        to90angle = false;
        to45angle = false;
        to0angle  = true;
        move_(0);
        srvmR.request.data.clear();
        srvmR.request.data+=strtf(srvmR.request.lenth, zero);
        sendCmdtoRound(clientmRound,srvmR);//电机恢复直行状态
        ros::Duration(2).sleep();          // 等待旋转到位
        while(!roundFlag)
        {
            ros::spinOnce();
            ros::Duration(0.02).sleep();
        }
        roundFlag = false;  //等待旋转到位后旋转到位标记置0；
        ROS_INFO("Park:Adjust_angle spinfun success.\n");
        msg.turnFlag=false;
        pub_turn.publish(msg);
    }

    void adjust_angle1()
    {
        if(stopFlag) {return;}

        communication::sendCmd automov;
        double thetatoturn=0;
        thetatoturn=error_theta1;

        char hold[23]={0x41,0x00,0x42,0x01,0x43,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                       0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x44,0x00}; //抱闸
        char zero[23]={0x41,0x00,0x42,0x00,0x43,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                       0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x44,0x00}; //回0度
        char turn[23]={0x41,0x00,0x42,0x00,0x43,0xFF,0xFF,0xEE,0x6C,0x00,0x00,0x11,0x94,
                       0x00,0x00,0x11,0x94,0xFF,0xFF,0xEE,0x6C,0x44,0x00}; //旋转45度

	/*//关闭左前右后电机使能
	//char state=0x0E;
	//宋凯
	//关闭左前电机使能
	char state=0x06;
	EnableMotor(state);
	ROS_INFO("Parking:Close motor");
	//等待电关闭
	while(motorState!=0x06)
	{   
	    ros::spinOnce();
            ros::Duration(0.02).sleep();
	}*/
	
        // 角度标记
        to90angle = false;
        to45angle = true;
        to0angle  = false;
        // 先调整轮子角度
        communication::sendCmd srvmR;
        srvmR.request.type=0x40;
        srvmR.request.lenth=23;
        srvmR.request.data.clear();
        srvmR.request.data+=strtf(srvmR.request.lenth, turn);
        sendCmdtoRound(clientmRound,srvmR);
        ros::Duration(2).sleep();//待定

        // 等待旋转到位
        while(!roundFlag)
        {
            ros::spinOnce();
            ros::Duration(0.02).sleep();
        }
        roundFlag = false;  //等待旋转到位后旋转到位标记置0；
        processor::isTurn msg;
        msg.turnFlag=true;
        pub_turn.publish(msg);
        ROS_INFO("Park:Now in adjust_angle1 turn!");


        while(fabs(thetatoturn)>0.01)   //先旋转到正确角度
        {
            ros::spinOnce();
            if(stopFlag)
            {
                move_(0);
                break;
            }

            int wfirst=0;

            //判断旋转翻方向
            if(thetatoturn>0){wfirst=(-1)*50;}
            if(thetatoturn<0){wfirst=50;}
            //待修改
            char ctrmov1[10]={0};
            ctrmov1[0]=0x41;//运动控制处理逻辑指令
            ctrmov1[1]=wfirst>>8;
            ctrmov1[2]=wfirst;
            ctrmov1[3]=0x42;
            ctrmov1[4]=0x00;
            ctrmov1[5]=0x00;
            ctrmov1[6]=0x43;
            ctrmov1[7]=0x00;
            ctrmov1[8]=0x44;
            ctrmov1[9]=0x01;

            automov.request.type=0x40;
            automov.request.lenth=10;
            automov.request.data="";
            automov.request.data+=strtf(automov.request.lenth,ctrmov1);
            sendCmdtoMove(clientm, automov);

            thetatoturn=error_theta1;
            //cout<<"error_theta1 "<<error_theta1<<endl;
            ROS_INFO("Park:error_theta1 is %.3f",error_theta1);

        }

        move_(0);
		
	/*//打开左前右后电机使能
        state=0x0F;
	EnableMotor(state);
	ROS_INFO("Parking:open motor");
	//等待开启
	while(motorState!=0x0F)
	{   
	    ros::spinOnce();
            ros::Duration(0.02).sleep();
	}*/
	
        // 角度标记
        to90angle = false;
        to45angle = false;
        to0angle  = true;
        srvmR.request.data.clear();
        srvmR.request.data+=strtf(srvmR.request.lenth, zero);
        sendCmdtoRound(clientmRound,srvmR);//电机恢复直行状态
        ros::Duration(2).sleep();          //等待旋转到位
        while(!roundFlag)
        {
            ros::spinOnce();
            ros::Duration(0.02).sleep();
        }
        roundFlag = false;  //等待旋转到位后旋转到位标记置0；
        ROS_INFO("Park:adjust_angle1 spinfun success.");
        msg.turnFlag=false;
        pub_turn.publish(msg);
    }

    void adjust_position(double goal,int vt)
    {
        if(stopFlag) {return;}

        communication::sendCmd automov;
        char hold[23]={0x41,0x00,0x42,0x01,0x43,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                       0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x44,0x00};//抱闸
        char zero[23]={0x41,0x00,0x42,0x00,0x43,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                       0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x44,0x00}; //回0度
        char angle_90[23]={0x41,0x00,0x42,0x00,0x43,0x00,0x00,0x23,0x28,0x00,0x00,0x23,0x28,
                           0x00,0x00,0x23,0x28,0x00,0x00,0x23,0x28,0x44,0x00}; //旋转90度
        int v=0;
        // 角度标记
        to90angle = true;
        to45angle = false;
        to0angle  = false;
        // 先调整轮子角度
        communication::sendCmd srvmR;
        srvmR.request.type=0x40;
        srvmR.request.lenth=23;
        srvmR.request.data.clear();
        srvmR.request.data+=strtf(srvmR.request.lenth, angle_90);
        sendCmdtoRound(clientmRound,srvmR);
        ros::Duration(2).sleep();//待定
        // 等待旋转到位
        while(!roundFlag)
        {
            ros::spinOnce();
            ros::Duration(0.02).sleep();
        }
        roundFlag = false;  //等待旋转到位后旋转到位标记置0；
        processor::isTurn msg;
        msg.turnFlag=true;
        pub_turn.publish(msg);

        if(goal>distance_r)
            v=vt;   //偏右
        else
            v=-vt;  //偏左
        bool flag=true;
        while(flag&&!stopFlag)   //调整位置
        {
            ros::spinOnce();
            if(v>0)
            {
                if(goal<=distance_r)
                    flag=false;
            }
            else
            {
                if(goal>distance_r)
                    flag=false;
            }
            move_(v);
            //cout<<"goal "<<goal<<" real "<<distance_r<<endl;
            ROS_INFO("Park:goal=%.3f, real distance= %.3f",goal,distance_r);

        }
        // 角度标记
        to90angle = false;
        to45angle = false;
        to0angle  = true;
        move_(0);
        srvmR.request.data.clear();
        srvmR.request.data+=strtf(srvmR.request.lenth, zero);
        sendCmdtoRound(clientmRound,srvmR);//电机恢复直行状态
        // 等待旋转到位
        ros::Duration(2).sleep();//待定
        while(!roundFlag)
        {
            ros::spinOnce();
            ros::Duration(0.02).sleep();
        }
        roundFlag = false;  //等待旋转到位后旋转到位标记置0；
        srvmR.request.data.clear();
        srvmR.request.data+=strtf(srvmR.request.lenth, hold);
        sendCmdtoRound(clientmRound,srvmR);//电机抱闸

        ROS_INFO("Park:Adjust X position success!");
        //cout<<"goal "<<goal<<"real "<<distance_r<<endl;
        msg.turnFlag=false;
        pub_turn.publish(msg);

    }

    void door(char opt) //对门进行操作 并等待操作到位
    {
        if(stopFlag) //如果急停状态
            return ;

        communication::sendCmd srvr;
        char gate=opt;
        srvr.request.type=0x50;
        srvr.request.lenth=1;
        srvr.request.data="";
        srvr.request.data.push_back(gate);
        sendCmdtoRoom(clientr, srvr);

        double last = ros::Time::now().toSec();
        double duration = 0.0;

        if(opt==0x01) //开门操作
        {
            ROS_INFO("Park:Open Door!");
            do
            {
                double current = ros::Time::now().toSec();
                duration = current-last;

                if(duration>25)  //动作超过正常时间，再次发送指令
                {
                    sendCmdtoRoom(clientr, srvr);
                    last=current;
                    ROS_INFO("Park:Open Door again!");
                }
                ros::spinOnce();
                if(stopFlag){break;}
            }
            while(!gateOpenFlag);
            //cout<< "waiting for gate fully-open! "<< gateOpenFlag<<endl;
            ROS_INFO("Park:ROOM DOOR is full open!");
        }
        if(opt==0x02) //关门操作
        {
            //cout<<"close the door"<<endl;
            ROS_INFO("Park:Close Door!");
            do
            {
                double current = ros::Time::now().toSec();
                duration = current-last;

                if(duration>25)  //动作超过正常时间，再次发送指令
                {
                    sendCmdtoRoom(clientr, srvr);
                    last=current;
                    ROS_INFO("Park:Close Door again!");
                }
                ros::spinOnce();
                if(stopFlag){break;}
            }
            while(!gateCloseFlag);
            //cout<< "waiting for gate fully-close"<< gateOpenFlag<<endl;
            ROS_INFO("Park:ROOM DOOR is full close!");
        }
    }

    void enter_room1(float pointX, float pointY,int v)  //里程计运动到屋内
    {
        float dpointX = pointX;
        float dpointY = pointY;

        double distance = sqrt((x0-dpointX)*(x0-dpointX)+(y0-dpointY)*(y0-dpointY));//求到目标点的距离

        double min_dis=9999999.0;
        double x0_last=x0;
        double y0_last=y0;

        bool flag=true;
        ROS_INFO("Park:Start entering room!");
        while((distance>STOPD)&&flag)
        {
            ros::spinOnce();
            if(stopFlag || x0<dpointX)
            {
                break;
            }
            // 防止地图崩溃
            if(fabs(x0_last-x0)>2||fabs(y0_last-y0)>2)
            {
                ROS_INFO("Park:Robot location is wrong\n");
                stopFlag=true;
            }
            x0_last=x0;
            y0_last=y0;

            move_(v);

            distance = sqrt((x0-dpointX)*(x0-dpointX)+(y0-dpointY)*(y0-dpointY));
            if(distance<0.2)
            {
                if(distance<=min_dis)
                {
                    min_dis=distance;
                }
                else
                {
                    if(distance-min_dis> (STOPD)) //范围
                    {
                        flag=false;
                        ROS_INFO("Park:flag is false!");
                    }
                }
            }
            //cout<<distance<<endl;
            ROS_INFO("Park:Enter_room1: Distance to Target Point(%.3f,%.3f) is %.3f",dpointX,dpointY,distance);
        }
        move_(0);
        //cout<<"adjust 11"<<endl;
        ROS_INFO("Park:Finish enter_room1.");
    }
    void enter_room_fix(float pointX, float pointY,int v)  //justParkoutFlag=true的入库修正
    {
        float dpointX = pointX;
        float dpointY = pointY;

        double distance = sqrt((x0-dpointX)*(x0-dpointX)+(y0-dpointY)*(y0-dpointY));//求到目标点的距离

        double min_dis=9999999.0;
        double x0_last=x0;
        double y0_last=y0;

        bool flag=true;
        ROS_INFO("Park:Start entering room!");
        while((distance>STOPD)&&flag)
        {
            ros::spinOnce();
            if(stopFlag || x0>dpointX)
            {
                break;
            }
            // 防止地图崩溃
            if(fabs(x0_last-x0)>2||fabs(y0_last-y0)>2)
            {
                ROS_INFO("Park:Robot location is wrong\n");
                stopFlag=true;
            }
            x0_last=x0;
            y0_last=y0;

            move_(v);

            distance = sqrt((x0-dpointX)*(x0-dpointX)+(y0-dpointY)*(y0-dpointY));
            if(distance<0.2)
            {
                if(distance<=min_dis)
                {
                    min_dis=distance;
                }
                else
                {
                    if(distance-min_dis> (STOPD)) //范围
                    {
                        flag=false;
                        ROS_INFO("Park:flag is false!");
                    }
                }
            }
            //cout<<distance<<endl;
            ROS_INFO("Park:Enter_room_fix: Distance to Target Point(%.3f,%.3f) is %.3f",dpointX,dpointY,distance);
        }
        move_(0);
        //cout<<"adjust 11"<<endl;
        enterfixFlag=true;
        ROS_INFO("Park:Finish enter_room_fix.");
    }

    void enter_room(float pointX, float pointY,int v)  //里程计运动到屋内
    {
        communication::sendCmd automov;
        float dpointX = pointX;
        float dpointY = pointY;
        /*从enter_room移出
        double temp_dtheta=0.0;
        pkg_srvs::SrvGetLine getLineService;//两点求直线服务
        geometry_msgs::PoseWithCovariance poseA,poseB;
        poseA.pose.position.x = x0;
        poseA.pose.position.y = y0;
        poseB.pose.position.x = dpointX;
        poseB.pose.position.y = dpointY;
        getLineService.request.poseA = poseB;
        getLineService.request.poseB = poseA;

        if(client1.call(getLineService))
        {
            temp_dtheta = getLineService.response.line[3];  //求得直线角度
        }

        spinfun(temp_dtheta);
        ros::Duration(1).sleep();
        ROS_INFO("Park:spin to chargepose!");
        */
        double distance = sqrt((x0-dpointX)*(x0-dpointX)+(y0-dpointY)*(y0-dpointY));//求到目标点的距离

        double min_dis=9999999.0;
        double x0_last=x0;
        double y0_last=y0;

        bool flag=true;
        ROS_INFO("Park:Start Back into ROOM!");
        while((distance>STOPD)&&flag)
        {
            ros::spinOnce();
            if(stopFlag || y0>dpointY)
            {
                break;
            }
            // 防止地图崩溃
            if(fabs(x0_last-x0)>2||fabs(y0_last-y0)>2)
            {
                ROS_INFO("Park:the location is wrong!");
                stopFlag=true;
            }
            x0_last=x0;
            y0_last=y0;
            if(enterfixFlag==false)
            {
                if(fabs(x0-dpointX)>0.2)//当x0与期望位置偏差超过20cm
                {
                    ROS_INFO("Park:(x0:%.3f-dpointx:%.3f=Bias:%.3f) > 0.2, so stopFlag = true.", x0, dpointX,x0-dpointX);
                    stopFlag=true;
                }
            }
            else
            {
                if(fabs(x0-dpointX-0.1)>0.2)//刚出库,就入库,需要减去补偿量0.1m再做比较；
                {
                    ROS_INFO("Park:After enter_room_fix;(x0:%.3f-dpointx:%.3f=Bias:%.3f) > 0.2, so stopFlag = true.", x0, dpointX,x0-dpointX);
                    stopFlag=true;
                }
            }

            move_(v);

            distance = sqrt((x0-dpointX)*(x0-dpointX)+(y0-dpointY)*(y0-dpointY));
            if(distance<0.2)
            {
                if(distance<=min_dis)
                {
                    min_dis=distance;
                }
                else
                {
                    if(distance-min_dis> (STOPD)) //范围
                    {
                        flag=false;
                        ROS_INFO("Park:flag 2 is false!");
                    }
                }
            }
        }
        if(!stopFlag)
        {
            move_(0);
            ROS_INFO("Park:Robot has get in ROOM!");

            //屋内 关闭激光定位
            pkg_srvs::SrvMode srvn;
            srvn.request.cmd="LASER_OFF";
            sendCmdtoLaser(clientn,srvn);
            ROS_INFO("Park:Close Laser!");
            ros::Duration(5).sleep();
            srvn.request.cmd="OPEN_LASER";
            sendCmdtoLaser(clientn,srvn);
            ROS_INFO("Park:Open Laser!");
        }
        else
        {
            ROS_INFO("stopFlag is true! so robot move to forward 10s with speed 0.2m/s");
            move_(200);
            ros::Duration(10).sleep();
        }
    }

    void backforward(double time ,int vt )//后退
    {
        int v=vt;
        double last = ros::Time::now().toSec();
        double duration = 0.0;
        while(!stopFlag)//888888
        {
            if( (connectFlag&&distance_m>0.88) || distance_m>0.89 )
            {
                ROS_INFO("Park:connectFlag:%d!",connectFlag);
                break;
            }
            ros::spinOnce();
            double current = ros::Time::now().toSec();
            duration = current-last;
            move_(-v);
            //ROS_INFO("Park:connectFlag:%d!",connectFlag);
        }
        move_(0);
        //cout<<"connectFlag "<<connectFlag<<endl;
        //cout<<"duration "<<duration<<" distance "<<distance_m<<endl;

        ROS_INFO("Park:connectFlag:%d, duration is %.3f, distance is %.3f!",connectFlag,duration,distance_m);
        ROS_INFO("Park:robot has back into charging pile!");
    }

    void device(char dev,char opt,ros::ServiceClient client,char type=0x00)
    {
        communication::sendCmd srvp;//发送给plc
        if(type)
            srvp.request.type=type;
        else
            srvp.request.type=0x30;

        srvp.request.lenth=2;
        srvp.request.data="";
        srvp.request.data.push_back(dev);
        srvp.request.data.push_back(opt);
        sendCmdtoPLC(clientp, srvp);
    }

    void powerOFF()
    {
        // 关闭云台
        device(0x33,OFF,clientp);
        // 关闭扬声器
        device(0x31,OFF,clientp);
        // 关闭激光
        device(0x32,OFF,clientp);
        // 关闭驱动器
        device(0x35,OFF,clientp);
        device(0x36,OFF,clientp);
        device(0x38,OFF,clientp);
        device(0x3B,OFF,clientp);
        //关闭温控系统
        device(0x56,OFF,clientp,0x50);
        //关闭其他设备
        device(0x34,OFF,clientp);
        ROS_INFO("Park:System power off, include Yuntai, Speaker, Laser, Driver, Wenkong, Other device!");
    }

    void powerON()
    {
        //开启其他设备
        device(0x34,ON,clientp);
        //开启温控系统
        device(0x56,ON,clientp,0x50);
        // 开启驱动器
        device(0x35,ON,clientp);
        device(0x36,ON,clientp);
        device(0x38,ON,clientp);
        device(0x3B,ON,clientp);
        // 开启激光
        device(0x32,ON,clientp);
        // 开启扬声器
        device(0x31,ON,clientp);
        // 开启云台
        device(0x33,ON,clientp);
        ROS_INFO("Park:System power on, include Other device, Wenkong, Driver, Laser, Speaker, Yuntai!");
    }

    bool get_in()
    {
        enterfixFlag=false;
        if(justParkoutFlag==false)
        {
            enter_room1(-0.10,0,-20);//如果机器人不是刚刚出库，则需要x方向后退0.1m才可入库。
        }
        else
        {
            enter_room_fix(0.10,0,20);//如果机器人不是刚刚出库，则需要x方向前进0.1m才可入库。
        }
        //给充电屋发开门指令
        char open_door=0x01;
        door(open_door);
        spinfun(-3.1416/2);
        //0.关闭激光定位 里程计运动到屋内
        // 关闭防跌落
        communication::sendCmd srvm;//发送给运控板
        srvm.request.type=0x60;
        srvm.request.lenth=2;
        srvm.request.data=0x01;
        sendCmdtoMove(clientm, srvm);
        //入库先显示状态未达
        arvstate="";
        arvstate+=0x01;
        pubstate();
        ros::Duration(1).sleep();
        enter_room(my_room.room_in.x,my_room.room_in.y,-100);//8888888
        //1.关闭库门
        if(stopFlag)//如果急停,说明不正常
        {
            move_(0);
            return false;
        }
        else
        {
            char close=0x02;
            door(close);

            adjust_angle();//2.调整姿态为正姿态 调整与门的距离
            adjust_ydistance(room_point.y,-50);//3.调整位置
            adjust_position(room_point.x,10);//3.调整位置
            adjust_angle();//3.调整姿态
            backforward(100,5);//4.后退定时间或光电信号
            if(stopFlag)
            {
                move_(0);
                return false;;
            }
            else
                return true;
        }
    }

    void out_room(float pointX, float pointY,int v)  //里程计运动到屋内
    {

        //communication::sendCmd automov;
        float dpointX = pointX;
        float dpointY = pointY;

        double distance = sqrt((x0-dpointX)*(x0-dpointX)+(y0-dpointY)*(y0-dpointY));//求到目标点的距离

        //double vslowrate=1.0;
        double min_dis=9999999.0;
        double x0_last=x0;
        double y0_last=y0;

        bool flag=true;
        ROS_INFO("Park:out_room:Start out!");
        while((distance>STOPD)&&flag)
        {
            ros::spinOnce();
            if(stopFlag || x0>dpointX)
            {
                break;
            }
            // 防止地图崩溃
            if(fabs(x0_last-x0)>2||fabs(y0_last-y0)>2)
            {
                ROS_INFO("Park:out_room:the location is wrong\n");
                stopFlag=true;
            }
            x0_last=x0;
            y0_last=y0;

            move_(v);
            distance = sqrt((x0-dpointX)*(x0-dpointX)+(y0-dpointY)*(y0-dpointY));

            if(distance<0.2)
            {
                if(distance<=min_dis)
                {
                    min_dis=distance;
                }
                else
                {
                    if(distance-min_dis>STOPD) //范围
                    {
                        flag=false;
                        ROS_INFO("Park: flag 3 is flase\n");
                    }
                }
            }
        }
        move_(0);
        ROS_INFO("Park:out_room:Robot has moved out from ROOM!");
    }

    void adjust_hDistance(double distance_max,int v ) //调整水平距离horizon
    {
        if(stopFlag) {return;}
        //转为直行模式
        communication::sendCmd automov;
        char hold[23]={0x41,0x00,0x42,0x01,0x43,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                       0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x44,0x00};//抱闸

        char zero[23]={0x41,0x00,0x42,0x00,0x43,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                       0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x44,0x00}; //回0度

        char angle_90[23]={0x41,0x00,0x42,0x00,0x43,0x00,0x00,0x23,0x28,0x00,0x00,0x23,0x28,
                           0x00,0x00,0x23,0x28,0x00,0x00,0x23,0x28,0x44,0x00}; //旋转90度

        // 角度标记
        to90angle = true;
        to45angle = false;
        to0angle  = false;
        // 先调整轮子角度
        communication::sendCmd srvmR;
        srvmR.request.type=0x40;
        srvmR.request.lenth=23;
        srvmR.request.data.clear();
        srvmR.request.data+=strtf(srvmR.request.lenth, angle_90);
        sendCmdtoRound(clientmRound,srvmR);//电机旋转90度
        ros::Duration(2).sleep();          //等待旋转到位
        // 等待旋转到位
        while(!roundFlag)
        {
            ros::spinOnce();
            ros::Duration(0.02).sleep();
        }
        roundFlag = false;  //等待旋转到位后旋转到位标记置0；
        processor::isTurn msg;
        msg.turnFlag=true;
        pub_turn.publish(msg);

        double valid_range=0.1;
        bool isMove=false;

        //cout<<"distance_real "<<distance_real<<" distance_max "<<distance_max<<endl;
        ROS_INFO("Park:adjust_hDistance");
        /*距离大于最大值，需要向左移动*/
        if(distance_real-distance_max>=1)
        {
            ROS_INFO("Park:move left");
            while(1)
            {
                ros::spinOnce();
                if(fabs(distance_real-distance_max)<0.08||stopFlag) //距离接近,就停下
                    break;
                move_(v);
                //cout<<"distance_real "<<distance_real<<" distance_max "<<distance_max<<endl;
                ROS_INFO("Park:move left distance_real: %.3f,distance_max: %.3f",distance_real,distance_max);
            }
            isMove=true;
        }
        /*距离接近 且未调整，需要向右移动*/
        else
        {
            ROS_INFO("Park:move right");
            while(!isMove)
            {
                ros::spinOnce();
                if( distance_real-distance_max>valid_range  || stopFlag)
                    break;
                move_(-v);
                //cout<<"distance_real "<<distance_real<<" distance_max "<<distance_max<<endl;
                ROS_INFO("Park:move right,distance_real: %.3f,distance_max: %.3f",distance_real,distance_max);
            }
        }
        move_(0);
        // 角度标记
        to90angle = false;
        to45angle = false;
        to0angle  = true;
        srvmR.request.data.clear();
        srvmR.request.data+=strtf(srvmR.request.lenth, zero);
        sendCmdtoRound(clientmRound,srvmR);//电机恢复直行状态
        ros::Duration(15).sleep();	   //等待旋转到位
        while(!roundFlag)
        {
            ros::spinOnce();
            ros::Duration(0.02).sleep();
        }
        roundFlag = false;  //等待旋转到位后旋转到位标记置0；

        srvmR.request.data.clear();
        srvmR.request.data+=strtf(srvmR.request.lenth, hold);
        sendCmdtoRound(clientmRound,srvmR);//电机抱闸

        ROS_INFO("Park:adjust position success!");
        msg.turnFlag=false;
        pub_turn.publish(msg);

    }

    bool get_out()
    {
        // 角度标记
        char zero[23]={0x41,0x00,0x42,0x00,0x43,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                       0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x44,0x00}; //回0度
        // 角度标记
        to90angle = false;
        to45angle = false;
        to0angle  = true;
        // 先调整轮子角度
        communication::sendCmd srvmR;
        srvmR.request.type=0x40;
        srvmR.request.lenth=23;
        srvmR.request.data.clear();
        srvmR.request.data+=strtf(srvmR.request.lenth, zero);
        sendCmdtoRound(clientmRound,srvmR);
        ros::Duration(2).sleep();//待定
        // 等待旋转到位
        while(!roundFlag)
        {
            ros::spinOnce();
            ros::Duration(0.02).sleep();
        }
        roundFlag = false;  //等待旋转到位后旋转到位标记置0；

        ROS_INFO("Park:Robot start to leave the Charging pile!");
        out_adjust_ydistance_inroom(room_point.y,50);//3.调整位置

        if(inroomFlag==false)
            return false;

        if(fabs(error_theta)>0.03)
        {
            ROS_INFO("Park:fabs(error_theta)>0.03, Begin to adjust angle to make it correct.");
            adjust_angle();//2.调整姿态为正对门的姿态
        }

        //开门
        char open_door=0x01;
        door(open_door);

        //出库运动
        ROS_INFO("Park:Robot start to leave the ROOM!");
        double targetth;
        //0.定时运动到原点附近
        out_room(my_room.room_out.x,my_room.room_out.y,150);
        //1.关闭库门
        char close=0x02;
        door(close);
        //2.原点掉头
        targetth=3.1416;
        spinfun(targetth);
        //3.调整姿态
        adjust_angle1();
        out_adjust_ydistance(DISTANCE_OUT_ROOM,-50);
        adjust_hDistance(DISTANCE_OUT, 10);
        adjust_angle1();
        if(stopFlag)//如果急停 说明不正常
        {
            move_(0);
            return false;
        }
        else
            return true;
    }

    void callback3(const pkg_back::state::ConstPtr& msg)
    {
        fullFlag=msg->fullFlag;
        positive_curr=msg->positive_curr;
    }

    bool uptoTime(struct tm* p)
    {
        struct tm t;
        t.tm_hour = 23;
        t.tm_min = 30;
        t.tm_sec = 0;
        unsigned int tickin = t.tm_hour * 3600 + t.tm_min * 60 + t.tm_sec;
        unsigned int ticksys = p->tm_hour * 3600 + p->tm_min * 60 + p->tm_sec;
        if (tickin > ticksys)
            return false;
        else
            return true;
    }
    void parkingcallback(const processor::parkingOrder::ConstPtr& msg_park)
    {
        stopFlag = msg_park->stopFlag;
        string parkingOrder="";
        parkingOrder+= msg_park->parkingOrder;

        if(!stopFlag)
        {
            if(parkingOrder[0]==PARKIN && (fabs(x0)<0.5 && fabs(y0)< 0.5))  //入库
            {
                pkg_srvs::SrvMode srvn;
                ROS_INFO("Park:Receive PARKIN order!");

                if(get_in())//运动到充电桩12*/
                {
                    //入库成功 关量测 关激光
                    srvn.request.cmd="ODOM_OFF";
                    sendCmdtoLaser(clientn,srvn);
                    ROS_INFO("Park:Close Odometry!");

                    srvn.request.cmd="CLOSE_LASER";
                    sendCmdtoLaser(clientn,srvn);
                    ROS_INFO("Park:Close Laser!");

                    //关闭设备
                    powerOFF();
                    //发布充电状态 处于充电状态 并且处于充电屋
                    move::charge msg;
                    msg.chargeFlag=true; msg.isInRoom=true;
                    pub_chargeState.publish(msg);
                    ros::Duration(1).sleep();
                    //发送充电屋开始充电
                    //循环充电
                    bool endCharge=false,needFull=false;
                    bool hasFulled=false; //是否充满过

                    ofstream fos("/home/user/log/record/charge.log");
                    time_t t1;

                    //循环充电
                    while(!stopFlag)
                    {
                        time(&t1);
                        fos<<ctime(&t1)<<"当前电量 "<<dianliang<<" 充电继电器 "<<chargeFlag<<" 需充满: "<<needFull<<" 已充满: "<<hasFulled<<"电流:"<<positive_curr<<endl;

                        if(endCharge &&chargeFlag)  //满足停止条件处于充电状态则关闭充电
                        {
                            communication::sendCmd srvr;
                            char charge=0x02;
                            srvr.request.type=0x60;
                            srvr.request.lenth=1;
                            srvr.request.data="";
                            srvr.request.data.push_back(charge);
                            sendCmdtoRoom(clientr, srvr);
                            fos<<"发送关闭充电器指令"<<endl;
                            time(&t1);
                            fos<<ctime(&t1)<<"完成充电 "<<dianliang<<" 充电继电器 "<<chargeFlag<<" 需充满: "<<needFull<<" 已充满: "<<hasFulled<<" 电流:"<<positive_curr<<endl;
                        }
                        else if(((dianliang<93)&& positive_curr) || (positive_curr&&needFull) ) //电量小于%90并且不在充电状态则开始充电
                        {
                            //先关闭再打开
                            communication::sendCmd srvr;
                            char charge=0x02;
                            srvr.request.type=0x60;
                            srvr.request.lenth=1;
                            srvr.request.data="";
                            srvr.request.data.push_back(charge);
                            sendCmdtoRoom(clientr, srvr);
                            fos<<"发送关闭充电器指令"<<endl;
                            ros::Duration(2).sleep();

                            charge=0x01;
                            srvr.request.data="";
                            srvr.request.data.push_back(charge);
                            sendCmdtoRoom(clientr, srvr);
                            sendCmdtoRoom(clientr, srvr);
                            fos<<"发送打开充电器指令"<<endl;
                            ros::Duration(30).sleep();
                            time(&t1);
                            fos<<ctime(&t1)<<"开始充电 "<<dianliang<<" 充电继电器 "<<chargeFlag<<" 需充满: "<<needFull<<" 已充满: "<<hasFulled<<"电流:"<<positive_curr<<endl;
                        }

                        //确定是否需要充满  未充满过且未确定是否要充满 进行判断
                        if((!hasFulled) && (!needFull))
                        {
                            //check the time;
                            time_t timeCurrent;
                            time(&timeCurrent);//获取当前时间
                            struct tm* p = localtime(&timeCurrent); //取得当地时间
                            needFull=uptoTime(p);
                        }
                        // 确定停止充电条件
                        if(needFull)
                        {
                            endCharge=fullFlag;
                            if(fullFlag)
                            {
                                hasFulled=true;
                                needFull=false;
                            }
                        }
                        else
                            endCharge=(dianliang>=98);

                        ros::spinOnce();
                        ros::Duration(5).sleep();
                    }
                    time(&t1);
                    fos<<ctime(&t1)<<"over "<<endl;
                    fos.close();

                    gateOpenFlag=false;
                    gateCloseFlag=false;
                    chargeFlag=false;
                    connectFlag=false;
                }
            }
            else if(parkingOrder[0]==PARKOUT) //出库
            {
                //cout<<"receive parkout order!"<<endl;
                ROS_INFO("Park:Receive PARKOUT order!");
                system("sh /home/user/.nav/log.sh");

                //if(1)
                //{
                ROS_INFO("Initial Postion: x0=%.3f,y0=%.3f",x0,y0);
                //cout<<x0<<y0<<endl;
                //发送充电屋结束充电
                communication::sendCmd srvr;
                char charge=0x02;
                srvr.request.type=0x60;
                srvr.request.lenth=1;
                srvr.request.data="";
                srvr.request.data.push_back(charge);
                sendCmdtoRoom(clientr, srvr);
                sendCmdtoRoom(clientr, srvr);

                //给PLC发送上电指令
                powerON();
                ROS_INFO("Park:Wait PLC Power On 20s.");
                ros::Duration(20).sleep();

                // 关闭防跌落
                communication::sendCmd srvm;//发送给运控板
                srvm.request.type=0x60;
                srvm.request.lenth=2;
                srvm.request.data=0x01;
                sendCmdtoMove(clientm, srvm);

                //发布放电状态
                move::charge msg;
                msg.chargeFlag=false;
                msg.isInRoom=true;
                pub_chargeState.publish(msg);
                pub_chargeState.publish(msg);
                ROS_INFO("Park:Publish chargeFlag=false state.");
                //等待云台自检结束
                ROS_INFO("Park:Wait Yuntai Power On 70s.");
                ros::Duration(70).sleep();

                //出库关量测
                pkg_srvs::SrvMode srvn;
                srvn.request.cmd="ODOM_OFF";
                ROS_INFO("Park:Close Odometry");
                sendCmdtoLaser(clientn,srvn);
                ros::Duration(10).sleep();

                //出库开量测
                srvn.request.cmd="ODOM_ON";
                ROS_INFO("Park:Open Odometry");
                sendCmdtoLaser(clientn,srvn);

                //开激光
                srvn.request.cmd="OPEN_LASER";
                sendCmdtoLaser(clientn,srvn);
                ROS_INFO("Park:Open Laser");
                ros::Duration(10).sleep();

                if(get_out())//如果出库成功
                {  /*
                        //断开与充电屋连接
                        processor::parkingOrder parkOrder;
                        parkOrder.zigbeeComFlag=false;
                        parkOrder.stopFlag=true;
                        pub1.publish(parkOrder);   */

                    // 打开防跌落
                    srvm.request.type=0x60;
                    srvm.request.lenth=2;
                    srvm.request.data=0x02;
                    sendCmdtoMove(clientm, srvm);

                    ofstream fos("/home/user/.nav/initPose.txt");
                    double x=0.0,y=0.0,th=1.57;
                    fos << x << endl;
                    fos << y << endl;
                    fos << th << endl;
                    fos.close();

                    srvn.request.cmd="CLOSE_LASER";
                    ROS_INFO("Park:Close Laser");
                    sendCmdtoLaser(clientn,srvn);
                    //开激光
                    srvn.request.cmd="LASER_ON";
                    sendCmdtoLaser(clientn,srvn);
                    ROS_INFO("Park:Open Laser Location!");

                    //等待地图加载完成  时间小于一定时间或者坐标为初始点
                    double last = ros::Time::now().toSec();
                    double duration = 0.0;
                    while(1)
                    {
                        double current = ros::Time::now().toSec();
                        duration = current-last;

                        if(fabs(x0)<0.1 && fabs(y0)<0.1)
                        {
                            //cout<<"locate success"<<endl;
                            ROS_INFO("Park:Location successfully!");
                            break;
                        }
                        ros::Duration(1).sleep();
                        ros::spinOnce();
                    }
                    //  ros::Duration(150).sleep();
                    double targetth=0;
                    spinfun(targetth);
                    justParkoutFlag = true;
                    //不处于充电屋
                    move::charge msg;
                    msg.chargeFlag=false;
                    msg.isInRoom=false;
                    pub_chargeState.publish(msg);
                    pub_chargeState.publish(msg);
                    //出库到达显示状态
                    arvstate="";
                    arvstate+=0x02;
                    pubstate();
                    ROS_INFO("Park:PARKOUT finish!");
                    ros::Duration(1).sleep();
                }
                //}
                //else
                //    cout<<"cannot out"<<endl;
            }
        }
    }

private:
    ros::NodeHandle n;

    ros::Publisher pub1;
    ros::Publisher pub_turn;
    ros::Publisher pub_state;
    ros::Publisher pub_chargeState;
    ros::Subscriber sub1;
    ros::Subscriber sub2;
    ros::Subscriber sub3;
    ros::Subscriber sub4;
    ros::Subscriber sub5;
    ros::Subscriber sub6;
    ros::Subscriber sub7;
    ros::Subscriber sub8;

    ros::ServiceClient client1;
    ros::ServiceClient clientm;
    ros::ServiceClient clientp;
    ros::ServiceClient clientr;
    ros::ServiceClient clientmRound;
    ros::ServiceClient clientn;
};

int main(int argc, char**argv)
{
    ros::init(argc, argv, "parking");

    Parking SAObject;

    ros::MultiThreadedSpinner s(10);
    ros::spin(s);

    return 0;
}
