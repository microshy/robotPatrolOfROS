#include <ros/ros.h>
#include <sstream>
#include <cmath>
#include "tf/transform_broadcaster.h"
#include "communication/sendCmd.h"
#include "communication/state.h"
#include "processor/moveorder.h"                      //主控制器提供的消息
#include "stateandalarm/state.h"                      //发送给状态模块的消息
#include "geometry_msgs/PoseWithCovarianceStamped.h"  //导航模块的位姿消息
#include "geometry_msgs/PoseWithCovariance.h"         //服务里的数据类型
#include "pkg_srvs/SrvGetLine.h"                      //两点求直线服务
#include "pkg_srvs/SrvGetYawBias.h"                   //求偏行角服务
#include "pkg_msgs/MsgPointInfo.h"                    //关键点发送给自主返航模块
#include "processor/parkingOrder.h"                   //发送给出入库充电模块
#include "pkg_srvs/SrvMode.h"
#include "processor/isTurn.h"                        //是否旋转
#include "move/charge.h"                             //充放电信息 放电为false
#include "pkg_srvs/SrvReturn.h"

#include <string.h>
#include <iostream>
#include <fstream>

using namespace std;

#define V 400.0      //定义机器人运行速度 mm/s
#define W 150.0      //定义机器人运行角速度 0.001rad/s
#define T 0.02       //定义每次发送速度指令的时间周期
#define movecount 1   
#define a 0.5
#define b 0.5 
#define Kx 1
#define Ky 5
#define K 1
#define STOPTH 0.02    //旋转允许误差角度（开始停止角度）  单位 rad
#define SLOWTH 0.40
#define STOPD  0.05    //停车距离    单位 m
#define SLOWD1 1.00    //第一次减速距离   单位 m
#define SLOWDTEST 0.50 //开始加速点距离 单位m

static unsigned int call_count=0;

class MoveControl
{
public:
    string arvstate; //说明：此数据发送给状态备份模块,为是否到达目标点;
    string mapState; //说明：此数据发送给状态备份模块,是否地图正常;
    unsigned char locationX[4]={0x00,0x00,0x00,0x00};
    unsigned char locationY[4]={0,0,0,0};
    unsigned char pose[2]={0,0};
    int  pointL = 0;
    double dspeed;
    double gain;
    double x0; //表示机器人实时x坐标
    double y0; //表示机器人实时y坐标
    double th0;//表示机器人实时姿态角

	double dlf;//左前编码器变量
	double dlb;//左后编码器变量
	double drf;//右前编码器变量
	double drb;//右后编码器变量
	double lfAlertLastTime;//左前电机速度为0且电流过高的时间
	double lbAlertLastTime;//左后电机速度为0且电流过高的时间
	double rfAlertLastTime;//右前电机速度为0且电流过高的时间
	double rbAlertLastTime;//右后电机速度为0且电流过高的时间
	int lfAlertNum = 0;//左前电机速度为0且电流过高的次数
	int lbAlertNum = 0;//左后电机速度为0且电流过高的次数
	int rfAlertNum = 0;//右前电机速度为0且电流过高的次数
	int rbAlertNum = 0;//右后电机速度为0且电流过高的次数
	bool lfFirstAlertFlag = false; //左前电机第一次速度为0且电流过高标志位
	bool lbFirstAlertFlag = false; //左后电机第一次速度为0且电流过高标志位
	bool rfFirstAlertFlag = false; //右前电机第一次速度为0且电流过高标志位
	bool rbFirstAlertFlag = false; //右后电机第一次速度为0且电流过高标志位

    int lfzxcurrent=0;//左前电机电流
    int lbzxcurrent=0;//左后电机电流
    int rfzxcurrent=0;//右前电机电流
    int rbzxcurrent=0;//右后电机电流

	const double leftfFix = 0.9784;
	const double leftbFix = 0.9805;
	const double rightfFix = 0.9889;
	const double rightbFix = 0.9775;

    bool stopFlag;  //停车状态标志位
    bool driveFlag; //驱动器使能标志位
    bool blockFlag; //障碍物标志位
    bool roundFlag; //旋转到达标志位
    bool highCurrentFlag = false; //过流标志
    bool to45angle=false; //转到45度标记
    bool to0angle=false;  //转到0度标记
    char motorState;

    MoveControl()
    {
        //Topic to publish
        pub_state  = n.advertise<stateandalarm::state>("state_route", 1000);        //发送给状态模块
        pub_point  = n.advertise<pkg_msgs::MsgPointInfo>("topic_point_info", 1000); //发送给自助返航模块
        pub_turn   = n.advertise<processor::isTurn>("processor/turn", 1000);        //发送导航模块，是否处于旋转状态
        pub_charge = n.advertise<processor::parkingOrder>("parkingOrder", 1000);    //发送给出入库充电模块

        //Topic to sub
        sub1 = n.subscribe("moveorder",5,&MoveControl::moveordercallback,this);              //接收主控板消息
        sub2 = n.subscribe("topic_robot_pose",2,&MoveControl::posecallback,this);            //接收导航模块实时位姿
        sub3 = n.subscribe("communication/state_move",1, &MoveControl::callback1,this);      //接受机器人运控板使能信息
        sub4 = n.subscribe("communication/state_moveRound",10, &MoveControl::callback2,this);//接受机器人旋转运控板使能信息
		ros::Subscriber odometrySensorSubscriber = n.subscribe("topic_odometry_sensor", 1000, &MoveControl::updateData,this);	//订阅量测传感器信息
        //service to be used
        client1 = n.serviceClient<pkg_srvs::SrvGetLine>("srv_get_line");//求直线及角度
        clientm = n.serviceClient<communication::sendCmd>("send_move_cmd"); //给运动控制板发送消息的服务
        clientmRound = n.serviceClient<communication::sendCmd>("send_moveRound_cmd"); //给旋转运动控制板发送消息的服务
        clientn = n.serviceClient<pkg_srvs::SrvMode>("srv_mode");//调用导航模块服务
        clientback = n.serviceClient<pkg_srvs::SrvReturn>("srv_return"); //调用自主返航服务

        dspeed =V;
        gain=0.8;
        stopFlag=false; //停车状态标志位
        driveFlag=true; //驱动器使能标志位
        blockFlag=false;//障碍物标志位
        roundFlag=false;//旋转到达标志位
    }

	void updateData(const pkg_msgs::MsgOdometrySensor::ConstPtr& msg)
	{
		dlf = msg->dlf;
		dlb = msg->dlb;
		drf = msg->drf;
		drb = msg->drb;

		if (fabs(dlf)>0.1 || fabs(dlb)>0.1 || fabs(drf)>0.1 || fabs(drb)>0.1)
		{
			if (dlf>0) dlf = 0.02;	 else dlf = -0.02;
			if (dlb>0) dlf = 0.02;	 else dlf = -0.02;
			if (drf>0) dlf = 0.02;	 else dlf = -0.02;
			if (drb>0) dlf = 0.02;	 else dlf = -0.02;
		}

		dlf = dlf * leftfFix;
		dlb = dlb * leftbFix;
		drf = drf * rightfFix;
		drb = drb * rightbFix;
	}

	void odomInvalidProtect()
	{
		//若某电机连续5分钟保持0速度且高电流则关掉该电机驱动
		//左前
		if (dlf <= 0.00005 && (abs(lfzxcurrent) > safeCurrent))
		{
			if (!lfFirstAlertFlag)
			{
				lfFirstAlertFlag = true;
				lfAlertLastTime = ros::Time::now().toSec();
				lfAlertNum = 0;
			}
			//时间帧连续则计数
			if (ros::Time::now().toSec() - lfAlertLastTime <= 0.2)
			{
				lfAlertNum++;
				//连续5分钟触发则断掉驱动
				if (lfAlertNum >= 3000)
				{
					lfFirstAlertFlag = false;
					char state = 0x00;
					EnableMotor(state);
					int retry_num = 0;
					while (motorState != 0x00)
					{
						ros::spinOnce();
						ros::Duration(0.1).sleep();
						retry_num++;
						if (retry_num == 3)
						{
							retry_num = 0;
							EnableMotor(state);
							ROS_INFO("Move:Close motor again");
						}
					}
				}
			}
			//不连续则重新开始
			else
			{
				lfFirstAlertFlag = false;
			}
		}
		//左后
		if (dlb <= 0.00005 && (abs(lbzxcurrent) > safeCurrent))
		{
			if (!lbFirstAlertFlag)
			{
				lbFirstAlertFlag = true;
				lbAlertLastTime = ros::Time::now().toSec();
				lbAlertNum = 0;
			}
			//时间帧连续则计数
			if (ros::Time::now().toSec() - lbAlertLastTime <= 0.2)
			{
				lbAlertNum++;
				//连续5分钟触发则断掉驱动
				if (lbAlertNum >= 3000)
				{
					lbFirstAlertFlag = false;
					char state = 0x00;
					EnableMotor(state);
					int retry_num = 0;
					while (motorState != 0x00)
					{
						ros::spinOnce();
						ros::Duration(0.1).sleep();
						retry_num++;
						if (retry_num == 3)
						{
							retry_num = 0;
							EnableMotor(state);
							ROS_INFO("Move:Close motor again");
						}
					}
				}
			}
			//不连续则重新开始
			else
			{
				lbFirstAlertFlag = false;
			}
		}
		//右前
		if (drf <= 0.00005 && (abs(rfzxcurrent) > safeCurrent))
		{
			if (!rfFirstAlertFlag)
			{
				rfFirstAlertFlag = true;
				rfAlertLastTime = ros::Time::now().toSec();
				rfAlertNum = 0;
			}
			//时间帧连续则计数
			if (ros::Time::now().toSec() - rfAlertLastTime <= 0.2)
			{
				rfAlertNum++;
				//连续5分钟触发则断掉驱动
				if (rfAlertNum >= 3000)
				{
					rfFirstAlertFlag = false;
					char state = 0x00;
					EnableMotor(state);
					int retry_num = 0;
					while (motorState != 0x00)
					{
						ros::spinOnce();
						ros::Duration(0.1).sleep();
						retry_num++;
						if (retry_num == 3)
						{
							retry_num = 0;
							EnableMotor(state);
							ROS_INFO("Move:Close motor again");
						}
					}
				}
			}
			//不连续则重新开始
			else
			{
				rfFirstAlertFlag = false;
			}
		}
		//右后
		if (drb <= 0.00005 && (abs(rbzxcurrent) > safeCurrent))
		{
			if (!rbFirstAlertFlag)
			{
				rbFirstAlertFlag = true;
				rbAlertLastTime = ros::Time::now().toSec();
				rbAlertNum = 0;
			}
			//时间帧连续则计数
			if (ros::Time::now().toSec() - rbAlertLastTime <= 0.2)
			{
				rbAlertNum++;
				//连续5分钟触发则断掉驱动
				if (rbAlertNum >= 3000)
				{
					rbFirstAlertFlag = false;
					char state = 0x00;
					EnableMotor(state);
					int retry_num = 0;
					while (motorState != 0x00)
					{
						ros::spinOnce();
						ros::Duration(0.1).sleep();
						retry_num++;
						if (retry_num == 3)
						{
							retry_num = 0;
							EnableMotor(state);
							ROS_INFO("Move:Close motor again");
						}
					}
				}
			}
			//不连续则重新开始
			else
			{
				rbFirstAlertFlag = false;
			}
		}

	}
    void sendCmd(ros::ServiceClient client, communication::sendCmd srv)
    {
        int count = 0;
        while(count<5)
        {
            if(driveFlag)
            {
                if(client.call(srv)&&srv.response.receive)
                {
                    //ROS_INFO("Move: send cmd success!");
                    count = 5;
                }
                else
                {
                    ROS_INFO("Move: send cmd again! %d",count);
                    count++;
                }
            }
            else
            {
                ROS_INFO("Move: driver is unabled!\n");
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
                    ROS_INFO("Move: send cmd again! %d",count);
                    count++;
                }
            }
        }
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

    void callback1(const communication::state::ConstPtr& msg_move)
    {
        //更新机器人驱动器使能信息
        unsigned char movstate[100];
        memcpy(movstate,msg_move->data.c_str(),msg_move->data.size()+1);
        //sk
	if(movstate[9]==0x07 || movstate[9]==0x0E || movstate[9]==0x0F || highCurrentFlag)
        {
            driveFlag = true;
        }
        else
        {
            driveFlag = false;
        }
        motorState=movstate[9];
        //字节顺序可参见stateandalarm.cpp statecallback4函数
        lfzxcurrent = (short)((movstate[29]<<8)|movstate[30]);
        rfzxcurrent = (short)((movstate[31]<<8)|movstate[32]);
        lbzxcurrent = (short)((movstate[33]<<8)|movstate[34]);
        rbzxcurrent = (short)((movstate[35]<<8)|movstate[36]);
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
        if(to45angle)
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

    void back()
    {
        processor::parkingOrder  parkOrder;
        parkOrder.parkingOrder=""; //发布入库指令
        parkOrder.parkingOrder.push_back(0x01);
        parkOrder.zigbeeComFlag=true;
        parkOrder.stopFlag=false;
        pub_charge.publish(parkOrder);

        //清空路径栈
        pkg_srvs::SrvReturn backhome;
        backhome.request.clear = true;
        if(clientback.call(backhome))
        {
            ROS_INFO("Move:clear back route!");
        }
    }

    void moveordercallback(const processor::moveorder::ConstPtr& msg)//moveorder消息的回调函数
    {
        char data[1000]={0};
        memcpy(data,msg->location.c_str(),msg->location.size()+1);
        char strpose[1000]={0};
        memcpy(strpose,msg->pose.c_str(),msg->pose.size()+1);
        char pointlevel[1000]={0};
        memcpy(pointlevel,msg->pointlevel.c_str(),msg->pointlevel.size()+1);

        stopFlag = msg->stopflag;
        blockFlag = msg->block;
        int returnFlag=0;
        returnFlag=msg->returnflag; //是否为自动返航

        call_count++;
        //cout<<call_count<<" new task,stopFlag: "<<stopFlag<<endl;
        ROS_INFO("Move:receive %d moveorder msgs, stopFlag is %d",call_count,stopFlag);

        if(!msg->speed.empty())
        {
            dspeed = msg->speed[msg->speed.size()-1] * 10.0;        //设定速度
            switch ((int)dspeed/100)
            {
            case 4: gain=0.70; break;
            case 5: gain=0.75; break;
            case 6: gain=0.80; break;
            case 7: gain=0.83; break;
            case 8: gain=0.85; break;
            case 10:gain=0.88; break;
            default :
            {
                dspeed=V;
                gain=0.8;
                //printf("NO avlid speed\n");
                ROS_INFO("Move: NO VALID SPEED!So set speed to default value, V=600mm/s,gain=0.8.");
                break;
            }
            }
            dspeed = dspeed/1000.0;
        }
        else
        {
            dspeed = V;
            gain = 0.8;//默认速度0.6m/s
            dspeed = dspeed/1000.0;
        }

        if(msg->state == "keep")//自动状态
        {
            locationX[0]=0;//读出对应点的x坐标
            locationX[1]=0;
            locationX[2]=0;
            locationX[3]=0;
            locationY[0]=0;//读出对应点的y坐标
            locationY[1]=0;
            locationY[2]=0;
            locationY[3]=0;

            pose[0]=0;
            pose[1]=0;
            pointL=0;

            arvstate="";
            if(!msg->routeid.empty())
            {
                arvstate+=0x01;
                arvstate+=msg->routeid;
            }
            else
            {
                arvstate.push_back(0x00);
                arvstate+=0xFF;
            }
            pubstate();

            int pn=0;
            pn=strlen(msg->pointtype.c_str());
            ROS_INFO("Move:Current Task include %d Points.",pn);
            int i = 0;
            do
            {
                ros::spinOnce();
                if(stopFlag)
                {
                    break;
                }

                locationX[0]=data[8*i+0];//读出对应点的x坐标
                locationX[1]=data[8*i+1];
                locationX[2]=data[8*i+2];
                locationX[3]=data[8*i+3];
                locationY[0]=data[8*i+4];//读出对应点的y坐标
                locationY[1]=data[8*i+5];
                locationY[2]=data[8*i+6];
                locationY[3]=data[8*i+7];

                pose[0]=strpose[2*i+0];
                pose[1]=strpose[2*i+1];

                pointL=(int)pointlevel[i];

                pkg_msgs::MsgPointInfo pointInfo;

                pointInfo.location="";
                pointInfo.location+=locationX[0];
                pointInfo.location+=locationX[1];
                pointInfo.location+=locationX[2];
                pointInfo.location+=locationX[3];
                pointInfo.location+=locationY[0];
                pointInfo.location+=locationY[1];
                pointInfo.location+=locationY[2];
                pointInfo.location+=locationY[3];

                pointInfo.pointlevel=pointL;
                pub_point.publish(pointInfo);

                //将目标点坐标转换成double型 单位m
                int ddpointX = 0;
                int ddpointY = 0;//目标点

                ddpointX =(((((((ddpointX | locationX[0]) << 8) | locationX[1]) << 8 ) |locationX[2]) << 8) | locationX[3]);
                ddpointY =(((((((ddpointY | locationY[0]) << 8) | locationY[1]) << 8 ) |locationY[2]) << 8) | locationY[3]);

                //printf("[target goal](%d,%d) \n",ddpointX,ddpointY);
                ROS_INFO("Move:[target goal](%d,%d)",ddpointX,ddpointY);
                float dpointX = ddpointX/100.0;
                float dpointY = ddpointY/100.0;
                double distance = sqrt((x0-dpointX)*(x0-dpointX)+(y0-dpointY)*(y0-dpointY));//求到目标点的距离

                if(distance < 0.1)
                {
                    //printf("too close\n");
                    ROS_INFO("Move:very close to target!");
                }
                else
                {
                    ROS_INFO("Move:Call controller!");
                    MoveControl::controller(locationX,locationY);//调用轨迹跟踪
                }

                i++ ;
            }while(i<pn);

            // 到达目标位置
            if(!stopFlag && msg->state == "keep")
            {
                int dpose=0;
                dpose=(short)(((dpose|pose[0])<<8)|pose[1]);
                ROS_INFO("Move:dpose is %d",dpose);
                //printf("dpose is %d",dpose);
                double ddpose=0;
                ddpose=2*M_PI*dpose/3600.0;
                spinfun(ddpose);

                arvstate="";
                arvstate+=0x02;
                arvstate+=msg->routeid;
                pubstate();
                //cout<<"arrived destination"<<endl;
                //cout<<endl;
                if(returnFlag) //发布入库
                {
                    back();
                }
            }
            //cout<<call_count<<" finist detect task,stopFlag: "<<stopFlag<<endl;
            //cout<<endl;
            ROS_INFO("Move:finish detect tasks: %d, stopFlag: %d",call_count,stopFlag);
        }
        else
        {
            //cout<<call_count<<" finist other task,stopFlag: "<<stopFlag<<endl;
            //cout<<endl;
            ROS_INFO("Move:finish other tasks: %d, stopFlag: %d",call_count,stopFlag);
        }
    }

    void posecallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg1)//位姿消息的回调函数,返回实时位姿//需要设定订阅频率
    {
        x0 = msg1->pose.pose.position.x ;//机器人实时x坐标
        y0 = msg1->pose.pose.position.y ;//机器人实时y坐标
        th0 = tf::getYaw(msg1->pose.pose.orientation);//机器人实时姿态角
    }

    double getYawBias(double aimtheta) // 求去偏航角，单位：rad，从机器人角度指向目标角度 逆时针方向为正
    {
        double yawbias=0;
        double yawb=0;
        yawb=th0-aimtheta;
        if(((-2*M_PI)<=yawb)&&(yawb<(-1*M_PI))){yawbias = -2*M_PI-yawb;}
        else if(((-1*M_PI)<=yawb)&&(yawb<0))   {yawbias = -1*yawb;}
        else if((0<=yawb)&&(yawb<M_PI))        {yawbias = -1*yawb;}
        else if((M_PI<=yawb)&&(yawb<=2*M_PI))  {yawbias = 2*M_PI-yawb;}

        return yawbias;
    }

    double getTHe(double targettheta,double realth)
    {
        double TH1=0.0;
        double TH2=0.0;
        TH1=targettheta-realth;
        if(((-2*M_PI<=TH1))&&(TH1<(-M_PI)))  {TH2 = TH1+2*M_PI;}
        else if((-M_PI<=TH1)&&(TH1<=M_PI))   {TH2 = TH1;}
        else if((M_PI<TH1)&&(TH1<=(2*M_PI))) {TH2 = -2*M_PI+TH1;}

        //cout<<"target "<<targettheta<<" real "<<realth<<"error "<<TH2<<endl;

        return TH2;
    }

    void move_(int vvt,int wt)
    {
        communication::sendCmd automov;
        char ctrmov3[10]={0};
        ctrmov3[0]=0x41;//运动控制处理逻辑指令
        ctrmov3[1]=vvt>>8;
        ctrmov3[2]=vvt;
        ctrmov3[3]=0x42;
        ctrmov3[4]=wt>>8;
        ctrmov3[5]=wt;
        ctrmov3[6]=0x43;
        ctrmov3[7]=0x00;
        ctrmov3[8]=0x44;
        ctrmov3[9]=0x00;
        automov.request.type=0x40;
        automov.request.lenth=10;
        automov.request.data="";
        automov.request.data+=strtf(automov.request.lenth,ctrmov3);
        sendCmd(clientm, automov);
        ROS_INFO("Move:Set line speed: %d, Set Angle speed: %d",vvt,wt);
    }

    void diff_control(double temp_dtheta,double dspeed,double vslowrate)
    {
        double xd = x0 +  0.6*T*cos(temp_dtheta)*vslowrate;
        double yd = y0 +  0.6*T*sin(temp_dtheta)*vslowrate;

        double wd = 0, Xe=0, Ye=0,THe=0;
        double vtd = 0, wtd = 0;

        double yknl = 0;//unit:rpm
        double yknr = 0;//unit:rpm
        double wheeldistance = 0.535;//unit:m
        double wheelradius = 150;//unit:mm

        Xe=(xd-x0)*cos(th0)+(yd-y0)*sin(th0);
        Ye=(-1)*(xd-x0)*sin(th0)+(yd-y0)*cos(th0);
        THe=getTHe(temp_dtheta,th0);

        wtd = wd + dspeed*(Ky*a*(Ye + K*THe) + (b/K)*sin(THe));
        vtd = dspeed*cos(THe)+Kx*Xe-K*THe*wtd;

        int vt=vtd*1000;
        int wt=wtd*1000;
        //减速

        int vvt=vt*vslowrate;
        //vvt unit:mm/s,wt unit: 0.001rad/s
        //如果速度太小，则角速度最大为20
        if(vvt>=400)
        {
            if(wt>0)
                wt=wt>40?40:wt;
            else
                wt=wt<-40?-40:wt;
        }
        if(vvt<400&&vvt>=300)
        {
            if(wt>0)
                wt=wt>25?25:wt;
            else
                wt=wt<-25?-25:wt;
        }

        if(vvt<300&&vvt>=250)
        {
            if(wt>0)
                wt=wt>20?20:wt;
            else
                wt=wt<-20?-20:wt;
        }

        if(vvt<250&&vvt>=200)
        {
            if(wt>0)
                wt=wt>17?17:wt;
            else
                wt=wt<-17?-17:wt;
        }

        if(vvt<200&&vvt>=150)
        {
            if(wt>0)
                wt=wt>12?12:wt;
            else
                wt=wt<-12?-12:wt;
        }

        if(vvt<150)
        {
            if(wt>0)
                wt=wt>8?8:wt;
            else
                wt=wt<-8?-8:wt;
        }

        if(vvt<0)
        {
            vvt=0;
            wt=0;
        }

        //限制角速度幅值
        //if(wt>80)
        //    wt=80;
        //if(wt<-80)
        //    wt=-80;
        //yknl = 300*(2*vvt-wt*wheeldistance)/(3.1415926*wheelradius);
        //yknr = 300*(2*vvt+wt*wheeldistance)/(3.1415926*wheelradius);
        //if((yknl*yknr)<0)
        //    wt=0;
        move_(vvt,wt);
    }

    bool slowdown(double time, int vt, double line_theta )
    {
        int v=vt;
        double last = ros::Time::now().toSec();
        double duration = 0.0;
        int count=0;
        while(duration<time&&v>0&&!stopFlag)
        {
            double thetatoturn=0;
            thetatoturn=getYawBias(line_theta);//偏航角
            if(fabs(thetatoturn)<=STOPTH)   //先旋转到正确角度
            {
                //printf("%f no need turn\n",thetatoturn);
                ROS_INFO("Move:thetatoturn : %f,less than STOPH, so no need turn",thetatoturn);
                ros::Duration(1).sleep();//待定
                return false;
            }

            double current = ros::Time::now().toSec();
            duration = current-last;
            v=vt - vt/20*count;
            if(v<0)
            {
                v=0;
            }
            move_(v,0);
            count++;
            ros::spinOnce();
            ros::Duration(0.01).sleep();
        }
        return true;
    }

    double get_error_theta(float dpointX,float dpointY)
    {
        pkg_srvs::SrvGetLine getLineService;//两点求直线服务
        geometry_msgs::PoseWithCovariance poseA,poseB;
        poseA.pose.position.x = x0;
        poseA.pose.position.y = y0;
        poseB.pose.position.x = dpointX;
        poseB.pose.position.y = dpointY;
        getLineService.request.poseA = poseA;
        getLineService.request.poseB = poseB;
        if(client1.call(getLineService))
        {
            return getLineService.response.line[3];  //求得直线角度
        }
    }

    void controller(unsigned char *pointX,unsigned char *pointY)
    {
        double error_theta = 0.0;
        double line_theta  = 0.0;
        int g_speed = dspeed*1000;
        //cout<<"g_speed "<<g_speed<<endl;
        double dtheta = 0;//目标姿态角
        int ddpointX = 0;
        int ddpointY = 0; //目标点

        double x0_last = x0;
        double y0_last = y0;

        //将目标点坐标转换成double型 单位m
        ddpointX =(((((((ddpointX | pointX[0]) << 8) | pointX[1]) << 8 ) |pointX[2]) << 8) | pointX[3]);
        ddpointY =(((((((ddpointY | pointY[0]) << 8) | pointY[1]) << 8 ) |pointY[2]) << 8) | pointY[3]);

        float dpointX = ddpointX/100.0;
        float dpointY = ddpointY/100.0;
        dtheta= get_error_theta(dpointX, dpointY);
        spinfun(dtheta);
        ros::Duration(1).sleep();

        //此时机器人已经旋转到正确的位姿，下面机器人开始走直线到达目标点
        double distance = sqrt((x0-dpointX)*(x0-dpointX)+(y0-dpointY)*(y0-dpointY));//求到目标点的距离
        double vslowrate = 1;
        double min_dis = 9999999.0;
        bool flag = true;
        bool spinFlag = true;
        double safe_distance = 0.4;//单位m
        double safe_angle = 0.03;  //单位弧度

        while(distance>STOPD && flag && !stopFlag) //没到停止距离
        {
            ros::spinOnce();
            //急停
            if(stopFlag)
            {
                break;
            }
            //如果遇到障碍就堵塞
            while(blockFlag && !stopFlag)
            {
                move_(0,0);
                ros::spinOnce();
                ros::Duration(0.02).sleep();
                //printf("block flag is true\n");
                ROS_INFO("Move: There is a block in the front!");
            }
            // 防止地图崩溃
            if(fabs(x0_last-x0)>2||fabs(y0_last-y0)>2)
            {
                ROS_INFO("Move:controller:the location is wrong\n");
                stopFlag=true;
                arvstate.clear();
                arvstate.push_back(0x00);arvstate+=0xFF; arvstate+=0x10;
                pubstate();
            }
            x0_last=x0;
            y0_last=y0;

            /***控制算法****/
            //计算机器人与目标点实时点角度差
            line_theta  = get_error_theta( dpointX, dpointY);
            error_theta = getTHe(line_theta,th0);
            distance    = sqrt((x0-dpointX)*(x0-dpointX)+(y0-dpointY)*(y0-dpointY));
            //调整减速比
            //减速过程
            if((STOPD<distance)&&(distance<3))
            {
                vslowrate=(gain/2.95)*distance+1-gain/2.95*3;
            }
            else if(distance<=STOPD)
                vslowrate=vslowrate<0.2?vslowrate:0.2;
            else
                vslowrate=1.0;
            //调整角度偏差容许值, 距离远容许值小, 理论最大偏差0.08m
            //float Angle_Distance=1.5;

            safe_angle=0.03;
            /*if(distance<Angle_Distance)
                safe_angle=0.05;
            else
                safe_angle=0.03;*/

            //如果角度差较小, 则按照给定速度直走
            if(fabs(error_theta) < safe_angle)
            {
                diff_control(line_theta,dspeed,vslowrate);
            }
            //如果大于调整角度, 先判断距离, 距离过近就减速到达, 过远就调整位姿
            else
            {
                if(distance > safe_distance)//距离大于调整范围，调整位姿
                {
                    double thetatoturn=0;
                    thetatoturn=getYawBias(line_theta);//偏航角
                    //printf("theta %f : error %f\n",thetatoturn,error_theta);
                    ROS_INFO("Move:controller:theta: %.3f, error: %.3f\n",thetatoturn,error_theta);
                    spinFlag = slowdown(2.0,g_speed*vslowrate,line_theta); //减速
                    if(spinFlag)
                    {
                        move_(0,0);//停车
                        ros::spinOnce();
                        spinfun(line_theta);//旋转到目标角度
                    }
                }
                else//距离小于调整范围直走
                {
                    //cout<<"close to target "<<vslowrate<<" ";
                    ROS_INFO("Move:close to target, so vslowrate: %.3f",vslowrate);
                    move_(g_speed*vslowrate,0);
                }
            }
            //停车判断
            if(distance < safe_distance)
            {
                if(distance<=min_dis)
                {
                    min_dis=distance;
                }
                else
                {
                    if((distance-min_dis)>(STOPD))
                    {
                        flag=false;
                        //printf("flag is flase\n");
                        ROS_INFO("Move:controller:flag is false!");
                    }
                }
            }

            ros::Duration(0.02).sleep();
        }
        move_(0,0);//停车
    }

    void sendCmdtoRound(ros::ServiceClient client, communication::sendCmd srv)
    {
        int count = 0;
        while(count<5)
        {
            if(client.call(srv)&&srv.response.receive)
            {
                //ROS_INFO("Move: send cmd to Xuanzhuanban success!");
                count = 5;
            }
            else
            {
                ROS_INFO("Move: send cmd to Xuanzhuanban again! %d",count);
                count++;
            }
        }
    }

	
	void EnableMotor(char state)
	{
		communication::sendCmd srvm;//发送给运控板
		srvm.request.type=0x70;
		srvm.request.lenth=1;
		srvm.request.data=state;
		sendCmd(clientm, srvm);
	}
	
	
    void spinfun(double targetth) //控制机器人旋转
    {
        communication::sendCmd automov;
        short int wfirst;
        double spinslowrate;
        char ctrmov1[10]={0};
        char ctrmov2[10]={0};
        ctrmov2[0]=0x41;     //开始旋转前先停止1s
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
        sendCmd(clientm, automov);
        ros::Duration(1).sleep();

		
		
        double thetatoturn=0;
        thetatoturn=getYawBias(targetth);//偏航角
        ROS_INFO("Move:spinfun, YawBias:%.3f",thetatoturn);
        if(fabs(thetatoturn)<=STOPTH)    //先旋转到正确角度
        {
            ROS_INFO("Move:YawBias is less than STOPH,so no need turn\n");
            ros::Duration(1).sleep();    //待定
            return;
        }
        char hold[23]={0x41,0x00,0x42,0x01,0x43,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                       0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x44,0x00};//抱闸
        char zero[23]={0x41,0x00,0x42,0x00,0x43,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                       0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x44,0x00}; //回0度
        char turn[23]={0x41,0x00,0x42,0x00,0x43,0xFF,0xFF,0xEE,0x6C,0x00,0x00,0x11,0x94,
                       0x00,0x00,0x11,0x94,0xFF,0xFF,0xEE,0x6C,0x44,0x00}; //旋转45度

					   
        //指出轮子调整目标，45度
        to45angle=true;
        to0angle=false;
        //先调整轮子角度
        communication::sendCmd srvmR;
        srvmR.request.type=0x40;
        srvmR.request.lenth=23;
        srvmR.request.data.clear();
        srvmR.request.data+=strtf(srvmR.request.lenth, turn);
        sendCmdtoRound(clientmRound,srvmR);
        ROS_INFO("Move: send 45 Round to Xuanzhuanban success!");
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
        pub_turn.publish(msg);
        ROS_INFO("Move:in 8 state, roundFlag:%d",roundFlag);
        // cout<<"thetatoturn"<<thetatoturn<<endl;
        //宋凯
		//关闭右后电机使能
		char state=0x0E;
		EnableMotor(state);
		ROS_INFO("Move:Close motor");
	
		//等待电关闭
		int retry_num = 0;
		while(motorState!=0x0E)
		{   
			ros::spinOnce();
			ros::Duration(0.1).sleep();
			retry_num++;
			if(retry_num == 10)
			{
				  retry_num=0;
				  EnableMotor(state);
				  ROS_INFO("Move:Close motor again");
			}
		}
		ROS_INFO("Move:motor closed");
        while(thetatoturn>STOPTH||thetatoturn<(-STOPTH))   //先旋转到正确角度
        {
            ros::spinOnce();
            if(stopFlag)
            {
                //char ctrmov2[10]={0};//转动停止
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
                sendCmd(clientm, automov);
                break;
            }
            wfirst=0;
            spinslowrate=1.0;
            if(((thetatoturn>STOPTH)&&(thetatoturn<SLOWTH))||((thetatoturn<(-STOPTH))&&(thetatoturn>(-SLOWTH))))
            {
                spinslowrate=(0.8/0.38)*fabs(thetatoturn)+1-0.8/0.38*0.4;
                //printf("slow rate%f\n",spinslowrate);
            }

            //判断旋转翻方向
            if(thetatoturn>0){wfirst=(short int)((-1)*W*spinslowrate);}
            if(thetatoturn<0){wfirst=(short int)(W*spinslowrate);}
            
            if(thetatoturn>2.61) wfirst*=-1;
            
            
            //当某个直行电机电流值大于8000mA时，机器人适应电流方向，反向旋转半圈，
			int safeCurrent=8000;
            if((abs(lfzxcurrent)>safeCurrent)||(abs(rfzxcurrent)>safeCurrent)||(abs(lbzxcurrent)>safeCurrent)||(abs(rbzxcurrent)>safeCurrent))
            {
                //宋
                //过流则全部电机停使能10S，重启
                ctrmov2[0]=0x41;     //停车
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
                sendCmd(clientm, automov);
                //关使能
                highCurrentFlag = true;
                char state = 0x00;
                EnableMotor(state);
                ROS_INFO("Move:ZXMotorCurrent is greater than safe current, close motor");
                int retry_num = 0;
				while(motorState!=0x00)
				{   
					ros::spinOnce();
					ros::Duration(0.1).sleep();
					retry_num++;
					if(retry_num == 3)
					{
						retry_num=0;
						EnableMotor(state);
						ROS_INFO("Move:Close motor again");
					}
				}
				//原地停止10s
				ros::Duration(10).sleep();
				//开使能
				state = 0x0E;
				EnableMotor(state);
				retry_num = 0;
				while(motorState!=0x0E)
				{   
					ros::spinOnce();
					ros::Duration(0.1).sleep();
					retry_num++;
					if(retry_num == 10)
					{
						retry_num=0;
						EnableMotor(state);
						ROS_INFO("Move:Open motor again");
					}
				}
				highCurrentFlag = false;
				ROS_INFO("Move: has waited 10s, open motor");
                ROS_INFO("Move:Calculate thetatoturn, spinslowrate and wfirst again.");
                thetatoturn=getYawBias(targetth);//重新计算偏航角，及速度系数
                if(((thetatoturn>STOPTH)&&(thetatoturn<SLOWTH))||((thetatoturn<(-STOPTH))&&(thetatoturn>(-SLOWTH))))
                {
                    spinslowrate=(0.8/0.38)*fabs(thetatoturn)+1-0.8/0.38*0.4;
                    //printf("slow rate%f\n",spinslowrate);
                }

                //判断旋转翻方向，同时计算速度值;
                if(thetatoturn>0){wfirst=(short int)((-1)*W*spinslowrate);}
                if(thetatoturn<0){wfirst=(short int)(W*spinslowrate);}
                if(thetatoturn>2.61) wfirst*=-1;
                
            }
            ROS_INFO("Move:spinfun speed :%d",wfirst);
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
            sendCmd(clientm, automov);
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
        sendCmd(clientm, automov);
	
		
        //指出轮子调整目标，0度
        to45angle=false;
        to0angle=true;
        srvmR.request.data.clear();
        srvmR.request.data+=strtf(srvmR.request.lenth, zero);
        sendCmdtoRound(clientmRound,srvmR);//电机恢复直行状态
        ROS_INFO("Move: send 0 Round to Xuanzhuanban success!");
        //printf("after target%f,real%f\n",targetth,th0);
        ROS_INFO("Move:spinfun:Target Angle:%.3f, Real Angle:%.3f",targetth,th0);
        ros::Duration(2).sleep();
        //等待旋转到位
        while(!roundFlag)
        {
            ros::spinOnce();
            ros::Duration(0.02).sleep();
        }
        roundFlag = false; //等待旋转到位后旋转到位标记置0；
        //抱闸 直线
        srvmR.request.data.clear();
        srvmR.request.data+=strtf(srvmR.request.lenth, hold);
        sendCmdtoRound(clientmRound,srvmR);//电机抱闸

        //打开右后电机使能
		state=0x0F;
		EnableMotor(state);
		ROS_INFO("Move:open motor");
		//等待电机开启
		retry_num=0;
		while(motorState!=0x0F)
		{   
			ros::spinOnce();
			ros::Duration(0.1).sleep();
			retry_num++;
			if(retry_num == 10)
			{
				retry_num=0;
				EnableMotor(state);
				ROS_INFO("Move:open motor again");
			}
		}
		ROS_INFO("Move: motor opened");  
        //printf("turn success,flag: %d\n",roundFlag);
        ROS_INFO("Move:spinfun:spin turn success, roundFlag is %d",roundFlag);
        msg.turnFlag=false;
        pub_turn.publish(msg);
        pub_turn.publish(msg);
        
        
    }

private:
    ros::NodeHandle n;
    ros::Publisher pub_state;
    ros::Publisher pub_point;
    ros::Publisher pub_turn;
    ros::Publisher pub_charge;

    ros::Subscriber sub1;
    ros::Subscriber sub2;
    ros::Subscriber sub3;
    ros::Subscriber sub4;
	ros::Subscriber odometrySensorSubscriber;

    ros::ServiceClient client1;
    ros::ServiceClient clientm;
    ros::ServiceClient clientmRound;
    ros::ServiceClient clientn;
    ros::ServiceClient clientback;
};

int main(int argc, char**argv)
{
    ros::init(argc, argv, "move");

    MoveControl SAObject;
    ros::MultiThreadedSpinner s(8);
    ros::spin(s);

    return 0;
}
