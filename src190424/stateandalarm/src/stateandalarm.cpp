#include "ros/ros.h"
#include "communication/state.h"
#include "stateandalarm/state.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "pkg_msgs/MsgOdometrySensor.h"
#include "processor/moveorder.h"         //主控制器提供的消息
#include "communication/sendCmd.h"
#include "move/charge.h"
#include "laser_node/laser_cur_state.h"
#include "stateandalarm/record.h"        //充放电信息 放电为false
#include <iomanip>
#include <processor/isTurn.h>

#include <sstream>
#include <stdlib.h>
#include <string>
#include <cmath>

using namespace std;

#define RED      0x52
#define GREEN    0x53
#define BLUE     0x54
#define SOUND    0x55
#define ON       0x01
#define OFF      0x00

//驱动器使能
#define driverEnable 0x0F

char origin[23]={0x41,0x01,0x42,0x00,0x43,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x44,0x00};//回原位

int hightmp=60;               //机器人本体高温线
int lowtmp=-5;                //机器人本体低温线
//float radio=0.15;           //机器人轮半径
float spdfactor=0.0008;       //机器人转速转化为线速度的系数
float grid=M_PI*0.30/200000;  //编码器单位栅格长度   0.35
bool first=true;
unsigned char comstate[6];    //通讯状态

int V_speed=0;
int test_count=0;

int red_count=0;
int yellow_count=0;
const char* file="/home/robot/log/record/alarm.txt";
const char* file_odom="/home/robot/log/record/odom.txt";
const char* file_odom2="/home/robot/log/record/raw_odom.txt";
class robotState
{
public:
    int count = 0;                     //测试用
    unsigned char state[104]={0};      //发送给上位机的状态

    unsigned char robotstate[3]={0};   //机器人本体状态
    unsigned char wave[3]={0};         //超声波信息

    double testdislf=0.0;
    double testdislb=0.0;
    double testdisrf=0.0;
    double testdisrb=0.0;

    float VLF=0.0;
    float VLB=0.0;
    float VRF=0.0;
    float VRB=0.0;

    int CDLF=0;
    int CDLB=0;
    int CDRF=0;
    int CDRB=0;
    int LDLF=0;
    int LDLB=0;
    int LDRF=0;
    int LDRB=0;

    float DLF=0.0;
    float DLB=0.0;
    float DRF=0.0;
    float DRB=0.0;

    ros::Time lasttime;
    ros::Time curtime;

    int upFlow = pow(2,31)-1;
    int downFlow = -pow(2,31);

    bool moveFlag;         //与运控板通讯 标志位
    bool moveRoundFlag;    //与运控板通讯 标志位
    bool plcFlag;          //与PLC通讯 标志位
    bool lowBatteryF;
    bool batteryF;
    bool ultrasonicF;      //超声波
    bool temperatureF;
    bool driverFlag;
    bool driverEnableF;
    bool is_inroom;
    bool turnFlag;
    ofstream OsWrite;
    ofstream OsWrite1;
    ofstream OsWrite2;

    robotState()
    {
        moveFlag=true;
        plcFlag=false;
        moveRoundFlag=true;

        lowBatteryF=true;
        batteryF=true;
        temperatureF=true;
        ultrasonicF=true;
        driverFlag=true;
        driverEnableF=true;
        is_inroom=false;   //默认机器人不在充电屋内
        turnFlag=false;
        state[0]=0x01;     //接收正常
        state[1]=0x51;     //机器人坐标，cm
        state[10]=0x52;    //机器人位姿
        state[13]=0x53;    //机器人速度
        state[16]=0x54;    //云台高度
        state[19]=0x55;    //机器人电量
        state[21]=0x56;    //机内温度
        state[26]=0x57;    //急停状态
        state[28]=0x58;    //超声波状态
        state[30]=0x59;    //机器人控制模式
        state[32]=0x5A;    //是否到达目标点
        state[34]=0x5B;    //机器人状态
        state[57]=0x5C;    //路径片id
        state[58]=0xFF;    //路径ID 初始赋值0xFF
        state[59]=0x5D;    //电机使能状态
        state[62]=0x5E;    //电机电流
        state[79]=0x5F;    //机器人本体设备状态

        //坐标 (0,4.80,-90)
        int Xcm=0;//
        int Ycm=480;
        int thdu=-900;
        state[2]=Xcm>>24;state[3]=Xcm>>16;state[4]=Xcm>>8;state[5]=Xcm;
        state[6]=Ycm>>24;state[7]=Ycm>>16;state[8]=Ycm>>8;state[9]=Ycm;
        state[11]=thdu>>8;state[12]=thdu;

        pub_state = n.advertise<communication::state>("processor/state",1000);

        pub_sensor = n.advertise<pkg_msgs::MsgOdometrySensor>("topic_odometry_sensor",1000);
        //发布给运动控制节点move
        pub = n.advertise<processor::moveorder>("moveorder", 1000);
        //订阅机器人位姿信息
        sub1 = n.subscribe("topic_robot_pose",1, &robotState::statecallback1,this);
        //订阅机器人实时速度消息
        sub2 = n.subscribe("odom",1, &robotState::statecallback2,this);
        //订阅plc板的状态消息
        sub3 = n.subscribe("communication/state_plc", 1, &robotState::statecallback3,this);
        //订阅运动控制板的状态消息
        sub4 = n.subscribe("communication/state_move", 10, &robotState::statecallback4,this);
        //订阅路径规划的信息
        sub5 = n.subscribe("state_route",1, &robotState::statecallback5,this);
        //订阅工控机指令消息
        sub6 = n.subscribe("moveorder",1, &robotState::statecallback6,this);//接收主控板消息
        //订阅旋转控制板的状态消息
        sub7 = n.subscribe("communication/state_moveRound", 1, &robotState::statecallback7,this);
        //订阅充放电状态消息
        sub8 = n.subscribe("parking/chargeState", 1, &robotState::statecallback8,this);
        //订阅转弯信息
        sub9 = n.subscribe("processor/turn", 10, &robotState::spinAround,this);
        //订阅激光节点初始化和崩溃消息
        sub10 = n.subscribe("laser_node/laser_state_cur", 10, &robotState::lasernodestate,this);
        //给PLC的发送消息的服务
        clientp = n.serviceClient<communication::sendCmd>("send_plc_cmd");
        //给运动控制板发送消息的服务
        clientm = n.serviceClient<communication::sendCmd>("send_move_cmd");
        //给旋转控制板发送消息的服务
        clientmRound = n.serviceClient<communication::sendCmd>("send_moveRound_cmd");

        //打开记录文本
        openFile(file,OsWrite);
        record_string("start",OsWrite);

        openFile(file_odom,OsWrite1);
        record_string("start",OsWrite1);

        openFile(file_odom2,OsWrite2);
        record_string("start",OsWrite2);
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
        sendCommand(client, srvp);
    }

    void sendCommand(ros::ServiceClient client, communication::sendCmd srv) //重复下发命令
    {
        int count = 0;
        while(count<5)
        {
            if(client.call(srv)&&srv.response.receive)
            {
                count = 5;
            }
            else
            {
                //  ROS_INFO("call again");
                count++;
            }
        }
    }

    void lasernodestate(const laser_node::laser_cur_state::ConstPtr& msg)
    {
        if(msg->laser_state==0)
            state[37]=state[37]|0x02;
        else
            state[37]=state[37]&0xFD;//清除激光异常报警标志;
        this->pubstate();
    }

    void spinAround(const processor::isTurn::ConstPtr& msg)
    {
        turnFlag=msg->turnFlag;
    }
    void emergencyStop(bool flag) //flag true 表示为急停，false 为普通停止
    {
        char ctrmov[10];
        ctrmov[0]=0x41;
        ctrmov[1]=0x00;
        ctrmov[2]=0x00;
        ctrmov[3]=0x42;
        ctrmov[4]=0x00;
        ctrmov[5]=0x00;
        ctrmov[6]=0x43;
        ctrmov[7]=0x01;
        ctrmov[8]=0x44;
        ctrmov[9]=0x00;

        processor::moveorder moveorder;
        if(flag)
        {
            moveorder.stopflag=true;
            communication::sendCmd srvm;//发送给运控板
            srvm.request.type=0x40;
            srvm.request.lenth=10;
            srvm.request.data="";
            srvm.request.data+=strtf(srvm.request.lenth, ctrmov);
            sendCommand(clientm, srvm);
            ROS_INFO("emergency stop");
        }
        else
        {
            if(V_speed<0||turnFlag) //如果后退，则不停
            {
                ;
            }
            else
            {
                communication::sendCmd srvm;//发送给运控板
                srvm.request.type=0x40;
                srvm.request.lenth=10;
                srvm.request.data="";
                srvm.request.data+=strtf(srvm.request.lenth, ctrmov);
                sendCommand(clientm, srvm);
            }
            moveorder.block=true;
        }
        pub.publish(moveorder);
        ros::Duration(1).sleep();
    }

    void redAlarm(ros::ServiceClient &client,bool flag=true)
    {
        red_count++;
        if(red_count==1)                  //如果有红色并且只有一个
        {
            Light(GREEN,OFF,client);        //关闭绿色报警灯
            Light(RED,ON,client);         //打开红色报警灯
            // Light(SOUND,ON,client);    //打开蜂鸣器
            emergencyStop(flag);
        }
        if(red_count<0) red_count=0;//****
    }

    void yellowAlarm(ros::ServiceClient &client)
    {
        yellow_count++;
        if(!red_count&&yellow_count==1) //如果无红色并且只有一个黄色则开黄灯
        {
            Light(GREEN,ON,client);     //打开绿色报警灯
            Light(RED,ON,client);       //打开红色报警灯
            Light(SOUND,OFF,client);    //关闭蜂鸣器
        }
    }

    void greenNormal(ros::ServiceClient &client)
    {
        if(!red_count&&!yellow_count)  //如果无红色无黄色则开绿灯
        {
            Light(GREEN,ON,client);    //打开绿色报警灯
            Light(RED,OFF,client);     //关闭红色报警灯
            Light(SOUND,OFF,client);   //关闭蜂鸣器
        }
        else if(!red_count&&yellow_count>=1)//如果无红色有黄色则开黄灯
        {
            Light(GREEN,ON,client);    //打开绿色报警灯
            Light(RED,ON,client);      //打开红色报警灯
            Light(SOUND,OFF,client);   //关闭蜂鸣器
        }
    }
    int convert(unsigned char a,unsigned b)
    {
        int temp=0;
        if(a&0x80) //负数
        {
            a=~a;b=~b;
            temp=a<<8|b+1;
            temp=-temp;
        }
        else
        {
            temp=a<<8|b;
        }
        return temp;
    }

    void statecallback1(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg1)  //获取机器人坐标以及姿态角度；
    {

        //ROS_INFO("receive msg from nav\n");
        double X,Y,th;

        X = msg1->pose.pose.position.x;
        Y = msg1->pose.pose.position.y;
        th = tf::getYaw(msg1->pose.pose.orientation);//实际使用
        int Xcm=0;//位置数据待处理
        int Ycm=0;
        Xcm=X*100;
        Ycm=Y*100;
        int thdu=0;//姿态数据待处理
        thdu=10*th*360/(2*M_PI);//转换为 度

        state[0]=0x01;//接收正常
        state[1]=0x51;//机器人坐标，cm
        state[2]=Xcm>>24;
        state[3]=Xcm>>16;
        state[4]=Xcm>>8;
        state[5]=Xcm;
        state[6]=Ycm>>24;
        state[7]=Ycm>>16;
        state[8]=Ycm>>8;
        state[9]=Ycm;
        state[10]=0x52;//机器人位姿
        state[11]=thdu>>8;
        state[12]=thdu;

        if(is_inroom) //在充电屋内
        {
            //坐标 (0,4.80,-90)
            int Xcm=0;//
            int Ycm=480;
            int thdu=-900;
            state[2]=Xcm>>24;state[3]=Xcm>>16;state[4]=Xcm>>8;state[5]=Xcm;
            state[6]=Ycm>>24;state[7]=Ycm>>16;state[8]=Ycm>>8;state[9]=Ycm;
            state[11]=thdu>>8;state[12]=thdu;
        }
        this->pubstate();
    }

    void statecallback2(const nav_msgs::Odometry::ConstPtr& msg2) //获取机器人速度信息
    {
        //  ROS_INFO("receive msg from odom!");
        double V=0;
        double Vx=msg2->twist.twist.linear.x;
        double Vy=msg2->twist.twist.linear.y;

        V=sqrt(Vx*Vx+Vy*Vy);//单位 m/s
        if(DLF<0)
            V=-V;
        int Vcms=0;
        Vcms=V*100;//单位 cm/s

        V_speed=Vcms;

        state[13]=0x53;//机器人速度
        state[14]=Vcms>>8;
        state[15]=Vcms;

        this->pubstate();
    }

    void statecallback3(const communication::state::ConstPtr& msg3)//获取PLC 板信息；
    {
        comstate[0]=0x00;      //工控机与PLC通讯超时 状态
        comstate[2]=0x00;      //工控机与PLC通讯故障 状态

        if(msg3->break_flag)   //与PLC通讯故障
        {
            comstate[2]=0x20;
            comstate[0]=0x00;  //工控机与PLC通讯故障 状态

            plcFlag=false;
        }
        else if(msg3->wrong_flag)  //与PLC通讯超时
        {
            comstate[0]=0x02;
            comstate[2]=0x00;  //工控机与PLC通讯超时 状态
            plcFlag=false;
        }
        else
        {
            if(plcFlag==false)
            {
                if(red_count>0){red_count--;}
                greenNormal(clientp);

                plcFlag=true;
                comstate[0]=0x00;  //工控机与PLC通讯超时 状态
                comstate[2]=0x00;  //工控机与PLC通讯故障 状态
            }
            char plc[50];
            memcpy(plc,msg3->data.c_str(),msg3->data.size()+1);

            state[21]=0x56;   //机内温度
            state[22]=plc[8]; //电池温度
            state[23]=plc[9];

            state[39]=plc[2]; //机器人电池错误状态
            state[40]=plc[3];

            //电源报警（除欠压）
            if(!((state[40]&0xF0)==0x00))
            {
                if(batteryF)
                {
                    redAlarm(clientp);
                    ROS_INFO("Stateandalarm: battery alarm ， stop!");
                    batteryF=false;
                    record_string("battery alarm",OsWrite);
                }
            }
            else
            {
                if(!batteryF)
                {
                    red_count--;
                    greenNormal(clientp);
                    batteryF=true;
                    record_string("battery alarm clear",OsWrite);
                }
            }

            //电池欠压报警
            if(!((state[40]&0x0F)==0x00))
            {
                if(lowBatteryF)
                {
                    yellowAlarm(clientp);
                    lowBatteryF=false;

                    record_string("low battery alarm ",OsWrite);
                }
            }
            else
            {
                if(!lowBatteryF)
                {
                    yellow_count--;
                    greenNormal(clientp);
                    lowBatteryF=true;

                    record_string("low battery alarm clear",OsWrite);
                }
            }

            short robottmp=0;
            short tmp1=0;
            tmp1=(plc[13]<<8) + ((short)plc[14]&0x00ff);

            robottmp=tmp1/10;
            robotstate[0]=0x00;
            robotstate[1]=0x00;

            if(robottmp>=hightmp)      {robotstate[0]=0x20;}
            else if(robottmp<=lowtmp)  {robotstate[1]=0x40;}


            state[24]=robottmp>>8; //工控机仓温度
            state[25]=robottmp;

            state[37]=0x00;        //机器人本体错误状态
            state[38]=robotstate[0] | robotstate[1];

            //本体温度报警
            if(!((state[38]&0xEF)==0x00))
            {
                if(temperatureF)
                {
                    yellowAlarm(clientp);
                    temperatureF=false;
                    record_string("robot temp alarm",OsWrite);
                }
            }
            else
            {
                if(!temperatureF)
                {
                    yellow_count--;
                    greenNormal(clientp);
                    temperatureF=true;
                    record_string("robot temp alarm clear",OsWrite);
                }
            }

            state[79]=0x5F;  //机器人本体设备状态
            state[80]=plc[5];
            state[81]=plc[6];
            state[82]=0x00;  //急停开关状态 默认正常

            if(!(plc[11]&0x40)) //第6位为0则表示急停
            {
                state[82]=0x01;
                LDLF=0; LDLB=0; LDRF=0; LDRB=0;
                emergencyStop(true); //flag true 表示为急停
                // cout<<"emergencyStop"<<endl;
            }

            state[19]=0x55;//机器人电量
            state[20]=plc[22];
        }
        this->pubstate();
    }

    void statecallback7(const communication::state::ConstPtr& msg7)//获取旋转运控板的信息
    {
        if(!state[83])
        {
            comstate[4]=0x00;  //工控机与旋转板通讯超时 状态
            comstate[5]=0x00;  //工控机与旋转板通讯故障 状态

            if(msg7->break_flag)
            {
                if(moveRoundFlag)
                {
                    comstate[4]=0x80;  //工控机与旋转板通讯超时 状态
                    comstate[5]=0x00;  //工控机与旋转板通讯故障 状态
                    redAlarm(clientp);
                    ROS_INFO("Stateandalarm: moveround communication fail， stop!");
                    moveRoundFlag=false;
                    record_string("moveround communication fail alarm",OsWrite);
                }
                comstate[4]=0x80;  //工控机与旋转板通讯超时 状态
                comstate[5]=0x00;  //工控机与旋转板通讯故障 状态
                // cout<<"move round fail"<<endl;
            }
            else if(msg7->wrong_flag)
            {
                comstate[4]=0x08;  //工控机与旋转板通讯超时 状态
                comstate[5]=0x00;  //工控机与旋转板通讯故障 状态
                // moveRoundFlag=false;
                //cout<<"move round wrong"<<endl;
                record_string("moveround communication wrong alarm",OsWrite);
            }
            else if(!msg7->data.empty())
            {
                comstate[4]=0x00;  //工控机与运控板通讯 超时状态
                comstate[5]=0x00;  //工控机与运控板通讯 错误状态

                if(moveRoundFlag==false)
                {
                    if(red_count>0) {red_count--;}
                    greenNormal(clientp);

                    record_string("moveround communication fail alarm clear",OsWrite);

                    moveRoundFlag=true;
                }

                unsigned char mov[100];

                memcpy(mov,msg7->data.c_str(),msg7->data.size()+1);
                //使能状态
                state[61]=mov[4];
                for(int i=0;i<85;i++)
		{
		  printf("mov[%d] = %x\n", i, mov[i]);
		}
                // 电机故障情况
                unsigned int mdstate[65]={0};
                unsigned int lumd=0;
                unsigned int ldmd=0;
                unsigned int wumd=0;
                unsigned int wdmd=0;

                mdstate[0]=0;
                if(mov[18]==0x01){mdstate[1]=(int)0x0001;}// 左上电机
                if(mov[18]==0x02){mdstate[2]=(int)0x0002;}
                if(mov[18]==0x04){mdstate[3]=(int)0x0004;}
                if(mov[18]==0x08){mdstate[4]=(int)0x0008;}
                if(mov[18]==0x10){mdstate[5]=(int)0x0010;}
                if(mov[18]==0x20){mdstate[6]=(int)0x0020;}
                if(mov[18]==0x40){mdstate[7]=(int)0x0040;}
                if(mov[18]==0x80){mdstate[8]=(int)0x0080;}
                if(mov[17]==0x01){mdstate[9]=(int)0x0100;}
                if(mov[17]==0x02){mdstate[10]=(int)0x0200;}
                if(mov[17]==0x04){mdstate[11]=(int)0x0400;}
                if(mov[17]==0x08){mdstate[12]=(int)0x0800;}
                if(mov[17]==0x10){mdstate[13]=(int)0x1000;}
                if(mov[17]==0x20){mdstate[14]=(int)0x2000;}
                if(mov[17]==0x40){mdstate[15]=(int)0x4000;}
                if(mov[17]==0x80){mdstate[16]=(int)0x8000;}

                for(int i=1;i<17;i++){lumd+=mdstate[i];}

                state[49]=lumd>>8;
                state[50]=lumd;

                if(mov[26]==0x01){mdstate[17]=(int)0x0001;}// 左下电机
                if(mov[26]==0x02){mdstate[18]=(int)0x0002;}
                if(mov[26]==0x04){mdstate[19]=(int)0x0004;}
                if(mov[26]==0x08){mdstate[20]=(int)0x0008;}
                if(mov[26]==0x10){mdstate[21]=(int)0x0010;}
                if(mov[26]==0x20){mdstate[22]=(int)0x0020;}
                if(mov[26]==0x40){mdstate[23]=(int)0x0040;}
                if(mov[26]==0x80){mdstate[24]=(int)0x0080;}
                if(mov[25]==0x01){mdstate[25]=(int)0x0100;}
                if(mov[25]==0x02){mdstate[26]=(int)0x0200;}
                if(mov[25]==0x04){mdstate[27]=(int)0x0400;}
                if(mov[25]==0x08){mdstate[28]=(int)0x0800;}
                if(mov[25]==0x10){mdstate[29]=(int)0x1000;}
                if(mov[25]==0x20){mdstate[30]=(int)0x2000;}
                if(mov[25]==0x40){mdstate[31]=(int)0x4000;}
                if(mov[25]==0x80){mdstate[32]=(int)0x8000;}
                for(int j=17;j<33;j++){ldmd+=mdstate[j];}
                state[51]=ldmd>>8;
                state[52]=ldmd;

                if(mov[22]==0x01){mdstate[33]=(int)0x0001;}// 右上电机
                if(mov[22]==0x02){mdstate[34]=(int)0x0002;}
                if(mov[22]==0x04){mdstate[35]=(int)0x0004;}
                if(mov[22]==0x08){mdstate[36]=(int)0x0008;}
                if(mov[22]==0x10){mdstate[37]=(int)0x0010;}
                if(mov[22]==0x20){mdstate[38]=(int)0x0020;}
                if(mov[22]==0x40){mdstate[39]=(int)0x0040;}
                if(mov[22]==0x80){mdstate[40]=(int)0x0080;}
                if(mov[21]==0x01){mdstate[41]=(int)0x0100;}
                if(mov[21]==0x02){mdstate[42]=(int)0x0200;}
                if(mov[21]==0x04){mdstate[43]=(int)0x0400;}
                if(mov[21]==0x08){mdstate[44]=(int)0x0800;}
                if(mov[21]==0x10){mdstate[45]=(int)0x1000;}
                if(mov[21]==0x20){mdstate[46]=(int)0x2000;}
                if(mov[21]==0x40){mdstate[47]=(int)0x4000;}
                if(mov[21]==0x80){mdstate[48]=(int)0x8000;}
                for(int k=33;k<49;k++){wumd+=mdstate[k];}
                state[53]=wumd>>8;
                state[54]=wumd;

                if(mov[30]==0x01){mdstate[49]=(int)0x0001;}// 右下电机
                if(mov[30]==0x02){mdstate[50]=(int)0x0002;}
                if(mov[30]==0x04){mdstate[51]=(int)0x0004;}
                if(mov[30]==0x08){mdstate[52]=(int)0x0008;}
                if(mov[30]==0x10){mdstate[53]=(int)0x0010;}
                if(mov[30]==0x20){mdstate[54]=(int)0x0020;}
                if(mov[30]==0x40){mdstate[55]=(int)0x0040;}
                if(mov[30]==0x80){mdstate[56]=(int)0x0080;}
                if(mov[29]==0x01){mdstate[57]=(int)0x0100;}
                if(mov[29]==0x02){mdstate[58]=(int)0x0200;}
                if(mov[29]==0x04){mdstate[59]=(int)0x0400;}
                if(mov[29]==0x08){mdstate[60]=(int)0x0800;}
                if(mov[29]==0x10){mdstate[61]=(int)0x1000;}
                if(mov[29]==0x20){mdstate[62]=(int)0x2000;}
                if(mov[29]==0x40){mdstate[63]=(int)0x4000;}
                if(mov[29]==0x80){mdstate[64]=(int)0x8000;}
                for(int l=49; l<65; l++){wdmd+=mdstate[l];}
                state[55]=wdmd>>8;
                state[56]=wdmd;

                //电流
                state[71]=mov[6];   //左前
                state[72]=mov[7];
                state[73]=mov[8];   //右前
                state[74]=mov[9];
                state[75]=mov[10];  //左后
                state[76]=mov[11];
                state[77]=mov[12];  //右后
                state[78]=mov[13];

                state[84]=mov[56];  //旋转复位标志 00未到位 01到位 02 正在旋转

                if(mov[56]==0x00)   //旋转未复位
                {
                    /******旋转电机回原位*****/
                    communication::sendCmd srvmR;
                    srvmR.request.type=0x40;
                    srvmR.request.lenth=23;
                    srvmR.request.data="";
                    srvmR.request.data+=strtf(srvmR.request.lenth, origin);
                    if(clientmRound.call(srvmR)&&srvmR.response.receive)//电机复位
                    {
                        // ROS_INFO("send zero sucess");
                    }
                    else
                    {
                        //  ROS_INFO("send zero moveRound failed");
                    }

                }

                state[85]=0x00;
                if(mov[32]==0x0f){state[85]=0x01; }//转到达标志

                //云台高度，在旋转电机
                state[16]=0x54;
                state[17]=mov[53];
                state[18]=mov[54];

                // 旋转角度
                state[86]=mov[34];
                state[87]=mov[35];
                state[88]=mov[36];
                state[89]=mov[37];
                state[90]=mov[38];
                state[91]=mov[39];
                state[92]=mov[40];
                state[93]=mov[41];
                state[94]=mov[42];
                state[95]=mov[43];
                state[96]=mov[44];
                state[97]=mov[45];
                state[98]=mov[46];
                state[99]=mov[47];
                state[100]=mov[48];
                state[101]=mov[49];
            }
        }
        this->pubstate();
    }

    void chargeBack1()
    {

        //运控板正常
        state[27]=0x00;
        state[29]=0x00;

        state[41]=0x00; state[42]=0x00; state[43]=0x00; state[44]=0x00;
        state[45]=0x00; state[46]=0x00; state[47]=0x00; state[48]=0x00;
        state[60]=0x0f;
        state[63]=0x00; state[64]=0x00; state[65]=0x00; state[66]=0x00;
        state[67]=0x00; state[68]=0x00; state[69]=0x00; state[70]=0x00;

        //旋转板正常
        state[61]=0x00;
        state[49]=0x00; state[50]=0x00; state[51]=0x00; state[52]=0x00;
        state[53]=0x00; state[54]=0x00; state[55]=0x00; state[56]=0x00;
        state[71]=0x00; state[72]=0x00; state[73]=0x00; state[74]=0x00;
        state[75]=0x00; state[76]=0x00; state[77]=0x00; state[78]=0x00;
        state[84]=0x01;
        state[85]=0x01;
        state[86]=0x00; state[87]=0x00; state[88]=0x00; state[89]=0x00;
        state[90]=0x00; state[91]=0x00; state[92]=0x00; state[93]=0x00;
        state[94]=0x00; state[95]=0x00; state[96]=0x00; state[97]=0x00;
        state[98]=0x00; state[99]=0x00; state[100]=0x00; state[101]=0x00;

        //通信
        comstate[0]=0x00;comstate[1]=0x00;comstate[2]=0x00;comstate[3]=0x00;  //工控机与运控板通讯 正常

        LDLF=0; LDLB=0; LDRF=0; LDRB=0; //若关设备需要此句
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
        sendCommand(client, srvp);
    }

    void statecallback4(const communication::state::ConstPtr& msg4)//获取运动控制板信息；
    {

        comstate[1]=0x00;  //工控机与运控板通讯 超时状态
        comstate[3]=0x00;  //工控机与运控板通讯 错误状态
        if(!state[83]){
            if(msg4->break_flag)   //通讯故障
            {
                if(moveFlag)
                {
                    comstate[3]=0x40;
                    comstate[1]=0x00;  //工控机与运控板通讯 错误状态
                    redAlarm(clientp);
                    ROS_INFO("Stateandalarm: move communication fail， stop!");
                    record_string("move communication fail alarm",OsWrite);
                    moveFlag=false;
                    // 关闭驱动器
                    //device(0x35,OFF,clientp);
                    //device(0x36,OFF,clientp);
                    //device(0x38,OFF,clientp);
                    //device(0x3B,OFF,clientp);
                }
            }//与运控断开
            else if(msg4->wrong_flag)   //超时
            {
                comstate[1]=0x04;
                comstate[3]=0x00;  //工控机与运控板通讯 超时状态

                record_string("move communication wrong alarm",OsWrite);
            }
            else if(!msg4->wrong_flag && !msg4-> break_flag)
            {
                comstate[1]=0x00;  //工控机与运控板通讯 超时状态
                comstate[3]=0x00;  //工控机与运控板通讯 错误状态

                if(moveFlag==false)
                {
                    if(red_count!=0){red_count--;}
                    greenNormal(clientp);

                    record_string("move communication fail alarm clear",OsWrite);

                    moveFlag=true;
                }
                unsigned char mov[100];

                memcpy(mov,msg4->data.c_str(),msg4->data.size()+1);

                state[26]=0x57;//急停状态
                state[27]=mov[7];
                state[28]=0x58;//超声波状态
                state[29]=0x00;

                state[102]=mov[4];
                state[103]=mov[5];

                wave[0]=0;wave[1]=0;wave[2]=0;

                if((mov[4]&0xF0 && mov[4]&0x0F  )|| ( mov[5]&0x20 && mov[4]&0x0F ))
                {
                    wave[0]=0x01;//障碍物
                    //cout<<"block"<<endl;
                    ROS_INFO("Stateandalarm: There is a block in the front!");
                }
                if(mov[5]&0x80)
                {
                    wave[1]=0x02;//前方跌落
                    ROS_INFO("Stateandalarm: There is a gap in the front!");
                    //cout<<"fall front"<<endl;
                }
                if(mov[5]&0x40)
                {
                    wave[2]=0x04;//后方跌落
                    ROS_INFO("Stateandalarm: There is a gap in the back!");
                    //cout<<"fall back"<<endl;
                }
                state[29]=wave[0]|wave[1]|wave[2];

                //超声波 警报
                if(!(state[29]==0x00))
                {
                    if(ultrasonicF)
                    {
                        redAlarm(clientp,false);

                        processor::moveorder moveorder;
                        moveorder.block=true;

                        pub.publish(moveorder);
                        ultrasonicF=false;
                        record_string("block alarm",OsWrite);
                        ROS_INFO("Stateandalarm: There is a block by ultrasonic!");
                    }
                }
                else
                {
                    if(!ultrasonicF)
                    {
                        red_count--;
                        greenNormal(clientp);

                        processor::moveorder moveorder;
                        moveorder.block=false;

                        pub.publish(moveorder);
                        ultrasonicF=true;
                        record_string("block alarm clear",OsWrite);
                        ROS_INFO("Stateandalarm: Block alarm by ultrasonic has been cleared!");
                    }
                }

                state[34]=0x5B;//机器人状态

                unsigned int mdstate[65]={0};
                unsigned int lumd=0;
                unsigned int ldmd=0;
                unsigned int wumd=0;
                unsigned int wdmd=0;

                mdstate[0]=0;

                if(mov[41]==0x01){mdstate[1]=(int)0x0001;}// 左前电机
                if(mov[41]==0x02){mdstate[2]=(int)0x0002;}
                if(mov[41]==0x04){mdstate[3]=(int)0x0004;}
                if(mov[41]==0x08){mdstate[4]=(int)0x0008;}
                if(mov[41]==0x10){mdstate[5]=(int)0x0010;}
                if(mov[41]==0x20){mdstate[6]=(int)0x0020;}
                if(mov[41]==0x40){mdstate[7]=(int)0x0040;}
                if(mov[41]==0x80){mdstate[8]=(int)0x0080;}
                if(mov[40]==0x01){mdstate[9]=(int)0x0100;}
                if(mov[40]==0x02){mdstate[10]=(int)0x0200;}
                if(mov[40]==0x04){mdstate[11]=(int)0x0400;}
                if(mov[40]==0x08){mdstate[12]=(int)0x0800;}
                if(mov[40]==0x10){mdstate[13]=(int)0x1000;}
                if(mov[40]==0x20){mdstate[14]=(int)0x2000;}
                if(mov[40]==0x40){mdstate[15]=(int)0x4000;}
                if(mov[40]==0x80){mdstate[16]=(int)0x8000;}
                for(int i=1;i<17;i++){lumd+=mdstate[i];}
                state[41]=lumd>>8;
                state[42]=lumd;

                if(mov[49]==0x01){mdstate[17]=(int)0x0001;}// 左后电机
                if(mov[49]==0x02){mdstate[18]=(int)0x0002;}
                if(mov[49]==0x04){mdstate[19]=(int)0x0004;}
                if(mov[49]==0x08){mdstate[20]=(int)0x0008;}
                if(mov[49]==0x10){mdstate[21]=(int)0x0010;}
                if(mov[49]==0x20){mdstate[22]=(int)0x0020;}
                if(mov[49]==0x40){mdstate[23]=(int)0x0040;}
                if(mov[49]==0x80){mdstate[24]=(int)0x0080;}
                if(mov[49]==0x01){mdstate[25]=(int)0x0100;}
                if(mov[49]==0x02){mdstate[26]=(int)0x0200;}
                if(mov[49]==0x04){mdstate[27]=(int)0x0400;}
                if(mov[49]==0x08){mdstate[28]=(int)0x0800;}
                if(mov[49]==0x10){mdstate[29]=(int)0x1000;}
                if(mov[49]==0x20){mdstate[30]=(int)0x2000;}
                if(mov[49]==0x40){mdstate[31]=(int)0x4000;}
                if(mov[49]==0x80){mdstate[32]=(int)0x8000;}
                for(int j=17;j<33;j++){ldmd+=mdstate[j];}
                state[43]=ldmd>>8;
                state[44]=ldmd;

                if(mov[45]==0x01){mdstate[33]=(int)0x0001;}// 右前电机
                if(mov[45]==0x02){mdstate[34]=(int)0x0002;}
                if(mov[45]==0x04){mdstate[35]=(int)0x0004;}
                if(mov[45]==0x08){mdstate[36]=(int)0x0008;}
                if(mov[45]==0x10){mdstate[37]=(int)0x0010;}
                if(mov[45]==0x20){mdstate[38]=(int)0x0020;}
                if(mov[45]==0x40){mdstate[39]=(int)0x0040;}
                if(mov[45]==0x80){mdstate[40]=(int)0x0080;}
                if(mov[44]==0x01){mdstate[41]=(int)0x0100;}
                if(mov[44]==0x02){mdstate[42]=(int)0x0200;}
                if(mov[44]==0x04){mdstate[43]=(int)0x0400;}
                if(mov[44]==0x08){mdstate[44]=(int)0x0800;}
                if(mov[44]==0x10){mdstate[45]=(int)0x1000;}
                if(mov[44]==0x20){mdstate[46]=(int)0x2000;}
                if(mov[44]==0x40){mdstate[47]=(int)0x4000;}
                if(mov[44]==0x80){mdstate[48]=(int)0x8000;}
                for(int k=33;k<49;k++){wumd+=mdstate[k];}
                state[45]=wumd>>8;
                state[46]=wumd;

                if(mov[53]==0x01){mdstate[49]=(int)0x0001;}// 右下电机
                if(mov[53]==0x02){mdstate[50]=(int)0x0002;}
                if(mov[53]==0x04){mdstate[51]=(int)0x0004;}
                if(mov[53]==0x08){mdstate[52]=(int)0x0008;}
                if(mov[53]==0x10){mdstate[53]=(int)0x0010;}
                if(mov[53]==0x20){mdstate[54]=(int)0x0020;}
                if(mov[53]==0x40){mdstate[55]=(int)0x0040;}
                if(mov[53]==0x80){mdstate[56]=(int)0x0080;}
                if(mov[52]==0x01){mdstate[57]=(int)0x0100;}
                if(mov[52]==0x02){mdstate[58]=(int)0x0200;}
                if(mov[52]==0x04){mdstate[59]=(int)0x0400;}
                if(mov[52]==0x08){mdstate[60]=(int)0x0800;}
                if(mov[52]==0x10){mdstate[61]=(int)0x1000;}
                if(mov[52]==0x20){mdstate[62]=(int)0x2000;}
                if(mov[52]==0x40){mdstate[63]=(int)0x4000;}
                if(mov[52]==0x80){mdstate[64]=(int)0x8000;}
                for(int l=49; l<65; l++){wdmd+=mdstate[l];}
                state[47]=wdmd>>8;
                state[48]=wdmd;

                //驱动器报警
                if(!((state[41]|state[42]|state[43]|state[44]|state[45]|state[46]|state[47]|state[48]) ==0x00))
                {
                    if(driverFlag)
                    {
                        redAlarm(clientp);
                        driverFlag=false;
                        record_string("driver error alarm",OsWrite);
                        ROS_INFO("Stateandalarm: driver error alarm!");
                    }
                }
                else
                {
                    if(!driverFlag)
                    {
                        red_count--;
                        greenNormal(clientp);
                        driverFlag=true;
                        record_string("driver error alarm clear",OsWrite);
                        ROS_INFO("Stateandalarm: driver error alarm clear!");
                    }
                }

                state[59]=0x5D;     //电机使能状态
                state[60]=mov[9];

                //报警  电机不使能报警
               /* if(!(state[60]==driverEnable || state[60]==0x06))
                {
                    if(driverEnableF)
                    {
                        redAlarm(clientp);
                        record_string("driver unable alarm",OsWrite);
                        ROS_INFO("Stateandalarm: driver unable alarm!");
                        driverEnableF=false;
                    }
                    state[37]=0x01;
                }
                else
                {
                    if(!driverEnableF)
                    {
                        red_count--;
                        greenNormal(clientp);
                        driverEnableF=true;
                        record_string("driver unable alarm clear",OsWrite);
                        ROS_INFO("Stateandalarm: driver unable alarm clear!");
                    }
                }*/

                state[62]=0x5E;     //电机电流
                state[63]=mov[29];  //左前
                state[64]=mov[30];
                state[65]=mov[31];  //右前
                state[66]=mov[32];
                state[67]=mov[33];  //左后
                state[68]=mov[34];
                state[69]=mov[35];  //右后
                state[70]=mov[36];

                int WLF=0;
                int WLB=0;
                int WRF=0;
                int WRB=0;

                //速度值
                WLF=(short)((0|mov[20])<<8)|mov[21];
                WLB=(short)((0|mov[24])<<8)|mov[25];
                WRF=(short)((0|mov[22])<<8)|mov[23];
                WRB=(short)((0|mov[26])<<8)|mov[27];

                VLF=spdfactor*WLF;
                VLB=spdfactor*WLB;
                VRF=spdfactor*WRF;
                VRB=spdfactor*WRB;

                //编码器值
                CDLF=0;CDLB=0;CDRB=0;CDRF=0;

                CDLF=(((((((CDLF&0)|mov[55])<<8)|mov[56])<<8)|mov[57])<<8)|mov[58]; //左前编码器
                CDLB=(((((((CDLB&0)|mov[63])<<8)|mov[64])<<8)|mov[65])<<8)|mov[66]; //左后编码器
                CDRF=(((((((CDRF&0)|mov[59])<<8)|mov[60])<<8)|mov[61])<<8)|mov[62]; //右前编码器
                CDRB=(((((((CDRB&0)|mov[67])<<8)|mov[68])<<8)|mov[69])<<8)|mov[70]; //右后编码器

                record_time(OsWrite2);
                OsWrite2<<" CDLF:"<<CDLF<<" CDLB:"<<CDLB<<" CDRF:"<<CDRF<<" CDRB:"<<CDRB;

                this->pubsensor();
                this->pubstate();
            }
        }
        else
            chargeBack1();
    }

    void statecallback5(const stateandalarm::state::ConstPtr& msg5)//获取路径规划信息；
    {

        char rout[3]={0,255,0};
        memcpy(rout,msg5->data.c_str(),msg5->data.size()+1);
        state[32]=0x5A;//是否到达目标点
        state[33]=rout[0];
        state[57]=0x5C;
        state[58]=rout[1];
        if(msg5->data.size()>2)
        {state[38]=0x10;} //地图崩溃

        this->pubstate();
    }

    void statecallback6(const processor::moveorder::ConstPtr& msg6)//主控板消息的回调函数
    {
        state[30]=0x59;//机器人控制模式
        if(msg6->state == "automatic")
        {
            state[31]=0x00;

        }
        else if(msg6->state == "controlled")
        {
            state[31]=0x01;

        }
        this->pubstate();
    }

    void statecallback8(const move::charge::ConstPtr& msg)//充放电消息
    {
        if(msg->chargeFlag)
        {
            state[83]=0x01;//充电状态
            chargeBack1();
            ROS_INFO("Stateandalarm: Robot is in charging state.");
        }
        else
        {
            state[83]=0x00;//放电状态
            red_count=0;
            yellow_count=0;
            ROS_INFO("Stateandalarm: Robot is working.");
        }

        is_inroom=msg->isInRoom;
        if(is_inroom) //在充电屋内
        {
            //坐标 (0,4.80,-90)
            int Xcm=0;//
            int Ycm=480;
            int thdu=-900;
            state[2]=Xcm>>24;state[3]=Xcm>>16;state[4]=Xcm>>8;state[5]=Xcm;
            state[6]=Ycm>>24;state[7]=Ycm>>16;state[8]=Ycm>>8;state[9]=Ycm;
            state[11]=thdu>>8;state[12]=thdu;
        }

        this->pubstate();
    }

    void pubstate()
    {

        state[35]=0x00;  //机器人通讯状态
        state[36]=comstate[0] | comstate[1] | comstate[2] | comstate[3] |comstate[4]|comstate[5];
        communication::state robstate;
        robstate.type=0x50;
        robstate.lenth=104;  //%修改
        robstate.data="";
        int m=0;
        for(m=0;m<robstate.lenth;m++){robstate.data+=state[m];}

        pub_state.publish(robstate);
    }

    void pubsensor()
    {
        if( (LDLF>upFlow/2)&&(CDLF<downFlow/2) )
        {
            DLF= grid*((CDLF-downFlow)+(upFlow-LDLF)+1);
        }
        else if( (LDLF<downFlow/2)&&(CDLF>upFlow/2))
        {
            DLF= -grid*((LDLF-downFlow)+(upFlow-CDLF)+1);
        }
        else
        {
            DLF=grid*(CDLF-LDLF);
        }

        if( (LDLB>upFlow/2)&&(CDLB<downFlow/2) )
        {
            DLB= grid*((CDLB-downFlow)+(upFlow-LDLB)+1);
        }
        else if( (LDLB<downFlow/2)&&(CDLB>upFlow/2))
        {
            DLB= -grid*((LDLB-downFlow)+(upFlow-CDLB)+1);
        }
        else
        {
            DLB=grid*(CDLB-LDLB);
        }

        if( (LDRF>upFlow/2)&&(CDRF<downFlow/2) )
        {
            DRF= grid*((CDRF-downFlow)+(upFlow-LDRF)+1);
        }
        else if( (LDRF<downFlow/2)&&(CDRF>upFlow/2))
        {
            DRF= -grid*((LDRF-downFlow)+(upFlow-CDRF)+1);
        }
        else
        {
            DRF=grid*(CDRF-LDRF);
        }

        if( (LDRB>upFlow/2)&&(CDRB<downFlow/2) )
        {
            DRB= grid*((CDRB-downFlow)+(upFlow-LDRB)+1);
        }
        else if( (LDRB<downFlow/2)&&(CDRB>upFlow/2))
        {
            DRB= -grid*((LDRB-downFlow)+(upFlow-CDRB)+1);
        }
        else
        {
            DRB=grid*(CDRB-LDRB);
        }
        if(first)
        {
            DLF=DLB=DRF=DRB=0;
            first=false;
        }

        pkg_msgs::MsgOdometrySensor sensor;
        sensor.type="ODOMETER";
        sensor.vlf=VLF;
        sensor.vlb=VLB;
        sensor.vrf=-VRF;
        sensor.vrb=-VRB;
        sensor.dlf=DLF;
        sensor.dlb=DLB;
        sensor.drf=-DRF;
        sensor.drb=-DRB;
        sensor.vth=0;
        sensor.th=0;
        /*if(fabs(DLF)>0.1 || fabs(DLB)>0.1 || fabs(DRF) >0.1 || fabs(DRB) >0.1)
 { OsWrite1<<"DLF:"<<DLF<<" DLB:"<<DLB<<" DRF: "<<DRF<<" DRB:"<<DRB<<endl;
   OsWrite1<<endl;
 }*/
        record_time(OsWrite1);
        OsWrite1<<" DLF:"<<DLF<<" DLB:"<<DLB<<" DRF: "<<DRF<<" DRB:"<<DRB;

        testdislf+=DLF;
        testdislb+=DLB;
        testdisrf+=DRF;
        testdisrb+=DRB;

        if(fabs(DLF)<1 && fabs(DLB)<1 && fabs(DRF)<1 && fabs(DRB)<1) //如果数据正常 则发布 并记录上一次
        {
            pub_sensor.publish(sensor);
        }

        LDLF=CDLF;
        LDLB=CDLB;
        LDRF=CDRF;
        LDRB=CDRB;
    }

    void close_ser()
    {
        record_string("close file",OsWrite);
        record_string("*******************",OsWrite);

        record_string("close file",OsWrite1);
        record_string("*******************",OsWrite1);

        record_string("close file",OsWrite2);
        record_string("*******************",OsWrite2);

        closeFile(OsWrite);//关闭文本
        closeFile(OsWrite1);//关闭文本
        closeFile(OsWrite2);//关闭文本
    }

private:
    ros::NodeHandle n;

    ros::Publisher pub_state;
    ros::Publisher pub_sensor;
    ros::Publisher pub;

    ros::ServiceClient clientp;
    ros::ServiceClient clientm;
    ros::ServiceClient clientmRound;
    ros::Subscriber sub1;
    ros::Subscriber sub2;
    ros::Subscriber sub3;
    ros::Subscriber sub4;
    ros::Subscriber sub5;
    ros::Subscriber sub6;
    ros::Subscriber sub7;
    ros::Subscriber sub8;
    ros::Subscriber sub9;
    ros::Subscriber sub10;
};

int main(int argc, char**argv)
{
    ros::init(argc, argv, "stateandalarm");

    robotState recOb;
    ros::MultiThreadedSpinner s(4);
    ros::spin(s);
    recOb.close_ser();
    return 0;

}
