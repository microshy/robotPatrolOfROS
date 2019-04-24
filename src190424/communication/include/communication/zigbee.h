#ifndef ZIGBEE_H
#define ZIGBEE_H

#include <boost/asio.hpp>                  //包含boost库函数
#include <boost/bind.hpp>
#include <communication/display.h>
#include <string>
#include <iostream>

using namespace std;
using namespace boost::asio;


class Zigbee
{
public:
    Zigbee(string portName,char ID);
    ~Zigbee();
    void sendInfo(string type ,string data); //发送数据
    void setSeial(int baud,int stopBit,char parity,int dataSize);
    void waitBack(char *p,size_t size);
    void dataProcess(char *p,size_t size);
    void asyncRead(char *p,size_t size); //异步read函数
    void read_callback(char *p,size_t size,boost::system::error_code ec);//异步读取回调函数
    void call_read();//执行回调函数
    unsigned char getCrc(string values);

private:
    boost::system::error_code ec;
    unsigned char order; //zhen xuhao
    char dev_ID;

public:
    string data_in;
    string type;
    serial_port *pSerialPort;
    io_service m_ios;
    bool receFlag;
    int len;
};

Zigbee::Zigbee(string portName,char ID )
{
    pSerialPort=new serial_port(m_ios);
    pSerialPort->open(portName,ec);
    dev_ID=ID;
    printf("open the port\n");
}

Zigbee::~Zigbee()
{
    if(pSerialPort)
    {
        delete pSerialPort;
    }
}

void Zigbee::setSeial(int baud,int stopBit,char parity,int dataSize)
{
    order=0;
    receFlag=true;

    pSerialPort->set_option(serial_port::baud_rate(baud));
    pSerialPort->set_option(serial_port::flow_control( serial_port::flow_control::none));
    pSerialPort->set_option(serial_port::character_size(dataSize));
    switch(parity)
    {
    case 'n':
    case 'N':
        pSerialPort->set_option(serial_port::parity(serial_port::parity::none));
        break;
    case 'o':
    case 'O':
        pSerialPort->set_option(serial_port::parity(serial_port::parity::odd));
        break;
    case 'e':
    case 'E':
        pSerialPort->set_option(serial_port::parity(serial_port::parity::even));
        break;
    default :
        printf("NO parity supported\n");
        return ;
    }

    switch(stopBit)
    {
    case 1:
        pSerialPort->set_option(serial_port::stop_bits(serial_port::stop_bits::one));
        break;
    case 2:
        pSerialPort->set_option(serial_port::stop_bits(serial_port::stop_bits::two));
        break;
    default :
        printf("NO stopBit supported\n");
        return ;
    }
}

void Zigbee::sendInfo(string type ,string data)
{
    string cmdToRoom="";
    char *sendBuf;

    //tranform into the standard command to room
    unsigned char lenth1;
    unsigned char lenth2;

    unsigned char head=0xFA;
    if(order==255){order=0x00;}else order+=1;

    int temp=data.size()+1; //zhiling changdu
    if(temp>255)
    {
        lenth1=temp-255;
        lenth2=255;
    }
    else
    {
        lenth1=0;
        lenth2=temp;
    }

    cmdToRoom+=head;cmdToRoom+=order;cmdToRoom+=dev_ID;cmdToRoom+=lenth1;cmdToRoom+=lenth2;
    cmdToRoom+=type;
    cmdToRoom+=data;

    unsigned char crc=getCrc(cmdToRoom);
    unsigned tail=0xFF;

    cmdToRoom+=crc;
    cmdToRoom+=tail;

    sendBuf=(char *)cmdToRoom.c_str();
    int out_len=cmdToRoom.size();

    cout<<"send messaage: ";
    HexDump(sendBuf,out_len,0);
    int send=write(*pSerialPort, buffer(sendBuf, out_len));
}     

void Zigbee::waitBack(char *p,size_t size) //阻塞模式，直到接收到指定长度字符才结束。
{
    int readByte=0;
    readByte=read (*pSerialPort,buffer(p,size));
    cout<<"receive messaage: ";
    HexDump(p,size,0);
    dataProcess(p,size);
}

void Zigbee::dataProcess(char *p,size_t size)
{
    string crc_data="";
    data_in="";
    unsigned char crc_get;
    unsigned char recN=0x00; //数据接受正常与否

    for (int i=0;i<size-1;i++)
    {
        if(i<size-2)
        {
            crc_data+=*(p+i); //校验数据
        }
        else
        {
            crc_get=*(p+i); //CRC值
        }

        if(5<i&&i<size-2)
        {
            if(i==6)
            {
                recN=*(p+i);
            }
            data_in+=*(p+i); //数据域
        }
    }

    if(crc_get==getCrc(crc_data)&&recN==0x01)
        //if(crc_get==0xA0&&recN==0x01)
    {
        printf("data is right \n");
        receFlag=true;
        //******//
        char *buf;
        buf=(char *)data_in.c_str();
        int out_len=data_in.size();
        len=out_len;
        cout<<"message to processor : ";
        HexDump(buf,out_len,0);
        //****//
    }
    else receFlag=false;
}

void Zigbee::read_callback(char *p,size_t size,boost::system::error_code ec)
{
    cout<<"receive messaage: ";
    HexDump(p,size,0);
    dataProcess(p,size);
}

void Zigbee::asyncRead(char *p,size_t size) //异步读取有点问题。待解决
{
    async_read(*pSerialPort,buffer(p,size),boost::bind(&Zigbee::read_callback,this,p,size,_1));
}

void Zigbee::call_read()
{
    m_ios.run();
    m_ios.reset();
}

unsigned char Zigbee::getCrc(string values)
{
    unsigned char crc_array[256] = {
        0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83,
        0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
        0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e,
        0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
        0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0,
        0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
        0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d,
        0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
        0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5,
        0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
        0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58,
        0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
        0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6,
        0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
        0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b,
        0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
        0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f,
        0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
        0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92,
        0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
        0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c,
        0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
        0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1,
        0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
        0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49,
        0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
        0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4,
        0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
        0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a,
        0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
        0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7,
        0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
    };
    unsigned char crc8 = 0;
    for (int i = 0; i < values.size(); i++)
    {
        crc8 = crc_array[crc8^(unsigned char)(values[i])];
    }
    return crc8;
}

#endif
