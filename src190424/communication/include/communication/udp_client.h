#ifndef UDP_CLIENT_H
#define UDP_CLIENT_H

#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <fcntl.h>

#include <cstring>
#include <string>
#include <iostream>
#include <ros/ros.h>
#include <communication/command.h>
#include <sys/select.h>
#include <fstream>
using namespace std;

class UDP_Client
{

public:
    //open udp
    bool open();
    /* initial the socket server;
                parameter: PORT
        */
    void init(int port,string ip,char *rec,string ID,const char *file);
    void sendInfo(string type ,string data); //发送数据

    void receive(int max);
    void close_ser();

    unsigned char getCrc(string values);
    void wait_back(int maxsize,int time,int time_ms=0); //接受数据

private:
    int sockfd;

    char *recBuf;
    struct sockaddr_in client;
    unsigned char order; //zhen xuhao
    bool connFlag;
    string dev_ID;
    ofstream OsWrite;
    const char *logfile;

public:
    string data_in;
    string type;

    bool receFlag;
    bool connect;

    int len;

};

#endif
