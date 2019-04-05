#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <string>
#include <vector>
#include <fstream>  

using namespace std;



typedef unsigned char uchar;
using std::vector;
using std::string;
using std::cout;
using std::endl;

class SerialPort
{
private: 
		
	const char* portName;	
	bool openState;
	bool blockState;
	float outTime;
	int baud;
	int dataBits;
	int stopBits;
	char parity;
	char inMode;
	char outMode;
	struct termios options;
	
	unsigned char order; //zhen xuhao
	char dev_ID;
        
	
public:
        int fd;	
	SerialPort(const char* pPortName);
	~SerialPort();
	void setID(char ID);
	int openPort();
	int openPort(int flag);
	int openPort(int flag,int perms);
	void closePort();
	
	void setBaud(int pBaud);
	int getBaud();
	void setDataBits(int pDataBits);
	int getDataBits();
	void setStopBits(int pStopBits);
	int getStopBits();
	void setParity(char pParity);
	char getParity();
	void setPort();
	void setPort(int pBaud,int pDataBits,int pStopBits,char pParity);
	
	bool isOpen();
	bool isBlock();
	void setBlock(bool on);
	void setOutTime(float pOutTime);
	float getOutTime();

	//@mode:'c'-classic mode
	//     'r'-raw mode
	void setInMode(char mode);
	char getInMode();
	void setOutMode(char mode);
	char getOutMode();

	int readPort(char* buffer,int len);
	int readPort(string &buffer,int len=100);
	string readPort2str(int len=100);
	int readPort(vector<uchar> &buffer,int len=100);
	int readline(char* buffer,int len,char eol='\n');
	int readline(string &buffer,int len=100,char eol='\n');
	string readline(int len=100,char eol='\n');
	
	int waitBack( char *data, size_t size, int timeout );
	
	int writePort(const char* buffer,int len);
	int writePort(const string &buffer);
	int writePort(const vector<uchar> &buffer);
    void HexDump(char *buf,int len,int addr) ;

	void flush();
	void inFlush();
	void outFlush();
	void sendInfo(string type ,string data); //发送指令
	void dataProcess(char *p,size_t size);
	unsigned char getCrc(string values);
	
	
	
	//数据
	string data_in;
	string type;
	bool receFlag;
	int len;
	bool connect;
};


#endif
