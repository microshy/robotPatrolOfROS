#include "communication/SerialPort.h"

SerialPort::SerialPort(const char* pPortName)
{
    fd = -1;
    portName = pPortName;
    openState = false;
    blockState = false;
    outTime = 5;
    baud = 115200;
    dataBits = 8;
    stopBits = 1;
    parity = 'n';
    inMode = 'c';
    outMode = 'c';
}

SerialPort::~SerialPort()
{

}

void SerialPort::setID(char ID)
{
    dev_ID=ID;
}

int SerialPort::openPort()
{
    fd = open(portName,O_RDWR|O_NOCTTY|O_NDELAY);
    if(-1 == fd)
    {
        cout << "open" << portName <<"fail\n";
        return -1;
    }
    openState = true;
}

int SerialPort::openPort(int flag)
{
    fd = open(portName,flag);

    if(-1 == fd)
    {
        cout << "open" << portName << "fail\n";
        return -1;
    }
    openState = true;
}

int SerialPort::openPort(int flag,int perms)
{
    fd = open(portName,flag,perms);

    if(-1 == fd)
    {
        cout << "open" << portName << "fail\n";
        return -1;
    }
    openState = true;
}

void SerialPort::closePort()
{
    if(fd != -1 && openState == true)
    {
        close(fd);
    }
    else
    {
        cout << "no port is open!\n";
    }
    openState = false;
}

void SerialPort::setBaud(int pBaud)
{
    //save old config
    if( tcgetattr(fd,&options) != 0)
    {
        cout << "please open port first!\n";
        return;
    }

    int tmp;
    switch(pBaud)
    {
    case 4800:
        tmp = B4800;
        break;
    case 9600:
        tmp = B9600;
        break;
    case 19200:
        tmp = B19200;
        break;
    case 38400:
        tmp = B38400;
        break;
    case 57600:
        tmp = B57600;
        break;
    case 115200:
        tmp = B115200;
        break;
    default:
        tmp = -1;
        cout << "unsupported baud\n";
        return;
    }
    baud = pBaud;
    cfsetispeed(&options, tmp);
    cfsetospeed(&options, tmp);
    options.c_cflag |= (CLOCAL | CREAD);
    tcflush(fd,TCIFLUSH);	//flush
    //set immediately
    if(tcsetattr(fd,TCSANOW,&options) !=0)
    {
        cout << "set baud fail\n";
        return;
    }
}

int SerialPort::getBaud()
{
    return baud;
}

void SerialPort::setDataBits(int pDataBits)
{
    //save old options
    if( tcgetattr(fd,&options) != 0 )
    {
        cout << "please open port first\n";
        return;
    }
    //set databits
    options.c_cflag &= ~CSIZE;	//clear
    switch(pDataBits)
    {
    case 5:
        options.c_cflag |= CS5;
        break;
    case 6:
        options.c_cflag |= CS6;
        break;
    case 7:
        options.c_cflag |= CS7;
        break;
    case 8:
        options.c_cflag |= CS8;
        break;
defalut:
        cout<< "unsupported data size\n";
        return;
    }
    dataBits = pDataBits;
    //set immediately
    tcflush(fd,TCIFLUSH);	//flush
    if(tcsetattr(fd,TCSANOW,&options) != 0)
    {
        cout << "set databits fail\n";
        return;
    }
}

int SerialPort::getDataBits()
{
    return dataBits;
}

void SerialPort::setStopBits(int pStopBits)
{
    //save old options
    if( tcgetattr(fd,&options) != 0)
    {
        cout  << "please open port first\n";
        return;
    }
    //set stopbits
    switch(pStopBits)
    {
    case 1:
        options.c_cflag &= ~CSTOPB;
        break;
    case 2:
        options.c_cflag |= CSTOPB;
        break;
    default:
        cout << "unsupported stopBits\n";
        return;
    }
    stopBits = pStopBits;
    //set immediately
    tcflush(fd,TCIFLUSH);
    if(tcsetattr(fd,TCSANOW,&options) != 0)
    {
        cout << "set stopbits fail\n";
        return;
    }
}

int SerialPort::getStopBits()
{
    return stopBits;
}

void SerialPort::setParity(char pParity)
{
    //save old options
    if( tcgetattr(fd,&options) != 0)
    {
        cout << "please open port first\n";
        return;
    }
    //set parity
    switch(pParity)
    {
    case 'n':	//none
    case 'N':
        options.c_cflag &= ~PARENB;
        options.c_iflag &= ~INPCK;
        break;
    case 'o':	//odd
    case 'O':
        options.c_cflag |= (PARODD | PARENB);
        options.c_iflag |= INPCK;
        break;
    case 'e':	//even
    case 'E':
        options.c_cflag |= PARENB;
        options.c_cflag &= ~PARODD;
        options.c_iflag |= INPCK;
        break;
    case 's':	//space
    case 'S':
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        break;
    default:
        cout << "unsupported parity\n";
        return;
    }
    parity = pParity;
    //set immediately
    tcflush(fd, TCIFLUSH);
    if( tcsetattr(fd,TCSANOW,&options) != 0)
    {
        cout << "set parity fail\n";
        return;
    }
}

char SerialPort::getParity()
{
    return parity;
}

void SerialPort::setPort()
{
    setBaud(baud);
    setDataBits(dataBits);
    setStopBits(stopBits);
    setParity(parity);
}

void SerialPort::setPort(int pBaud,int pDataBits,int pStopBits,char pParity)
{
    order=0;
    receFlag=false;
    connect=false;
    setBaud(pBaud);
    setDataBits(pDataBits);
    setStopBits(pStopBits);
    setParity(pParity);
}

bool SerialPort::isOpen()
{
    return openState;
}

bool SerialPort::isBlock()
{
    return blockState;
}

void SerialPort::setBlock(bool on)
{
    int flags;
    flags = fcntl(fd,F_GETFL,0);
    if(on)
    {
        flags &= ~O_NONBLOCK;
    }
    else
    {
        flags |= O_NONBLOCK;
    }
    blockState = on;
    fcntl(fd,F_SETFL,flags);
}

void SerialPort::setOutTime(float pOutTime)
{
    //save old options
    if( tcgetattr(fd,&options) != 0)
    {
        cout << "please open port first\n";
        return;
    }
    //setOutTime
    options.c_cc[VTIME] = (int)(pOutTime*10);
    outTime = options.c_cc[VTIME]/10;
    //set immediately
    if(tcsetattr(fd, TCSANOW, &options) != 0)
    {
        cout << "set out time fail\n";
        return;
    }
}

float SerialPort::getOutTime()
{
    return outTime;
}

void SerialPort::setInMode(char mode)
{
    //save old options
    if( tcgetattr(fd,&options) != 0)
    {
        cout << "please open port first\n";
        return;
    }
    switch(mode)
    {
    case 'c':	//classic
    case 'C':
        options.c_lflag |= (ICANON | ECHO | ECHOE);
        inMode = 'c';
        break;
    case 'r':	//raw
    case 'R':
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        inMode = 'r';
        break;
    default:
        cout << "only support c/C for classic mode and r/R for raw mode\n";
        return;
    }
    //set immediately
    if(tcsetattr(fd,TCSANOW,&options)!=0)
    {
        cout << "set input mode fail\n";
        return;
    }
}

char SerialPort::getInMode()
{
    return inMode;
}

void SerialPort::setOutMode(char mode)
{
    //save old options
    if( tcgetattr(fd,&options) != 0)
    {
        cout << "please open port first\n";
        return;
    }
    switch(mode)
    {
    case 'c':	//classic
    case 'C':
        options.c_oflag |= OPOST;
        outMode = 'c';
        break;
    case 'r':	//raw
    case 'R':
        options.c_oflag &= ~OPOST;
        outMode = 'r';
        break;
    default:
        cout << "only support c/C for classic mode and r/R for raw mode\n";
        return;
    }
    //set immediately
    if(tcsetattr(fd,TCSANOW,&options)!=0)
    {
        cout << "set output mode fail\n";
        return;
    }
}

char SerialPort::getOutMode()
{	
    return outMode;
}

int SerialPort::readPort(char* buffer, int len)
{
    if(fd == -1)
    {
        cout << "please open port first\n";
        return -1;
    }
    int nBytes = read(fd,buffer,len);
    return nBytes;
}

int SerialPort::waitBack( char *data, size_t size, int timeout ) //非阻塞接受
{
    int readlen=0, fs_sel;
    fd_set fs_read;
    struct timeval tv_timeout;

    if ( fd == -1 ) //串口未被打开
        return -1;

    FD_ZERO(&fs_read);
    FD_SET(fd, &fs_read);

    if ( timeout > 0 )
    {
        tv_timeout.tv_sec = timeout; //time out : unit sec
        tv_timeout.tv_usec = 0;

        fs_sel = select( fd+1, &fs_read, NULL, NULL, &tv_timeout);
    }
    else//死等
    {
        fs_sel = select( fd+1, &fs_read, NULL, NULL, NULL );
    }

    if(fs_sel<0)
    {
        //exit(-1);
        printf("error\n");
    }
    else if(fs_sel==0)
    {
        printf("time out \n");
        connect=false;
        return(-1);
    }
    else
    {
        readlen = read(fd, data, size);
        // HexDump(data,readlen,0);
        dataProcess(data,size);
        connect=true;
    }
    flush();
    return readlen;
}

void SerialPort::sendInfo(string type ,string data) //发送指令
{
    string cmdToRoom="";
    char *sendBuf;

    //tranform into the standard command to room
    unsigned char lenth1;
    unsigned char lenth2;

    unsigned char head=0xFA;
    if(order==255){order=0x00;}else order+=1;

    int temp=data.size()+1; //command lenth
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

    //HexDump(sendBuf,out_len,0);
    int send=write(fd, sendBuf, out_len);
}  

void SerialPort::dataProcess(char *p,size_t size)
{
    string crc_data="";
    data_in="";
    unsigned char crc_get;
    unsigned char recN=0x00; //数据接受正常与否
    unsigned char getOrder;

    getOrder=*(p+1);
    //cout<<"get order"<<(int)getOrder<<"send order "<<(int)order<<endl;

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

    //if(crc_get==0xA0&&recN==0x01)
    if(crc_get==getCrc(crc_data)&&recN==0x01&&getOrder==(int)order)
    {
        printf("data is right \n");
        //cout<<endl;
        
        receFlag=true;
        //******//
        /*	char *buf;
                buf=(char *)data_in.c_str();
                int out_len=data_in.size();
                len=out_len;
                cout<<"message to processor : ";
                HexDump(buf,out_len,0);*/
        //****//
    }
    else
    {
        receFlag=false;
        printf("data is wrong \n");
        cout<<endl;
    }
    flush();
}

unsigned char SerialPort::getCrc(string values)
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

int SerialPort::readPort(string &buffer,int len){
    char* cBuffer = new char[len];
    int nBytes = readPort(cBuffer,len);
    if(nBytes == -1)
    {
        return -1;
    }
    else
    {
        buffer.insert(0,cBuffer,nBytes);
        return nBytes;
    }
}

string SerialPort::readPort2str(int len)
{
    string buffer;
    readPort(buffer,len);
    return buffer;
}

int SerialPort::readPort(vector<uchar> &buffer,int len)
{
    char* cBuffer = new char[len];
    buffer.resize(len);
    int nBytes = readPort(cBuffer,len);
    if(nBytes == -1){
        delete[] cBuffer;
        return -1;
    }
    else{
        for(int i=0;i<nBytes;i++){
            buffer[i] = (uchar)cBuffer[i];
        }
        delete[] cBuffer;
        return nBytes;
    }
}

int SerialPort::readline(char* buffer,int len,char eol)
{
    char ch=' ';
    int idx=0;
    while(ch!=eol){
        if(readPort(&ch,1)>0){
            if(ch!=eol){
                buffer[idx++] = ch;
            }
        }
        if(idx==len){
            cout << "no eol in len bytes" << endl;
            break;
        }
    }
    return idx;
}

int SerialPort::readline(string &buffer,int len,char eol)
{
    char* cBuffer = new char[len];
    int nBytes = readline(cBuffer,len,eol);
    if(nBytes == -1)
    {
        delete[] cBuffer;
        return -1;
    }
    else
    {
        buffer.insert(0,cBuffer,nBytes);
        delete[] cBuffer;
        return nBytes;
    }
}

string SerialPort::readline(int len,char eol)
{
    string buffer;
    readline(buffer,len,eol);
    return buffer;
}

int SerialPort::writePort(const char* buffer, int len)
{
    if(fd == -1)
    {
        cout << "please open port first\n";
        return -1;
    }
    int nBytes = write(fd,buffer,len);
    return nBytes;
}

int SerialPort::writePort(const string &buffer)
{
    if(fd == -1){
        cout << "please open port first\n";
        return -1;
    }
    for(int i=0;i<buffer.size();i++){
        write(fd,&buffer[i],1);
    }
    return buffer.size();
}

int SerialPort::writePort(const vector<uchar> &buffer)
{
    if(fd == -1){
        cout << "please open port first\n";
        return -1;
    }
    for(int i=0;i<buffer.size();i++){
        write(fd,&buffer[i],1);
    }
    return buffer.size();
}

void SerialPort::flush()
{
    tcflush(fd,TCIOFLUSH);
}

void SerialPort::inFlush()
{
    tcflush(fd,TCIFLUSH);
}

void SerialPort::outFlush()
{
    tcflush(fd,TCOFLUSH);
}

void SerialPort::HexDump(char *buf,int len,int addr)
{
    int i,j,k;
    char binstr[80];

    for (i=0;i<len;i++)
    {
        if (0==(i%16))
        {
            sprintf(binstr,"%08x -",i+addr);
            sprintf(binstr,"%s %02x",binstr,(unsigned char)buf[i]);
        } else if (15==(i%16)) {
            sprintf(binstr,"%s %02x",binstr,(unsigned char)buf[i]);
            sprintf(binstr,"%s  ",binstr);
            for (j=i-15;j<=i;j++)
            {
                //       sprintf(binstr,"%s%c",binstr,('!'<buf[j]&&buf[j]<='~')?buf[j]:'.');
            }
            printf("%s\n",binstr);
        }
        else
        {
            sprintf(binstr,"%s %02x",binstr,(unsigned char)buf[i]);
        }
    }
    if (0!=(i%16))
    {
        k=16-(i%16);
        for (j=0;j<k;j++)
        {
            sprintf(binstr,"%s   ",binstr);
        }
        sprintf(binstr,"%s  ",binstr);
        k=16-k;
        for (j=i-k;j<i;j++)
        {
            //     sprintf(binstr,"%s%c",binstr,('!'<buf[j]&&buf[j]<='~')?buf[j]:'.');
        }
        printf("%s\n",binstr);
    }
}
