#ifndef RECORD_H
#define RECORD_H

#include <fstream>   
#include <iostream>  
#include <string>
#include <sys/time.h>
#include <stdio.h> 
#include <sys/stat.h>
#include <unistd.h>
#include <time.h>

using namespace std;

void openFile(const char* file,ofstream &OsWrite)
{

   OsWrite.open(file,std::ofstream::app);
   if(OsWrite.is_open())
   {
       //cout<<"open file success"<<endl;
   } 
   else
   {
       cout<<"open file error"<<endl;
   }
}
 
string string_to_hex(const string& str,ofstream &OsWrite) //transfer string to hex-string  
{  
    string result;  
    string tmp;  
    stringstream ss;  
    for(int i=0;i<str.size();i++)  
    {  
        ss<<hex<<(short int)str[i]<<endl;  
        ss>>tmp;  
        result+="0x";
        result+=tmp;  
        result+=" ";  
    }  
    time_t t; 
    time(&t); 
    OsWrite<<ctime(&t)<<result;
    OsWrite<<endl;
    return result;  
}  

void closeFile(ofstream &OsWrite)
{

   if(OsWrite.is_open())
   {
       OsWrite.close();  
       //cout<<"close file success"<<endl;
   } 
   else
   {
       cout<<"no open file!"<<endl;
   }
}

void record_string(const string& str,ofstream &OsWrite) //transfer string to hex-string  
{   
    time_t t; 
    time(&t); 
    OsWrite<<ctime(&t)<<str;
    OsWrite<<endl;  
}  

#endif
