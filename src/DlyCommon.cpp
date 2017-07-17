/*
 * DlyCommon.cpp
 *
 *  Created on: 2016年12月4日
 *      Author: 邓永雷
 */
#include "DlyCommon.h"
#include <sys/time.h>
#include "moto.h"
#include <sys/select.h>
#include <sys/select.h>
#include <stdlib.h>
using namespace std;
extern string  lastcmd;//上一条指令
int DylCommon::conn = 0;
DylCommon::DylCommon()
{
	// TODO Auto-generated constructor stub
}

DylCommon::~DylCommon()
{
	// TODO Auto-generated destructor stub
}
//求出分割时间的函数
int DylCommon::time_substract(struct timeval *result, struct timeval *begin,struct timeval *end)
{
	if(begin->tv_sec > end->tv_sec)
		return -1;
	if((begin->tv_sec == end->tv_sec) && (begin->tv_usec > end->tv_usec))
		return -2;
	result->tv_sec = (end->tv_sec - begin->tv_sec);
	result->tv_usec = (end->tv_usec - begin->tv_usec);
	if(result->tv_usec < 0)
	{
	result->tv_sec--;
	result->tv_usec += 1000000;
	}
	return 0;

}

//发送坐标值的函数
void DylCommon::protocol_send(const string &str)
{
	MD5 md5;
	md5.reset();
	md5.update(str);
	string ss = "[" + str + "," + md5.toString() + "]\n";
	send(conn,ss.c_str(),ss.size(),0);
	lastcmd=ss;
}


//发送坐标值的函数但是不保存到最后的代码中
void DylCommon::protocol_send1(const string &str)
{
	MD5 md5;
	md5.reset();
	md5.update(str);
	string ss="["+str+","+md5.toString()+"]\n";
	send(conn,ss.c_str(),ss.size(),0);
}

void DylCommon::protocol_send2(const string &str)
{
	send(conn,str.c_str(),str.size(),0);
}
//字符串分割函数
 vector<string> DylCommon::split( string str, string pattern)
  {
      string::size_type pos;
      vector<string> result;
      str += pattern;//扩展字符串以方便操作
      int size = str.size();
     for(int i=0; i < size; i++)
     {
        pos=str.find(pattern,i);
        if((int)pos < size)
        {
            string s = str.substr(i,pos-i);
             result.push_back(s);
             i=pos+pattern.size()-1;
        }
     }
     return result;
  }

 void  DylCommon::getCurrentPosition(Joint &curJ,Coint &curC)
 {
            if(robotType == CoordRobot)
            {
				curJ.j1 = (double)(MOT->J1step) / J1PUPR;
				curJ.j2 = (double)(MOT->J2step) / J2PUPR;
            }
            else
            {
            	curJ.j1 = (double)(MOT->J1step) * pi / 180.0 / J1PUPR;
            	curJ.j2 = (double)(MOT->J2step) * pi / 180.0 / J2PUPR;
            }
			curJ.j3 = (double)(MOT->J3step) / J3PUPR;
			curJ.j4 = (double)(MOT->J4step) * pi / 180.0 /J4PUPR;
			curJ.j5 = (double)(MOT->J5step) * pi / 180.0 /J5PUPR;
			curC.c1 = (double)(MOT->J7step) * pi / 180.0 /J7PUPR;
			curC.c2 = (double)(MOT->J8step) * pi / 180.0/ J8PUPR;

 }

 bool DylCommon::IsCoordEqual(Joint curJ ,Joint lastcurJ,Coint curC,Coint  lastcurC)
 {
	 if(fabs(curJ.j1-lastcurJ.j1)>0.009||fabs(curJ.j2-lastcurJ.j2)>0.009||fabs(curJ.j3-lastcurJ.j3)>0.009||fabs(curJ.j4-lastcurJ.j4)>0.009
	     		 ||fabs(curJ.j5-lastcurJ.j5)>0.009||fabs(curJ.j6-lastcurJ.j6)>0.009||fabs(curC.c1-lastcurC.c1)>0.009||fabs(curC.c2-lastcurC.c2)>0.009)
	     	return false;
	  else
	     	return true;
 }
