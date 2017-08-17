/*
 * Coordinates.cpp
 *
 *  Created on: 2016年11月17日
 *      Author: Administrator
 */

#include "Coordinates.h"
#include "moto.h"
#include <iostream>
using namespace std;
#include <stdio.h>
#include <string>
#include "robot_class/udp.h"
//初始化列表
Coordinates::Coordinates()
{
	lastcurJ={1.112,1.32,1.111,1.121,1.111,1.111};
	lastcurC={1.22,1.11};
}

Coordinates::~Coordinates()
{


}
void Coordinates::getPosition()
{
	DylCommon::getCurrentPosition(curJ,curC);                 //得到当前的坐标
	if(DylCommon::IsCoordEqual(curJ,lastcurJ,curC,lastcurC))   //判断坐标是否相等
		return;
	lastcurJ=curJ;
	lastcurC=curC;
	xyzrpw = pose_2_xyzrpw(fksolution(curJ));
	ostringstream os;
	os.clear();
	string str = "D0,11";
	for (int i = 0; i < 6; i++)
	{
		os.str("");
		os << xyzrpw[i];
		str.append(",").append(os.str());
	}
	os.str("");
	os << curC.c1 * 180 / pi;
	str.append(",").append(os.str());
	os.str("");
	os << curC.c2 * 180 / pi;
	str.append(",").append(os.str());
	DylCommon::protocol_send1(str);      //回传坐标值
//	cout << str << endl;
	if(udp::IsOpenUdp)   //开启udp模式才回传坐标值
	{
		char jdata[1000];
		sprintf(jdata,"UC,16,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",xyzrpw[0],xyzrpw[1],xyzrpw[2],xyzrpw[3],xyzrpw[4],xyzrpw[5],curJ.j1,curJ.j2,curJ.j3,curJ.j4,curJ.j5,curJ.j6,curC.c1,curC.c2);
		string strJData(jdata);
		udp::sendProgramData(strJData);
	}
}
//坐标数据回传
void Coordinates::run()
{
    getPosition();
    if(udp::IsOpenUdp)
    usleep(50000);     //两秒钟回传一次
    else
    usleep(400000);
}
