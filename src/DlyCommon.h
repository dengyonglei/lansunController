/*
 * DlyCommon.h
 *
 *  Created on: 2016年12月4日
 *      Author: 邓永雷
 */

#ifndef LANSUNV2_0_SRC_DLYCOMMON_H_
#define LANSUNV2_0_SRC_DLYCOMMON_H_
#include <iostream>
#include <sys/time.h>
#include <string>
#include <vector>
#include "common.h"
using namespace std;
class DylCommon
{
public:
	DylCommon();
	virtual ~DylCommon();
	static int conn;
	static int udp;  //套接字
	static int addr_len;  //长度
	static bool IsOpenUdp;
	//时间测量
	static void setConn(int n)
	{
		conn = n;
	}
	static int time_substract(struct timeval *result, struct timeval *begin,struct timeval *end);
	static void protocol_send(const string &str);
	static void protocol_send1(const string &str);
	static void protocol_send2(const string &str);
	static vector<string> split( string str, string pattern);
	static void getCurrentPosition(Joint&curJ,Coint &curC);
	static bool IsCoordEqual(Joint curJ ,Joint lastcurJ,Coint curC,Coint  lastcurC);

};



#endif /* LANSUNV2_0_SRC_DLYCOMMON_H_ */
