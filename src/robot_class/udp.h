/*
 * udp.h
 *
 *  Created on: 2017年5月9日
 *      Author: deng
 */

#ifndef ROBOT_CLASS_UDP_H_
#define ROBOT_CLASS_UDP_H_
//udp类的对象
#define UdpPort 8080
#include <string>
#include <string.h>
#include <iostream>
#include <netinet/in.h>
using namespace std;
class udp
{
public:
	udp();
	virtual ~udp();
	static string udpAddress;
	static int udpsocket;          //套接字
	static int addr_len;           //长度
	static bool IsOpenUdp;         //是否打开udp通信
	static bool IsBackCancle;      //是否回零取消 便于仿真
	static sockaddr_in udpaddr;    //udp的套接字
	static void initUdpSocket();   //初始化udp
	static void sendProgramData(const string &str);//发送报文
    static void closeUdp();       //关闭udp;
};

#endif /* ROBOT_CLASS_UDP_H_ */
