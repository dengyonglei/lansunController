/*
 * udp.cpp
 *
 *  Created on: 2017年5月9日
 *      Author: deng
 */

#include "udp.h"
#include <unistd.h>

#include <sys/socket.h>
#include <arpa/inet.h>
 string udp::udpAddress = "192.168.1.101";
 int  udp::udpsocket = 0;              //套接字
 int  udp::addr_len = sizeof(sockaddr_in);  //长度
 bool udp::IsOpenUdp = false;       //是否打开udp通信
 bool udp::IsBackCancle = false;    //是否回零取消 便于仿真
 sockaddr_in udp::udpaddr;    //udp的套接字
udp::udp()
{
  // TODO Auto-generated constructor stub
}

udp::~udp()
{
	// TODO Auto-generated destructor stub
}

void udp::initUdpSocket()
 {
	 if(!IsOpenUdp)
		 return;
	 /* 建立socket*/
	 if((udpsocket = socket(AF_INET,SOCK_DGRAM,0))<0)   //初始化socket
	 {
		 perror("socket");
		 exit(1);
	 }
	 else
	 {
		 cout << "udp初始化成功" << endl;
	 }
	 bzero(&udpaddr,sizeof(udpaddr));
	 udpaddr.sin_family = AF_INET;
	 udpaddr.sin_port = htons(UdpPort);
	 udpaddr.sin_addr.s_addr = inet_addr(udpAddress.c_str());
//	 udpaddr.sin_addr.s_addr = htonl(INADDR_ANY);
//	 if (bind(udp, (struct sockaddr *)&udpaddr, sizeof(udpaddr)) < 0)
//	 {
//		 perror("bind");
//		 exit(1);
//	 }
//	 else
//	 {
//		 cout << "udp绑定成功" << endl;
//	 }
//	     char buff[512];
//	     struct sockaddr_in clientAddr;
//
//	     unsigned len = sizeof(clientAddr);
//	     int n = recvfrom(udp, buff, 511, 0, (struct sockaddr*)&clientAddr, &len);
//		if (n>0)
//		{
//			buff[n] = 0;
//			printf("upd接收的数据是：%s %u says: %s\n", inet_ntoa(clientAddr.sin_addr), ntohs(clientAddr.sin_port), buff);
//			n = sendto(udp, buff, n, 0, (struct sockaddr *)&clientAddr, sizeof(clientAddr));
//			if (n < 0)
//			{
//				perror("udp接受失败1");
//
//			}
//		}
//		else
//		{
//			    perror("udp接收失败2");
//		}
 }

 void udp::sendProgramData(const string &str)
 {
	 if(IsOpenUdp)
	 sendto(udpsocket,str.c_str(),str.size(),0,(sockaddr *)&udpaddr,addr_len);
 }
 void udp::closeUdp()
 {
	 if(udp::IsOpenUdp)
     close(udpsocket);
 }
