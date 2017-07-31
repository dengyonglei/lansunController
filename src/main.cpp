/*
 *  main.cpp
 *
 *  Created on: 2016年5月16日
 *      Author: Yin
 */

//AVO->field.ch=0;  指定通道0-3
//AVO->field.data=0x123      0-0x3FF 1-10v电压数据
#include "hps_0.h"                    //hps_0.h
#include "common.h"                   //
#include "moto.h"                      //
#include "DlyCommon.h"
using namespace std;
#include <pthread.h>
#include <semaphore.h>
#include <math.h>
#include "Parser.h"
#include "Parameter.h"
#include "robot_class/udp.h"
#include <time.h>
// debug
#include "FixedLengthMoving.h"
volatile unsigned long h2p_lw_led_addr;
volatile unsigned long h2p_lw_key_addr;      //灯
volatile unsigned long h2p_lw_mot_addr;
volatile unsigned long h2p_lw_IOM_addr;     //IO口
volatile unsigned long h2p_lw_AVO_addr;    //电流电压口
//==========================每1mm离散点个数
//extern float RESOLUTION;
extern float RESOLUTION;
double J1PUPR = 3746.0317460317460317460317460317;
double J2PUPR = 2285.7142857142857142857142857143;
double J3PUPR = 1591.5494309189533576888376337251;
double J4PUPR = 1111.1111111111112;
double J5PUPR = 1111.1111111111112;
double J6PUPR = 1111;
double  J7PUPR = 1000;
double  J8PUPR = 1000;
RobotType robotType = CoordRobot;


Joint lastJ = {0,0,0,0,0};
bool lastJIsinit = false;
Matrix4d TCFmatrix;
pthread_mutex_t mutex;
Parser parser;  //解析类
Parameter structParameter;      //结构参数
string lastcmd;   //保存上一条指令
Coint currentC = { 0, 0 };
int conn;   //端口号
sem_t sem;                //信号量
char sendbuf[BUFFER_SIZE];        //发送缓冲
char recvbuf[BUFFER_SIZE];        //接受缓冲
int recvlen;
int socket_descriptor;

unsigned long int vddTable[VADDTIMES];     //速度列表
string joinstr = "";
unsigned long int nowSpeed = VVVSSS;                   //1000步
//数据拆分
void splitStr(char *s, list<string> &strArray);
//数据拼接
void joinStr(vector<string>& strArray);
int server_sockfd;
//得到数据并解析



void getDateParser();
//函数运行
void run();
//多线程函数
void *thread_function0(void *arg);
void *thread_function1(void *arg);
void *thread_function2(void *arg);

int main()
{

	version = "V2017 07 31 -1";	//版本号需要自己设定
	// 初始化系统信息
	initSystemState();


	dh =  0;
	TCFmatrix << 1, 0, 0,0,
				0, 1, 0, 0,
				0, 0, 1, 0,
				0, 0, 0, 1;
	POSITIONER << 1, 0, 0, 0,
			      0, 1, 0, 0,
			      0, 0, 1, 0,
			      0, 0, 0, 1;
	if(udp::IsOpenUdp)
		{
			if(robotType == CoordRobot)
			{
			 TCFmatrix << 1, 0, 0,220,
						0, 1, 0, -5,
						0, 0, 1, -170,
						0, 0, 0, 1;
			}
			else
			{
				POSITIONER <<  1, 0, 0, 700,
							   0, 1, 0, -600,
							   0, 0, 1, 0,
							   0, 0, 0, 1;
				dh =  233.924;
			}
			}
	RESOLUTION = 10; 	//表示离散精度为0.1mm
	RESOLUTION_ATT = 20;  //表示姿态离散精度为1/30
	  float JAAA_min[6] = {-17000000.0, -17000000.0, -5000, -3600.0 / 180 * pi, -3600.0 / 180 * pi, -3600.0 / 180 * pi};
	  float JAAA_max[6] = { 17000000.0,  17000000.0,   5000,  3600.0 / 180 * pi, 3600.0 / 180 * pi,  3600.0 / 180 * pi};
	for(int i = 0; i < 6; i++)
	{
		 JA_min[i] = JAAA_min[i];
		 JA_max[i] = JAAA_max[i];
	}
	////==========================关节机械手限制角度
	//float JA_min[6] = {-170.0/180*pi, -170.0/180*pi, -500, -170.0/180*pi, -90.0/180*pi, -170.0/180*pi};
	//float JA_max[6] = { 170.0/180*pi,  170.0/180*pi,   10,  170.0/180*pi,  90.0/180*pi,  170.0/180*pi};
	//
	////==========================变位机构限制角度
	//float CA_min[2] = {-60.0/180*pi, -170.0/180*pi};
	//float CA_max[2] = { 60.0/180*pi,  170.0/180*pi};

	void *virtual_base;
	int fd;
// 打开内存映射设备驱动
	if ((fd = open("/dev/mem", ( O_RDWR | O_SYNC))) == -1) //打开映射驱动
	{
		printf("ERROR: could not open \"/dev/mem\"...\n");
		return (1);
	}

//映 射物理地址到用户空间
	virtual_base = mmap( NULL, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE),
			MAP_SHARED, fd, HW_REGS_BASE); //映射到内存
	if (virtual_base == MAP_FAILED) {
		printf("ERROR: mmap() failed...\n");
		close(fd);
		return (1);
	}

//soc硬件
	h2p_lw_led_addr = (unsigned long) virtual_base
			+ ((unsigned long) ( ALT_LWFPGASLVS_OFST + CUSTOM_LEDS_0_BASE)
					& (unsigned long) ( HW_REGS_MASK));
	h2p_lw_key_addr = (unsigned long) virtual_base
			+ ((unsigned long) ( ALT_LWFPGASLVS_OFST + BUTTON_PIO_BASE)
					& (unsigned long) ( HW_REGS_MASK));
	h2p_lw_mot_addr = (unsigned long) virtual_base
			+ ((unsigned long) ( ALT_LWFPGASLVS_OFST + ROBOT_0_BASE)
					& (unsigned long) ( HW_REGS_MASK));
	h2p_lw_IOM_addr = (unsigned long) virtual_base
			+ ((unsigned long) ( ALT_LWFPGASLVS_OFST + IO_MOD_V2_0_BASE)
					& (unsigned long) ( HW_REGS_MASK));
	h2p_lw_AVO_addr = (unsigned long) virtual_base
			+ ((unsigned long) ( ALT_LWFPGASLVS_OFST + AVO_MOD_0_BASE)
					& (unsigned long) ( HW_REGS_MASK));


//***********************************************
	IOM->DATA=0xFFFFFFFF;//关闭点火氧气等
	moto_init();//
	getVList(vddTable);//
	udp::initUdpSocket();
//***********************************************



	//=============================Socket_init
	server_sockfd = socket(AF_INET, SOCK_STREAM, 0);
	struct sockaddr_in server_sockaddr;
	server_sockaddr.sin_family = AF_INET;
	server_sockaddr.sin_port = htons(PORT);
	server_sockaddr.sin_addr.s_addr = htonl(INADDR_ANY);

	if (bind(server_sockfd, (struct sockaddr *) &server_sockaddr,
			sizeof(server_sockaddr)) < 0)
	{
		perror("bind");
		exit(1);
	}

	if (listen(server_sockfd, 10) < 0)
	{
		perror("listen");
		exit(1);
	}

	struct sockaddr_in ckient_addr;
	socklen_t length = sizeof(ckient_addr);
	cout << "等待监听中。。。呵呵" << endl;

	conn = accept(server_sockfd, (struct sockaddr *) &ckient_addr, &length);
	if (conn < 0)
	{
		perror("connect");
		exit(1);
	} else
		cout << "与安卓系统连接成功" << endl;
	DylCommon::setConn(conn);
	int res;
	pthread_t a_thread[3];
	void *thread_result;

// 创建信号量
	res = pthread_mutex_init(&mutex, NULL);
	if (res != 0)
	{
		perror("Mutex init failed!");
		exit(EXIT_FAILURE);
	}

// 创建信号量
	res = sem_init(&sem, 0, 0);
	if (res != 0)
	{
		perror("Sem init failed!");
		exit(EXIT_FAILURE);
	}

//创建线程
	res = pthread_create(&a_thread[0], NULL, thread_function0, (void *) 0);
	res = pthread_create(&a_thread[1], NULL, thread_function1, (void *) 1);
	res = pthread_create(&a_thread[2], NULL, thread_function2, (void *) 2);
	if (res != 0)
	{
		perror("Thread creation failed!");
		exit(EXIT_FAILURE);
	}

//线程
	while (1)   //主线程运动
	{
	  getDateParser();
	}
    sem_post(&sem);
    sleep(1);
	printf("\nWaiting for thread to finish...\n");
	close(conn);
	close(server_sockfd);
	res = pthread_join(a_thread[0], &thread_result);
	res = pthread_join(a_thread[1], &thread_result);
	res = pthread_join(a_thread[2], &thread_result);
	if (res != 0)
	{
		perror("Thread join failed!\n");
		exit(EXIT_FAILURE);
	}

	printf("Thread joined\n");

	res = pthread_mutex_destroy(&mutex);
	if (res != 0) {
		perror("Thread destroy failed!\n");
		exit(EXIT_FAILURE);
	}

// 清除内存映射
	if (munmap(virtual_base, HW_REGS_SPAN) != 0)
	{
		printf("ERROR: munmap() failed...\n");
		close(fd);
		return (1);
	}

// 关闭设备驱动
	close(fd);
	udp::closeUdp();
	return 0;
}


//分割套接的数据
void splitStr(char *s, list<string> &strArray)
{
	const char *d1 = "\n";
	char *p = strtok(s, d1);
	while (p)
	{
		strArray.push_back(p);
		p = strtok(NULL, d1);
	}

}



//拼接数据
void joinStr(vector<string>& strArray)
{
	//拼接   数据发送多可能出现拆分情况
	if (joinstr != "")
	{
		string begin = *strArray.begin();
		if (begin[0] != '[')
			*strArray.begin() = joinstr + *(strArray.begin());
		joinstr = "";
	}
	if (strArray.size() > 0)
	{
		string end = *(--strArray.end());
		if (*(--end.end()) != ']')
		{
			joinstr = end;
			strArray.erase(--strArray.end());
		}
	}
}


//*****************接受数据数据*******************//
void getDateParser()  //
{
	while (1)
	{
		memset(recvbuf, 0, sizeof(recvbuf));
		recvlen = recv(conn, recvbuf, sizeof(recvbuf), 0); //接收到的函数
		if (recvlen <= 0)
		{
		  if (errno == EINTR)
		  {

		  }
		  else
			  exit(1);
		}
		string str(recvbuf);
		vector<string> strArray = DylCommon::split(str, "\n");
		joinStr(strArray); //拼接
		for (vector<string>::iterator it = strArray.begin();it != strArray.end(); it++)
		{
			if (*it == "\n" || *it == "")
				continue;
			parser.DateParser(*it); //数据解析
		}

	}

}
//运动的运行入口
void run()
{
	while (1)
	{
		parser.run();
		usleep(10000);
	}

}

//****************主运动***************//
void *thread_function0(void *arg)
{
	    run();
		return 0;

}
//*********遥杆运动********************/
void *thread_function1(void *arg)
{
	while (1)
		{
			parser.roctorbar.RoctorRun();
			usleep(10000);
		}
		return 0;
}
//此线程就一直给上位机回传坐标
void *thread_function2(void *arg)
{

    double count = 0;
	while (1)
	{
		//回零完成才上传给上位机坐标
		if (parser.back_finished)
		{
			if (!parser.welding.runing || udp::IsOpenUdp)   //不在焊接过程中是不执行坐标的
			{
				parser.coord.run();   //上传坐标参数
				if(!parser.IsSend )
				count += 2;          //说明是两秒钟
			}

		}
		if(parser.IsSendIOdata)
		parser.ioparameter.run();   //上传IO口数据
		if(!udp::IsOpenUdp)
		{
		usleep(100000);
		if(!parser.IsSend)
		count ++;
		if(count >= 50)  //5s之后检测是否是连接的状态
		{
			pthread_mutex_lock (&mutex);
			if(parser.IsConnnect)  //表示处于连接状态
				parser.IsConnnect = false;
			else
			{
				if(!Variable::IsStop)
				{
					Variable::IsStop = true;
				  IOM->DATA=0xFFFFFFFF;
				  cout <<"心跳中断   运动停止" << endl;
				}
			}
			pthread_mutex_unlock(&mutex);
			count = 0; //数据清零
		}
		}
	}
	return 0;
}


