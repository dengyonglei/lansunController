/*
 * common.h
 *
 *  Created on: 2016年5月17日
 *      Author: Yin
 */

#ifndef LANSUNV2_0_SRC_COMMON_H_
#define LANSUNV2_0_SRC_COMMON_H_


using namespace std;

#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <sys/time.h>
#include <sys/select.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>
#include <sstream>

#include <time.h>

#include <math.h>

#include <list>
#include <string>
#include "hwlib.h"
#include "hps_0.h"
#include "socal/socal.h"
#include "socal/hps.h"
#include <stdbool.h>

#include "scaralib.h"
#include "poseOfWeldingGun.h"
#include "Torchcalibration.h"
#include "md5.h"
#include "ProjectUpdate.h"
#include "robot_class/Variable.h"      //变量类

#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )
enum RobotType
{
    JointRobot = 0,
	CoordRobot = 1
};
//
typedef struct
{
    unsigned long int DATA;
    unsigned long int DIRECTION;
    unsigned long int INTERRUPT_MASK;
    unsigned long int EDGE_CAPTURE;
}PIO_STR;


typedef struct
{   //多变 占4个字节  脉冲数    能够被多线程访问
    volatile long int J1step : 32;		// W : 坐标对应的脉冲数			R : J1PUCOUNT
    volatile long int J2step : 32;		// W : 坐标对应的脉冲数			R : J2PUCOUNT
    volatile long int J3step : 32;		// W : 坐标对应的脉冲数			R : J3PUCOUNT
    volatile long int J4step : 32;		// W : 坐标对应的脉冲数			R : J4PUCOUNT
    volatile long int J5step : 32;		// W : 坐标对应的脉冲数			R : J5PUCOUNT
    volatile long int J6step : 32;		// W : 坐标对应的脉冲数			R : J6PUCOUNT
    volatile long int J7step : 32;		// W : 坐标对应的脉冲数			R : J7PUCOUNT
    volatile long int J8step : 32;		// W : 坐标对应的脉冲数			R : J8PUCOUNT

    volatile unsigned long int contia : 32;		// W : 容器					R : Message
    volatile unsigned long int     Tt : 32;		// W : 定时间隔，改变速度	R : Message

    union{
        volatile unsigned long int CMD : 32;		// W : 控制指令  ‘C’:清除绝对值计数  ‘R’:开始运行	R : ‘N’:正在运行，‘Y’:运行完成
    	struct{
    		volatile unsigned char D0 : 8;
    		volatile unsigned char D1 : 8;
    		volatile unsigned char D2 : 8;
    		volatile unsigned char D3 : 8;
    	}field;//其实就相当于是一个对象的值
    };

}MOT_SIR;    //MOT  暂用多少个字节数


typedef union{
    unsigned long int CMD;
	struct{
    	volatile unsigned short int data 		: 10;	//模拟量数据0-0x3FF 对应 0-10V
    	volatile unsigned short int spd 		: 1;	//speed control bit:	1--fast mode  0--slow mode
    	volatile unsigned short int pwr 		: 1;	//power control bit: 	1--power down 0--power operation
    	volatile unsigned short int ch 			: 2;	//AVO channel selection:0-3
    	volatile unsigned short int start 		: 2;	//start control: 		2'b11--start  other--stop
	}field;
}AVO_SIR;


#define ZERO_THRESH 	(0.000001)

////==========================电机脉冲当量 pu/度 或 pu/mm 步进电机版本
//#define J1PUPR		(3746.0317460317460317460317460317)
//#define J2PUPR		( 666.6666666666666666666666666667)
//#define J3PUPR		( 954.9296585513720146133025802351)
//#define J4PUPR		(  83.3333333333333333333333333333)
//#define J5PUPR		(1777.7777777777777777777777777778)
//#define J6PUPR		( 500.0000000000000000000000000000)
//#define J7PUPR		( 500.0000000000000000000000000000)
//#define J8PUPR		( 500.0000000000000000000000000000)


//==========================电机脉冲当量 pu/度 或 pu/mm 伺服版本      //电机脉冲当量，哈哈
//#define J1PUPR			(3746.0317460317460317460317460317)
//#define J2PUPR			(2666.6666666666666666666666666667)
//#define J3PUPR			(954.92965855137201461330258023509)
//#define J4PUPR			(177.77777777777777777777777777778)
//#define J5PUPR			(1777.7777777777777777777777777778)
//#define J6PUPR			( 500.0000000000000000000000000000)
//#define J7PUPR			(7111.1111111111111111111111111111)
//#define J8PUPR			(3333.3333333333333333333333333333)


////==========================电机脉冲当量 pu/度 或 pu/mm 测试版本
//#define J1PUPR			(374.60317460317460317460317460317)
//#define J2PUPR			(266.66666666666666666666666666667)
//#define J3PUPR			(95.492965855137201461330258023509)
//#define J4PUPR			(17.777777777777777777777777777778)
//#define J5PUPR			(177.77777777777777777777777777778)
//#define J6PUPR			( 50.00000000000000000000000000000)
//#define J7PUPR			(711.11111111111111111111111111111)
//#define J8PUPR			(333.33333333333333333333333333333)

//切割脉冲当量
//#define J1PUPR			(2547.7707006369426751592356687898)
//#define J2PUPR			(1819.0913638637500568466051207422)
//#define J3PUPR			(1591.0797703753675394269567099016)
//#define J4PUPR			(1428.5714285714285714285714285714)
//#define J5PUPR			(1123.5955056179775280898876404494)
//#define J6PUPR			( 500.0000000000000000000000000000)
//#define J7PUPR			(7111.1111111111111111111111111111)
//#define J8PUPR			(3333.3333333333333333333333333333)
////小机器人
//#define J1PUPR			(200.78431372549019607843137254902*5*100/105)
//#define J2PUPR			(89.875732569540895493016713433891)
//#define J3PUPR			(954.92965855137201461330258023509)
//#define J4PUPR			(133.33333333333333333333333333333)
//#define J5PUPR			(133.33333333333333333333333333333)
//#define J6PUPR			( 500.0000000000000000000000000000)
//#define J7PUPR			(7111.1111111111111111111111111111)
//#define J8PUPR			(3333.3333333333333333333333333333)

//#define J1PUPR			(3746.0317460317460317460317460317)
//#define J2PUPR			(2666.6666666666666666666666666667*18/21)
//#define J3PUPR			(10000*10/20/pi)
//#define J4PUPR			(277.7777777777778*4)
//#define J5PUPR			(277.7777777777778*4)
//#define J6PUPR			( 500.0000000000000000000000000000)
//#define J7PUPR			(7111.1111111111111111111111111111)
//#define J8PUPR			(3333.3333333333333333333333333333)

#define PORT 1000
#define BUFFER_SIZE 	1024
extern sockaddr_in udpaddr;




//==========================================关节与变位机构整合结构体
typedef struct
{
	float j1; float j2; double j3; float j4;
	float j5; float j6; float j7; float j8;

}getJoint;

extern double J1PUPR;
extern double J2PUPR;
extern double J3PUPR;
extern double J4PUPR;
extern double J5PUPR;
extern double J6PUPR;
extern double J7PUPR;
extern double J8PUPR;
extern Joint lastJ;
extern bool lastJIsinit;
extern RobotType robotType;
extern volatile unsigned long h2p_lw_led_addr;
extern volatile unsigned long h2p_lw_key_addr;
extern volatile unsigned long h2p_lw_mot_addr;
extern volatile unsigned long h2p_lw_IOM_addr;
extern volatile unsigned long h2p_lw_AVO_addr;

#define MOT 	((MOT_SIR *)h2p_lw_mot_addr)  //其实就是一个地址  代表SOC的地址
#define IOM		((PIO_STR *)h2p_lw_IOM_addr)
#define AVO		((AVO_SIR *)h2p_lw_AVO_addr)

//限位接口
#define J1LMT	((~(IOM->DATA) & 0x00000100))             //限位结构     soc io口的数据吗 8位  16进制    32位 4字节
#define J2LMT	((~(IOM->DATA) & 0x00000200))
#define J3LMT	((~(IOM->DATA) & 0x00000400))
#define J4LMT	((~(IOM->DATA) & 0x00000800))
#define J5LMT	((~(IOM->DATA) & 0x00001000))
#define J6LMT	((~(IOM->DATA) & 0x00002000))
#define J7LMT	((~(IOM->DATA) & 0x00004000))
#define J8LMT	((~(IOM->DATA) & 0x00008000))

#define XLMT	((~(IOM->DATA) & 0X00010000))
#define YLMT	((~(IOM->DATA) & 0X00020000))
#define ZLMT	((~(IOM->DATA) & 0X00040000))
#define EmergencyIO	((~(IOM->DATA) & 0x00080000))

//===========================加减速定义
//===========================加减速定义
#define VVVSSS		( 1000)			//初速度
#define VVVCCC		(24000*3)			//末速度
#define ADDTIM		(    1)			//加速时间
#define VADDTIMES 	( 1000)			//加速步数                    加速的步数
//#define VVVCCC1		(24000*3)			//末速度       摇杆焊接速度
#define VVVAAA		((double)(VVVCCC - VVVSSS) / (VADDTIMES - 1) / ADDTIM)	//加速度
#define MINPU		((VVVSSS + VVVCCC) / 2 * ADDTIM)
//加速长度
//#define MINPU1		((VVVSSS + VVVCCC1) / 2 * ADDTIM)
//#define VADDTIMES1 	( 1000)			//加速步数                    加速的步数
//#define VVVAAA1		((double)(VVVCCC1 - VVVSSS) / (VADDTIMES1 - 1) / ADDTIM)	//加速度



//下面是空移的数据
/**********************************/
//#define ADDTIM2		(    1)
//#define VVVSSS2		( 3000)
//#define VVVCCC2		(24000 * 5)
//#define VADDTIMES2 	( 1000)	  //加速步数
//#define VVVAAA2	((double)(VVVCCC2 - VVVSSS2) / (VADDTIMES2 - 1) / ADDTIM2)	//加速度
//#define MINPU2		((VVVSSS2 + VVVCCC2) / 2 * ADDTIM2)
//extern unsigned long int vddTable2[VADDTIMES2];	//加速改变列表

/************************************/


extern unsigned long int vddTable[VADDTIMES];	//加速改变列表
extern unsigned long int nowSpeed;

//===========================手柄控制定义
//A B X Y LB RB BACK START JLB JRB left right down up
#define JOY_A				( 0)
#define JOY_B				( 1)
#define JOY_X				( 2)
#define JOY_Y				( 3)
#define JOY_LB				( 4)
#define JOY_RB				( 5)
#define JOY_BACK			( 6)
#define JOY_START			( 7)
#define JOY_JLB				( 8)
#define JOY_JRB				( 9)
#define JOY_left			(10)
#define JOY_right			(11)
#define JOY_down			(12)
#define JOY_up				(13)

extern  bool joyButtons[14];        //14个按钮
//LT RT LUDT LLRT RUDT RLRT;
extern double  joySticks[ 5];

extern Joint currentJ, targetJ;

extern Coint targetC, currentC;

extern double interpolationSpeed;

// debug
extern Matrix4d TCFmatrix;

extern bool TESKVOID;
extern ofstream writefile;


#endif /* LANSUNV2_0_SRC_COMMON_H_ */
