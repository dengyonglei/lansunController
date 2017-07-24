/*
 * moto.cpp
 *
 *  Created on: 2016年5月23日
 *      Author: Administrator
 */

#include "moto.h"
#include "Parameter.h"
using namespace std;
#include <stdio.h>
#include <iostream>

//MOT
static int a1 = 0;	//加速度
double Vspeed=0;


//轴的清理工作
void clcPUs(int axis)
{
	MOT->field.D3 = 'C';                                   //D1 D2 D3 D4 代表什么
	MOT->field.D2 = 'J';
	switch(axis)   //记录是哪个轴在运行和工作
	{
		case 1:MOT->field.D1 = '1';break;   //x
		case 2:MOT->field.D1 = '2';break;   //y
		case 3:MOT->field.D1 = '3';break;   //z
		case 4:MOT->field.D1 = '4';break;
		case 5:MOT->field.D1 = '5';break;
		case 6:MOT->field.D1 = '6';break;
		case 7:MOT->field.D1 = '7';break;
		case 8:MOT->field.D1 = '8';break;
	   default:MOT->field.D1 = '0';break;
	}
	MOT->field.D0 = 'c';
}


//   vs：启动速度 pu/s
//   vc：最终速度 pu/s
//    t：加速时间 s
//table：插补间隔输出（控制底层整体插补速度）
void getVList(unsigned long int *table)
{

	for(int i=0;i<VADDTIMES;i++)
	{
		table[i] = (unsigned long int)(6250000.0 / (VVVSSS + VVVAAA * ADDTIM * i));//记录不同的速度
	}

}
//
void delayNus(int n)    //微秒级别的
{
	struct timeval temp;
	if(n<=0)
		return;
	temp.tv_sec = 0;
	temp.tv_usec = n;
	select(0,NULL,NULL,NULL,&temp);
}

void delay1ms(void)
{
	struct timeval temp;
	temp.tv_sec = 0;
	temp.tv_usec = 1000;
	select(0,NULL,NULL,NULL,&temp);//估计就是耗时的程序
}

void delayNms(float n){
    if(n<=0)
    	return;
	struct timeval temp;
	temp.tv_sec = 0;
	temp.tv_usec = (int)(1000 * n);
	select(0,NULL,NULL,NULL,&temp);
}

//获取两个向量之间的角度
bool getTwoVector3Angle(Vector3d p1, Vector3d p2)
{
	 //两个向量积/模  cos值  -1 到 1 之间
//		float costheta = p1.dot(p2) / (Vector3d().absV3(p1) * Vector3d().absV3(p2));
	float costheta = p1.dot(p2) / (p1.norm() * p2.norm());
	if(costheta >= 1)
		costheta = 1;
	else if(costheta <= -1)
		costheta = -1;
	float theta = acosf(costheta);   //角度值 是弧度
	return (theta < 0.05*pi) ? true : false;   //判断弧度的范围，返回状态            弧度太小，返回范围哈哈

}

float getTwoVector3AngleValue( Vector3f p1,  Vector3f p2)
{
    //两个向量积/模  cos值  -1 到 1 之间
//	float costheta = p1.dot(p2) / (Vector3d().absV3(p1) * Vector3d().absV3(p2));
	float costheta = p1.dot(p2) / (p1.norm() * p2.norm());
	if(costheta >= 1.)
		costheta = 1.;
	else if(costheta <= -1.)
		costheta = -1;
	float theta = acosf(costheta);   //角度值 是弧度

	return theta*180/pi;   //判断弧度的范围，返回状态            弧度太小，返回范围哈哈
}

//此函数的功能是给定关节角的对数
void moto_runJ(Joint j, Coint c,double speed)
{
    cout << "单轴移动开始" << endl;
    double rate = speed / 6000.0;
    if(rate > 1)
    	rate = 1;
	int i = 0;
	int a = 0;	//加速度
	unsigned long int isContia = 0;     //容器
	MOT->field.D3 = 'S';//停止指令，给

	//转化为脉冲数
	 long long int j1pu;
	 long long int j2pu;
	if(robotType == CoordRobot)
	{
	  j1pu = j.j1  * J1PUPR;
	  j2pu = j.j2  * J2PUPR;
	}
	else
	{
	  j1pu = j.j1 / pi*180 * J1PUPR;
	  j2pu = j.j2 / pi*180 * J2PUPR;
	}
	long long int j3pu = j.j3          * J3PUPR;
	long long int j4pu = j.j4 / pi*180 * J4PUPR;
	long long int j5pu = j.j5 / pi*180 * J5PUPR;
	long long int j6pu = j.j6 / pi*180 * J6PUPR;
	long long int j7pu = c.c1 / pi*180 * J7PUPR;
	long long int j8pu = c.c2 / pi*180 * J8PUPR;

    //将以前的脉冲数读出来
	long int oldJ1step = MOT->J1step;
	long int oldJ2step = MOT->J2step;
	long int oldJ3step = MOT->J3step;
	long int oldJ4step = MOT->J4step;
	long int oldJ5step = MOT->J5step;
	long int oldJ6step = MOT->J6step;
	long int oldJ7step = MOT->J7step;
	long int oldJ8step = MOT->J8step;

   //找到一个整体的容积
	unsigned long int contia = (unsigned long int)(sqrtf(j1pu * j1pu + j2pu * j2pu + \
			   										 j3pu * j3pu + j4pu * j4pu + \
														 j5pu * j5pu + j6pu * j6pu + \
														 j7pu * j7pu + j8pu * j8pu));

	   if(contia == 0)
	   {
		   cout << "距离短不移动" <<endl;
		   return;
	   }
	   long int maxspeed = 0;
	   while(1)
	   {
		   long int maxspeed1 = maxspeed;
		   maxspeed1 += 500;
		   double v1 = fabs((double)j1pu / (double)contia * maxspeed1);
		   double v2 = fabs((double)j2pu / (double)contia * maxspeed1);
		   double v3 = fabs((double)j3pu / (double)contia * maxspeed1);
		   double v4 = fabs((double)j4pu / (double)contia * maxspeed1);
		   double v5 = fabs((double)j5pu / (double)contia * maxspeed1);
		   double v6 = fabs((double)j6pu / (double)contia * maxspeed1);
		   double v7 = fabs((double)j7pu / (double)contia * maxspeed1);
		   double v8 = fabs((double)j8pu / (double)contia * maxspeed1);
		   if(v1 > Parameter::SingleAxisMaxJspeed[0] || v2 > Parameter::SingleAxisMaxJspeed[1] ||
			  v3 > Parameter::SingleAxisMaxJspeed[2] || v4 > Parameter::SingleAxisMaxJspeed[3] ||
			  v5 > Parameter::SingleAxisMaxJspeed[4] || v6 > Parameter::SingleAxisMaxJspeed[5] ||
			  v7 > Parameter::SingleAxisMaxJspeed[6] || v8 > Parameter::SingleAxisMaxJspeed[7]
			 )
			   break;
		   maxspeed =  maxspeed1;

	   }
	   //通过单轴的速度限制求出最大的合速度


	int  ADDTIM2 = 1;
	int  VVVSSS2 = 3000;
	long  int  VVVCCC2 = maxspeed *rate;
	int  VADDTIMES2 = 1000;  //加速步数
	double VVVAAA2= ((double)(VVVCCC2 - VVVSSS2) / (VADDTIMES2 - 1) / ADDTIM2);	//加速度
	double MINPU2	=	((VVVSSS2 + VVVCCC2) / 2 * ADDTIM2);
	long int vddTable2[VADDTIMES2];
	for(int i=0;i<VADDTIMES;i++)
		{
		vddTable2[i] = (long int)(6250000.0 / (VVVSSS2 + VVVAAA2 * ADDTIM2 * i));//记录不同的速度
		}
//将数据写入
	MOT->J1step = j1pu;         //这是起点位置
	MOT->J2step = j2pu;
	MOT->J3step = j3pu;
	MOT->J4step = j4pu;
	MOT->J5step = j5pu;
	MOT->J6step = j6pu;
	MOT->J7step = j7pu;
	MOT->J8step = j8pu;
	MOT->contia = contia;
	MOT->Tt = vddTable2[0];//定时间隔，改变速度
	MOT->field.D3 = 'R';//开始运行            要等开始运行时才下面才会传数据给上面         //嵌入式界面
//运行未完成的状态

	while(MOT->field.D3 == 'N')
	{
		 if(Variable::IsStop)   //如果接收到停止指令就让他停下来
		 {
//			 cout<<"减速"<<endl;
		  i = i - 5;
		  if(i < 1)
		  {
			cout<<"轴移动暂停"<<endl;
			robot_stop();
			Variable::IsStop = false;
			break;
		  }
		 }
//老值与新值得差距，完成度    这个是不断刷新的吗
		long long int j1has = MOT->J1step - oldJ1step;   //实际已经完成的情况哈哈
		long long int j2has = MOT->J2step - oldJ2step;
		long long int j3has = MOT->J3step - oldJ3step;
		long long int j4has = MOT->J4step - oldJ4step;
		long long int j5has = MOT->J5step - oldJ5step;
		long long int j6has = MOT->J6step - oldJ6step;
		long long int j7has = MOT->J7step - oldJ7step;
		long long int j8has = MOT->J8step - oldJ8step;

		isContia = (unsigned long int)(sqrtf(j1has * j1has + j2has * j2has + \
											 j3has * j3has + j4has * j4has + \
											 j5has * j5has + j6has * j6has + \
											 j7has * j7has + j8has * j8has));    //总的完成情况

		if(contia <= 2 * MINPU2){	//当线段长度不满足加减速必要长度时  就是这段速度好短

			if(isContia < contia / 2){       //完成量达不到一半的时候
				a += VVVAAA2;	//加速度         //加一点速度
			}else{
				a -= VVVAAA2;       //就开始减一点速度
				if(a <= 0){
					a = 0;
				}
			}
			MOT->Tt = (unsigned long int)(6250000.0 / (VVVSSS2 + a));      //整体的速度控制

		}else{						//完全加减速

			if(isContia < MINPU2)                          //完成情况比那个
			{											//加速
				MOT->Tt = vddTable2[i];   //得到的一个速度梯度，把速度梯度赋给他的一个整体的速度，哈哈，这么厉害吧
				i++;
				if(i >= VADDTIMES2){
					i = VADDTIMES2 - 1;
				}
			}else if(isContia >= MINPU2 && isContia < (contia - MINPU2)){		//匀速   哈哈，一直匀速运动到对称位置
				MOT->Tt = vddTable2[i];
			}else{
				MOT->Tt = vddTable2[i];		//并把速度梯度赋给他 哈哈								//减速
				i--;
				if(i <= 0){
					i = 0;
				}
			}
		}
		delay1ms();     //延时1s,让其全部完成为止
	 if(Variable::IsStop)   //如果接收到停止指令就让他停下来
			 {
	//			 cout<<"减速"<<endl;
			  i = i - 5;
			  if(i < 1)
			  {
				cout<<"轴移动暂停"<<endl;
				robot_stop();
				Variable::IsStop = false;
				break;
			  }
	}
	}
	robot_stop();
}



//绝对
bool moto_runJAbs(Joint j, Coint c,double speed)
{
	    int i = 0;
		int a = 0;	//加速度
		double rate = speed / 6000;
		if(rate > 1)   //速度不能超过1
			rate = 1;
		unsigned long int isContia = 0;
		MOT->field.D3 = 'S';
		long int oldJ1step = MOT->J1step;
		long int oldJ2step = MOT->J2step;
		long int oldJ3step = MOT->J3step;
		long int oldJ4step = MOT->J4step;
		long int oldJ5step = MOT->J5step;
		long int oldJ6step = MOT->J6step;
		long int oldJ7step = MOT->J7step;
		long int oldJ8step = MOT->J8step;
		long long int j1pu;
		long long int j2pu;
        if(robotType == CoordRobot)
        {
		 j1pu = j.j1 * J1PUPR - oldJ1step;
		 j2pu = j.j2 * J2PUPR - oldJ2step;
        }
        else
        {
         j1pu = j.j1 / pi*180 * J1PUPR - oldJ1step;
         j2pu = j.j2 / pi*180 * J2PUPR - oldJ2step;
        }
		long long int j3pu = j.j3          * J3PUPR - oldJ3step;
		long long int j4pu = j.j4 / pi*180 * J4PUPR - oldJ4step;
		long long int j5pu = j.j5 / pi*180 * J5PUPR - oldJ5step;
		long long int j6pu = j.j6 / pi*180 * J6PUPR - oldJ6step;
		long long int j7pu = c.c1 / pi*180 * J7PUPR - oldJ7step;
		long long int j8pu = c.c2 / pi*180 * J8PUPR - oldJ8step;



		unsigned long int contia = (unsigned long int)(sqrtf(j1pu * j1pu + j2pu * j2pu + \
															 j3pu * j3pu + j4pu * j4pu + \
															 j5pu * j5pu + j6pu * j6pu + \
															 j7pu * j7pu + j8pu * j8pu));
		           if(contia == 0)
		           {
		        	   cout << "距离短不移动" <<endl;
		        	   return true;
		           }
				   long int maxspeed = 0;
				   while(1)
				   {
					   long int maxspeed1 = maxspeed;
					   maxspeed1 += 500;
					   double v1 = fabs((double)j1pu / (double)contia * maxspeed1);
					   double v2 = fabs((double)j2pu / (double)contia * maxspeed1);
					   double v3 = fabs((double)j3pu / (double)contia * maxspeed1);
					   double v4 = fabs((double)j4pu / (double)contia * maxspeed1);
					   double v5 = fabs((double)j5pu / (double)contia * maxspeed1);
					   double v6 = fabs((double)j6pu / (double)contia * maxspeed1);
					   double v7 = fabs((double)j7pu / (double)contia * maxspeed1);
					   double v8 = fabs((double)j8pu / (double)contia * maxspeed1);
					   if(v1 > Parameter::SingleAxisMaxJspeed[0] || v2 > Parameter::SingleAxisMaxJspeed[1] ||
						  v3 > Parameter::SingleAxisMaxJspeed[2] || v4 > Parameter::SingleAxisMaxJspeed[3] ||
						  v5 > Parameter::SingleAxisMaxJspeed[4] || v6 > Parameter::SingleAxisMaxJspeed[5] ||
				          v7 > Parameter::SingleAxisMaxJspeed[6] || v8 > Parameter::SingleAxisMaxJspeed[7]
						 )
						   break;
					   maxspeed =  maxspeed1;

				   }
				   //通过单轴的速度限制求出最大的合速度


		        int  ADDTIM2 = 1;
		        int  VVVSSS2 = 1000;
		        long  int  VVVCCC2 = maxspeed *rate;
		        int  VADDTIMES2 = 1000;  //加速步数
		        double VVVAAA2= ((double)(VVVCCC2 - VVVSSS2) / (VADDTIMES2 - 1) / ADDTIM2);	//加速度
		        double MINPU2	=	((VVVSSS2 + VVVCCC2) / 2 * ADDTIM2);
		        long int vddTable2[VADDTIMES2];
		        for(int i=0;i<VADDTIMES;i++)
		        	{
		        	vddTable2[i] = (long int)(6250000.0 / (VVVSSS2 + VVVAAA2 * ADDTIM2 * i));//记录不同的速度
		        	}
		MOT->J1step = j1pu;
		MOT->J2step = j2pu;
		MOT->J3step = j3pu;
		MOT->J4step = j4pu;
		MOT->J5step = j5pu;
		MOT->J6step = j6pu;
		MOT->J7step = j7pu;
		MOT->J8step = j8pu;
		MOT->contia = contia;
		MOT->Tt = vddTable2[0];
		MOT->field.D3 = 'R';

		while(MOT->field.D3 == 'N')
		{
			 if(Variable::IsStop)
			 {
			  i = i - 1;
			  if(i < 1)
			  {
			    cout<<"轴移动暂停"<<endl;
			    robot_stop();
			    return false;
			  }
			 }
			long long int j1has = MOT->J1step - oldJ1step;
			long long int j2has = MOT->J2step - oldJ2step;
			long long int j3has = MOT->J3step - oldJ3step;
			long long int j4has = MOT->J4step - oldJ4step;
			long long int j5has = MOT->J5step - oldJ5step;
			long long int j6has = MOT->J6step - oldJ6step;
			long long int j7has = MOT->J7step - oldJ7step;
			long long int j8has = MOT->J8step - oldJ8step;

			isContia = (unsigned long int)(sqrtf(j1has * j1has + j2has * j2has + \
												 j3has * j3has + j4has * j4has + \
												 j5has * j5has + j6has * j6has + \
												 j7has * j7has + j8has * j8has));


			if(contia <= 2 * MINPU2){	//当线段长度不满足加减速必要长度时

				if(isContia < contia / 2){
					a += VVVAAA2;	//加速度
				}else{
					a -= VVVAAA2;
					if(a <= 0){
						a = 0;
					}
				}
				MOT->Tt = (unsigned long int)(6250000.0 / (VVVSSS2 + a));

			}else{						//完全加减速

				if(isContia < MINPU2)
				{											//加速
					MOT->Tt = vddTable2[i];
					i++;
					if(i >= VADDTIMES2)
					{
						i = VADDTIMES2 - 1;
					}
				}else if(isContia >= MINPU2 && isContia < (contia - MINPU2)){		//匀速
					MOT->Tt = vddTable2[i];
				}else{
					MOT->Tt = vddTable2[i];										//减速
					i--;
					if(i <= 0){
						i = 0;
					}
				}
			}
			delay1ms();
		}
    return true;
}

void moto_runJoyAbs(Joint j, Coint c)
{
	int i = 0;
	unsigned long int isContia = 0;
	MOT->field.D3 = 'S';
	long int oldJ1step = MOT->J1step;
	long int oldJ2step = MOT->J2step;
	long int oldJ3step = MOT->J3step;
	long int oldJ4step = MOT->J4step;
	long int oldJ5step = MOT->J5step;
	long int oldJ6step = MOT->J6step;
	long int oldJ7step = MOT->J7step;
	long int oldJ8step = MOT->J8step;
	long long int j1pu;
	long long int j2pu;
	if(robotType == CoordRobot)
	{
	 j1pu = j.j1  * J1PUPR - oldJ1step;
	 j2pu = j.j2  * J2PUPR - oldJ2step;
	}
	else
	{
	  j1pu = j.j1 / pi*180 * J1PUPR - oldJ1step;
	  j2pu = j.j2 / pi*180 * J2PUPR - oldJ2step;
	}
	long long int j3pu = j.j3          * J3PUPR - oldJ3step;
	long long int j4pu = j.j4 / pi*180 * J4PUPR - oldJ4step;
	long long int j5pu = j.j5 / pi*180 * J5PUPR - oldJ5step;
	long long int j6pu = j.j6 / pi*180 * J6PUPR - oldJ6step;
	long long int j7pu = c.c1 / pi*180 * J7PUPR - oldJ7step;
	long long int j8pu = c.c2 / pi*180 * J8PUPR - oldJ8step;

	unsigned long int contia = (unsigned long int)(sqrtf(j1pu * j1pu + j2pu * j2pu + \
														 j3pu * j3pu + j4pu * j4pu + \
														 j5pu * j5pu + j6pu * j6pu + \
														 j7pu * j7pu + j8pu * j8pu));

	MOT->J1step = j1pu;
	MOT->J2step = j2pu;
	MOT->J3step = j3pu;
	MOT->J4step = j4pu;
	MOT->J5step = j5pu;
	MOT->J6step = j6pu;
	MOT->J7step = j7pu;
	MOT->J8step = j8pu;
	MOT->contia = contia;
	MOT->Tt = vddTable[0];
	MOT->field.D3 = 'R';
	while(MOT->field.D3 == 'N')
	{
		 if(Variable::IsStop)
		 break;
		long long int j1has = MOT->J1step - oldJ1step;
		long long int j2has = MOT->J2step - oldJ2step;
		long long int j3has = MOT->J3step - oldJ3step;
		long long int j4has = MOT->J4step - oldJ4step;
		long long int j5has = MOT->J5step - oldJ5step;
		long long int j6has = MOT->J6step - oldJ6step;
		long long int j7has = MOT->J7step - oldJ7step;
		long long int j8has = MOT->J8step - oldJ8step;

		isContia = (unsigned long int)(sqrtf(j1has * j1has + j2has * j2has + \
											 j3has * j3has + j4has * j4has + \
											 j5has * j5has + j6has * j6has + \
											 j7has * j7has + j8has * j8has));


		if(contia <= 2 * MINPU){	//当线段长度不满足加减速必要长度时

			if(isContia < contia / 2)
			{
				a1 += VVVAAA;	//加速度
			}else
			{
				a1 -= VVVAAA;
				if(a1 <= 0)
				{
					a1 = 0;
				}
			}
			MOT->Tt = (unsigned long int)(6250000.0 / (VVVSSS + a1));

		}else{						//完全加减速

			if(isContia < MINPU)
			{											//加速
				MOT->Tt = vddTable[i];
				i++;
				if(i >= VADDTIMES)
				{
					i = VADDTIMES  - 1;
				}
			}else if(isContia >= MINPU && isContia < (contia - MINPU)){		//匀速
				MOT->Tt = vddTable[i];
			}else{
				MOT->Tt = vddTable[i];										//减速
				i--;
				if(i <= 0)
				{
					i = 0;
				}
			}
		}
		delay1ms();
	}
}


//此时加减速通过线段条数控制，传进来是绝对量,是弧度值，并且是绝对的弧度值
void moto_runInterpolationAbs(Joint j, Coint c,float speed)
{
	//开始准备状态
	MOT->field.D3 = 'S';
	//先把当前的脉冲当量存起来，发的是当前的脉冲当量
	long int oldJ1step = MOT->J1step;
	long int oldJ2step = MOT->J2step;
	long int oldJ3step = MOT->J3step;
	long int oldJ4step = MOT->J4step;
	long int oldJ5step = MOT->J5step;
	long int oldJ6step = MOT->J6step;
	long int oldJ7step = MOT->J7step;
	long int oldJ8step = MOT->J8step;

    //这是需要走的脉冲当量数
	long long int j1pu;
	long long int j2pu;
	if(robotType == CoordRobot)
	{
	 j1pu = j.j1  * J1PUPR - oldJ1step;
	 j2pu = j.j2  * J2PUPR - oldJ2step;
	}
	else
	{
	 j1pu = j.j1 / pi*180 * J1PUPR - oldJ1step;
	 j2pu = j.j2 / pi*180 * J2PUPR - oldJ2step;
	}
	long long int j3pu = j.j3          * J3PUPR - oldJ3step;
	long long int j4pu = j.j4 / pi*180 * J4PUPR - oldJ4step;
	long long int j5pu = j.j5 / pi*180 * J5PUPR - oldJ5step;
	long long int j6pu = j.j6 / pi*180 * J6PUPR - oldJ6step;
	long long int j7pu = c.c1 / pi*180 * J7PUPR - oldJ7step;
	long long int j8pu = c.c2 / pi*180 * J8PUPR - oldJ8step;


    //总的需要总的脉冲当量的合值
	unsigned long int contia = (unsigned long int)(sqrtf(j1pu * j1pu + j2pu * j2pu + \
														 j3pu * j3pu + j4pu * j4pu + \
														 j5pu * j5pu + j6pu * j6pu + \
														 j7pu * j7pu + j8pu * j8pu));
    //复制给，表示要发多少脉冲给他
	MOT->J1step = j1pu;
	MOT->J2step = j2pu;
	MOT->J3step = j3pu;
	MOT->J4step = j4pu;
	MOT->J5step = j5pu;
	MOT->J6step = j6pu;
	MOT->J7step = j7pu;
	MOT->J8step = j8pu;
	MOT->contia = contia;
	//Tt表示多少时间需要把脉冲发完，一个总的时间量
	MOT->Tt=(unsigned long int)(37500000/(speed*contia));
	MOT->field.D3 = 'R';
	while(MOT->field.D3 == 'N')
	{
		if(Variable::IsStop)
		break;
	}
}



void moto_runJoy(Joint j, Coint c)
{

	MOT->field.D3 = 'S';

	long int oldJ1step = MOT->J1step;
	long int oldJ2step = MOT->J2step;
	long int oldJ3step = MOT->J3step;
	long int oldJ4step = MOT->J4step;
	long int oldJ5step = MOT->J5step;
	long int oldJ6step = MOT->J6step;
	long int oldJ7step = MOT->J7step;
	long int oldJ8step = MOT->J8step;
	long long int j1pu;
	long long int j2pu;
	if(robotType == CoordRobot)
	{
	 j1pu = j.j1  * J1PUPR - oldJ1step;
	 j2pu = j.j2  * J2PUPR - oldJ2step;
	}
	else
	{
	 j1pu = j.j1 / pi*180  * J1PUPR - oldJ1step;
	 j2pu = j.j2 / pi*180  * J2PUPR - oldJ2step;
	}
	long long int j3pu = j.j3           * J3PUPR - oldJ3step;
	long long int j4pu = j.j4 / pi*180  * J4PUPR - oldJ4step;
	long long int j5pu = j.j5 / pi*180  * J5PUPR - oldJ5step;
	long long int j6pu = j.j6 / pi*180  * J6PUPR - oldJ6step;
	long long int j7pu = c.c1 / pi*180  * J7PUPR - oldJ7step;
	long long int j8pu = c.c2 / pi*180  * J8PUPR - oldJ8step;



	unsigned long int contia = (unsigned long int)(sqrtf(j1pu * j1pu + j2pu * j2pu + \
														 j3pu * j3pu + j4pu * j4pu + \
														 j5pu * j5pu + j6pu * j6pu + \
														 j7pu * j7pu + j8pu * j8pu));


	MOT->J1step = j1pu;
	MOT->J2step = j2pu;
	MOT->J3step = j3pu;
	MOT->J4step = j4pu;
	MOT->J5step = j5pu;
	MOT->J6step = j6pu;
	MOT->J7step = j7pu;
	MOT->J8step = j8pu;
	MOT->contia = contia;
	MOT->field.D3 = 'R';

	while(MOT->field.D3 == 'N'&&!Variable::IsStop)
	{

		MOT->Tt = (unsigned long int)(6250000.0 / (nowSpeed));

	}

}


//指定轴移动指定角度
//入口参数：	ch		指定轴，取值范围1,2,3,4,5,6,7,8
//			angle	旋转角度,单位度(第三轴单位为mm)
void moto_SettingJ(char ch, double angle,double speed)
{

	Joint j;
	Coint c;

	j.j1 = 0; j.j2 = 0; j.j3 = 0; j.j4 = 0;
	j.j5 = 0; j.j6 = 0; c.c1 = 0; c.c2 = 0;

//根据轴来切换
	switch(ch)
{
		case 1:
		{
			if(robotType == CoordRobot)
			j.j1 = angle;
			else
			j.j1 = angle / 180.0 * pi;
			break;
		}
		case 2:
		{
			if(robotType == CoordRobot)
			j.j2 = angle ;
			else
			j.j2 = angle / 180.0 * pi;
			break;
		}
		case 3: j.j3 = angle ; break;
		case 4: j.j4 = angle / 180.0 * pi; break;
		case 5: j.j5 = angle / 180.0 * pi; break;
		case 6: j.j6 = angle / 180.0 * pi; break;
		case 7: c.c1 = angle / 180.0 * pi; break;
		case 8: c.c2 = angle / 180.0 * pi; break;
		default:break;
	}

	moto_runJ(j, c,speed);//旋转一个角度吗
}

//指定轴运行到限位位置
//    ch:指定轴
//speedL:速度等级，[0,VADDTIMES]，速度等级越小距离限位临界点越近
void J1RunToLimit(char ch, int speedL)        //轴    速度
{

	int i = 0;
	bool lastLMT, nowLMT;
	int speedDownFlag = 0;

	long int j1pu = 0,j2pu = 0,j3pu = 0,j4pu = 0;
	long int j5pu = 0,j6pu = 0,j7pu = 0,j8pu = 0;      //8个脉冲数

	if(speedL >= VADDTIMES)
	{     //不能超过加速速度
		speedL = VADDTIMES;//最大限速来做哈的东西
	}

	MOT->field.D3 = 'S';         //是下面mot就是一个地址来控制的哈哈
	switch(ch)
	{     //根据轴来判定          是极限位置就乘以180   不是极限
		case 1:
			    if(Variable::BigArmDirectionChange)
			    j1pu = (J1LMT) ? -180 * J1PUPR :  180 * J1PUPR;
			    else
			    j1pu = (J1LMT) ? 180 * J1PUPR : - 180 * J1PUPR;//旋转180吗，  怎么回事呢
				lastLMT = (J1LMT) ? true : false;    //
				MOT->contia = abs(j1pu);   //取模去运动
				break;
		case 2:
			    if(Variable::SmallArmDirectionChange)
			    j2pu = (J2LMT) ? -180 * J2PUPR : 180 * J2PUPR;
			    else
			    j2pu = (J2LMT) ? 180 * J2PUPR : - 180 * J2PUPR;  //不是极限位置
				lastLMT = (J2LMT) ? true : false;
				MOT->contia = abs(j2pu);
				break;
		case 3:  if(Variable::UpDownAxisDirectionChange)
			     j3pu = (J3LMT) ? -2000 * J3PUPR :  2000 * J3PUPR;
		         else
			     j3pu = (J3LMT) ? 2000 * J3PUPR : - 2000 * J3PUPR;
				lastLMT = (J3LMT) ? true : false;
				MOT->contia = abs(j3pu);
				break;
		case 4:
			    if(Variable::RotorAxisDirectionChange)
			    j4pu = (J4LMT) ? -180 * J4PUPR :  180 * J4PUPR;
			    else
			    j4pu = (J4LMT) ? 180 * J4PUPR :  -180 * J4PUPR;
				lastLMT = (J4LMT) ? true : false;
				MOT->contia = abs(j4pu);
				//楼上坐标式回零改变方向
				break;
		case 5:
			    if(Variable::SwingAxisDirectionChange)
			    j5pu = (J5LMT) ? 180 * J5PUPR : -180 * J5PUPR;
			    else
			    j5pu = (J5LMT) ? -180 * J5PUPR : 180 * J5PUPR;
				lastLMT = (J5LMT) ? true : false;
				MOT->contia = abs(j5pu);
				break;
		case 6:
				break;
		case 7:
			if(Variable::ModifiedGear1DirectionChange)
			j7pu = (J7LMT) ? 180 * J7PUPR : -180 * J7PUPR;
			else
			j7pu = (J7LMT) ? -180 * J7PUPR : 180 * J7PUPR;
			lastLMT = (J7LMT) ? true : false;
			MOT->contia = abs(j7pu);
			break;

		case 8:
			if(Variable::ModifiedGear2DirectionChange)
			j8pu = (J8LMT) ? 180 * J8PUPR : -180 * J8PUPR;
			else
			j8pu = (J8LMT) ? -180 * J8PUPR : 180 * J8PUPR;
			lastLMT = (J8LMT) ? true : false;
			MOT->contia = abs(j8pu);
			break;
		default:break;
	}
	MOT->J1step = j1pu;
	MOT->J2step = j2pu;
	MOT->J3step = j3pu;
	MOT->J4step = j4pu;
	MOT->J5step = j5pu;
	MOT->J6step = j6pu;
	MOT->J7step = j7pu;
	MOT->J8step = j8pu;
	MOT->field.D3 = 'R';

	while(MOT->field.D3 == 'N'){
		 if(Variable::IsStop)    //急停
		 {
		  robot_stop();
		  return;
		 }
		delay1ms();
		switch(ch){
			case 1: nowLMT = (J1LMT) ? true : false; break;
			case 2: nowLMT = (J2LMT) ? true : false; break;
			case 3: nowLMT = (J3LMT) ? true : false; break;
			case 4: nowLMT = (J4LMT) ? true : false; break;
			case 5: nowLMT = (J5LMT) ? true : false; break;
			case 6: nowLMT = (J6LMT) ? true : false; break;
			case 7: nowLMT = (J7LMT) ? true : false; break;
			case 8: nowLMT = (J8LMT) ? true : false; break;
			default:break;
		}

		//两次状态不一致，表示机器从限位到不限位 或 机器不限位到限位      状态都是一致的
		if(lastLMT ^ nowLMT){
			speedDownFlag ++;
		}

		if(speedDownFlag > 10){
			i--;                                                            //1、这个程序不懂
			if(i <= 0){
				MOT->field.D3 = 'S';
				return;
			}
		}else{
			i++;                   //到1000步后会无限循环下去
			if(i >= speedL){
				i = speedL - 1;
			}
		}
		MOT->Tt = vddTable[i];
		if(Variable::IsStop)    //急停
		{
		 robot_stop();
         return;
		}
	}
}


void moto_init(void)
{

	for(int i=1;i<9;i++)
	{
		clcPUs(i);
	}

	robot_stop();
}

void moto_XYZclear(void)
{

	for(int i=1;i<4;i++)
	{
		clcPUs(i);
	}

	robot_stop();


}

void robot_stop(void)
{
	MOT->field.D3 = 'S';
	usleep(50000);
	MOT->J1step = 0;
	MOT->J2step = 0;
	MOT->J3step = 0;
	MOT->J4step = 0;
	MOT->J5step = 0;
	MOT->J6step = 0;
	MOT->J7step = 0;
	MOT->J8step = 0;
	MOT->contia = 0;
	MOT->Tt = 0xFFFF;
    usleep(50000);
	MOT->field.D3 = 'R';
}

