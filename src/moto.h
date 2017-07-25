/*
 * moto.h
 *
 *  Created on: 2016年5月23日
 *      Author: Yin
 */

#ifndef LANSUNV2_0_SRC_MOTO_H_
#define LANSUNV2_0_SRC_MOTO_H_

#include "common.h"
#include <iostream>
using namespace std;
void clcPUs(int axis);
void moto_init(void);
void moto_XYZclear(void);
extern double Vspeed;
//回零后各轴偏移指定距离，保证机械臂成一条线
//机械臂停止
void robot_stop(void);
void moto_SettingJ(char ch, double angle,double speed);
void moto_runJoy(Joint j, Coint c);
void moto_runJoy1(int ch);
bool moto_runJAbs(Joint j, Coint c,double speed);
void moto_runJoyAbs(Joint j, Coint c);
//=====================================================插补线段运动控制
void moto_runInterpolationAbs(Joint j, Coint c,float speed);
void moto_runJ(Joint j, Coint c,double speed);
void getVList(unsigned long int *table);
void delayNus(int n);
void delay1ms(void);
void delayNms(float n);

//获取两个向量之间的角度
bool getTwoVector3Angle(Vector3d p1, Vector3d p2);
float getTwoVector3AngleValue( Vector3f p1,  Vector3f p2);
//J1运行到限位临界点 如果已经限位则逆时针旋转，否则顺时针转，检测到限位临界点则减速停止
void J1RunToLimit(char ch, int speedL);

//调用此函数时，一定要保证J4（枪回转），相对于初始位置左侧
void allAxisInit(void);

#endif /* LANSUNV2_0_SRC_MOTO_H_ */
