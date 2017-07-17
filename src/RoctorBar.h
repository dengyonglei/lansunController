/*
 * RoctorBar.h
 *
 *  Created on: 2016年11月16日
 *      Author: Administrator
 */
#include "common.h"
#include "moto.h"
#include "FixedLengthMoving.h"
#include "DlyCommon.h"

#ifndef SRC_ROCTORBAR_H_
#define SRC_ROCTORBAR_H_

class RoctorBar
{
public:
	RoctorBar();
	virtual ~RoctorBar();
private:
	bool IsFixedPoint;
	AxisType axistype; //轴的类型
	double speed;
	bool runing;  //运动标志位
	bool Joyruning;//摇杆运动标志位
	Joint targetJ;
	Coint targetC;
	int joyruntime;//摇杆走一小段运行时间，单位微秒
	double rate;
	double lastspeed;
    void RoctorMove();
    void Roctor1Move();
	void SingleAxisMove();//单轴移动
	Vector3d lastV;
    Vector3d lastA;
    double XAxisSpeed;   //X轴速度
    double YAxisSpeed;   //Y轴速度
    double ZAxisSpeed;   //升降轴速度
    double ModifiedGear1JoySpeed;    //变位机构1速度
    double ModifiedGear2JoySpeed;    //变位机构2速度
    double FixedPointRotationSpeed;     //定点转速度
    double FixedPointSwingSpeed;        //定点摆速度
    double x,y,z;
    void clearSpeed();
public:
	void RoctorRun();
	void setData(AxisType axistype,double speed);    //获取数据
    void run();
};

#endif /* SRC_ROCTORBAR_H_ */
