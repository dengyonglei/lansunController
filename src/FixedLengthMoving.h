/*
 * FixedLengthMoving.h
 *
 *  Created on: 2016年11月15日
 *      Author: Administrator
 */
#include <iostream>
using namespace std;
#ifndef SRC_FIXEDLENGTHMOVING_H_
#define SRC_FIXEDLENGTHMOVING_H_
#include "moto.h"
#include "common.h"
//x y 大臂 小臂 升降轴 回转轴 摆抢 变位机构1 变位机构2 定点转 定点摆
enum AxisType
{    XAxis,YAxis,ZAxis,BigArm,SmallArm,UpDownAxis,RotorAxis,SwingAxis,ModifiedGear1,ModifiedGear2,FixedPointRotation,
	 FixedPointSwing,ModifiedGear1Joy,ModifiedGear2Joy,UnKnown
};
//定长移动
class FixedLengthMoving
{
public:
	FixedLengthMoving();
	virtual ~FixedLengthMoving();

private:
	AxisType axistype;      //轴号
	double speed;           //移动速度
	double distance;        //移动距离
	bool runing;           //运行状态
	Joint j;    //
	Coint c;
	void SingleAxismove();
public:
	void setData(AxisType,double speed,double distance);
    bool  run();


};

#endif /* SRC_FIXEDLENGTHMOVING_H_ */
