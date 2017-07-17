/*
 * RobotBack.h
 *
 *  Created on: 2016年11月16日
 *      Author: Administrator
 */

#ifndef SRC_ROBOTBACK_H_
#define SRC_ROBOTBACK_H_
#include "common.h"
class Robot_Back
{
public:
	Robot_Back();
	virtual ~Robot_Back();
private:
	double speed;//回零速度
	int AxisNumber;//回零轴号
	double distance;//回零偏移
	bool runing;    //标志位，是否运动率
    Joint j={0,0,0,0,0,0};
   	Coint c={0,0};

public:

	//设置数据
    void setData(int AxisNumber,double speed,double distance);
    //单轴回零
    void SingleAxisBack();
    //回零运动
    bool run();

};

#endif /* SRC_ROBOTBACK_H_ */
