/*
 * FixedLengthMoving.cpp
 *
 *  Created on: 2016年11月15日
 *      Author: Administrator
 */

#include "FixedLengthMoving.h"
#include "DlyCommon.h"
FixedLengthMoving::FixedLengthMoving() :axistype(UnKnown),speed(0),distance(0),runing(false)  //初始化
{


}
FixedLengthMoving::~FixedLengthMoving()
{

}
//设置数据
void FixedLengthMoving::setData(AxisType axistype,double speed,double distance)
{
	this->axistype=axistype;
    this->speed=speed;
    this->distance=distance;
    runing=true;
}

//定长移动过程中
void FixedLengthMoving::SingleAxismove()
{
     if(axistype==BigArm)
    	moto_SettingJ(1,distance,speed);
else if(axistype==SmallArm)
        moto_SettingJ(2,distance,speed);
else if(axistype==UpDownAxis)
        moto_SettingJ(3,distance,speed);
else if(axistype==RotorAxis)
        moto_SettingJ(4,distance,speed);
else if(axistype==SwingAxis)
        moto_SettingJ(5,distance,speed);
else if(axistype==ModifiedGear1)
        moto_SettingJ(7,distance,speed);
else if(axistype==ModifiedGear2)
        moto_SettingJ(8,distance,speed);

}

bool FixedLengthMoving::run()    //要加是否处于限位的思路
{

	if(axistype==UnKnown)
		return false;
	if(!runing)
		return false;
	cout<<axistype<<"轴移动开始"<<endl;
	SingleAxismove();//单轴移动
	DylCommon::protocol_send("AA,3");
	cout<<axistype<<"轴移动结束"<<endl;
	runing=false;
	return true;
}


