/*
 * RobotBack.cpp
 *
 *  Created on: 2016年11月16日
 *      Author: Administrator
 */

#include "RobotBack.h"
#include "moto.h"
#include "DlyCommon.h"
double Zmovedistance=0;
Robot_Back::Robot_Back():speed(0),AxisNumber(0),distance(0),runing(false)
{


}


Robot_Back::~Robot_Back()
{

}

void Robot_Back::setData(int AxisNumber,double speed,double distance)
{
	this->AxisNumber=AxisNumber;
	this->speed=speed;
	this->distance=distance;
	runing=true;
}

//单轴回零



void Robot_Back::SingleAxisBack()
{
	cout<<AxisNumber<<"轴回零开始"<<endl;
	double rate=speed/6000.0;
	string str;
	switch(AxisNumber)
	{

		case 1:  J1RunToLimit(1,1000*rate);  if(Variable::IsStop) return; sleep(1);  J1RunToLimit(1,50*rate); if(Variable::IsStop) return;sleep(1);moto_SettingJ(1,distance,5000  * rate);   str="C0,3"; break;
		case 2:  J1RunToLimit(2,600*rate);   if(Variable::IsStop) return; sleep(1);  J1RunToLimit(2,12*rate); if(Variable::IsStop) return;sleep(1);moto_SettingJ(2,distance,5000  * rate);   str="C1,3"; break;
		case 3:  J1RunToLimit(3,500*rate);   if(Variable::IsStop) return; sleep(1);  J1RunToLimit(3,50*rate); if(Variable::IsStop) return;sleep(1);moto_SettingJ(3,distance,5000  * rate);   str="C2,3"; break;
		case 4:  J1RunToLimit(4,500*rate);   if(Variable::IsStop) return; sleep(1);  J1RunToLimit(4,30*rate); if(Variable::IsStop) return; sleep(1);moto_SettingJ(4,distance,5000 * rate); str="C3,3"; break;
		case 5:  J1RunToLimit(5,1000*rate);  if(Variable::IsStop) return; sleep(1);  J1RunToLimit(5,50*rate); if(Variable::IsStop) return; sleep(1);moto_SettingJ(5,distance,5000 * rate);  str="C4,3"; break;
		//变位机构做处理变位机构
		case 7:  J1RunToLimit(7,2000*rate);  if(Variable::IsStop) return; sleep(1);   J1RunToLimit(7,100*rate); if(Variable::IsStop) return;sleep(1); moto_SettingJ(7,distance,5000 * rate); str="C5,3";break;
		case 8:  J1RunToLimit(8,2000*rate);  if(Variable::IsStop) return; sleep(1);   J1RunToLimit(8,100*rate); if(Variable::IsStop) return;sleep(1); moto_SettingJ(8,distance,5000 * rate); str="C6,3";break;
	}
	DylCommon::protocol_send(str);
	cout<< AxisNumber <<"轴回零结束"<<endl;
    return ;
}
bool Robot_Back::run()
{
	if(!runing)
	return false;
    else if(AxisNumber == 0)
	return false;
//每个轴回零运动
   SingleAxisBack();
   runing=false;
   return true;
}
