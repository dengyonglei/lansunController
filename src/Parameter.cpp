/*
 * Parameter.cpp
 *
 *  Created on: 2016年11月17日
 *      Author: Administrator
 */

#include "Parameter.h"
#include "common.h"
long int Parameter::SingleAxisMaxJspeed[8] = {120000,120000,120000,100000,100000,100000,24000,24000};   //上面小机器人
long int  Parameter::JoyMaxJspeed[8] = {80000,6000,80000,24000,8000,12000,24000,24000};   //
//  long int Parameter::SingleAxisMaxJspeed[8] = {100000,70000,60000,30000,100000,24000,24000,24000};    //下面龙门式机器人
//  long int Parameter::JoyMaxJspeed[8] =       {100000,90000,60000,30000,72000,12000,24000,24000};   //摇杆能转动的最大值
Parameter::Parameter()
{

}

Parameter::~Parameter()
{

}
void Parameter::setData(double bigArmLength,double smallArmLength,double weldingTorchLength,double reservedLength)
{
     this->bigArmLength=bigArmLength;
     this->smallArmLength=smallArmLength;
     this->weldingTorchLength=weldingTorchLength;
     this->reservedLength=reservedLength;
}



