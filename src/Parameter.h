/*
 * Parameter.h
 *
 *  Created on: 2016年11月17日
 *      Author: Administrator
 */

#ifndef SRC_PARAMETER_H_
#define SRC_PARAMETER_H_
class Parameter         //结构参数
	{
public:
	    Parameter();
		virtual ~Parameter();
		static   long int  SingleAxisMaxJspeed[8];  //脉冲速度  pu/s   弧度每秒
		static   long int  JoyMaxJspeed[8];
private:
		double bigArmLength;       //大臂长度
		double smallArmLength;     //小臂长度
		double weldingTorchLength;   //焊枪长度
		double reservedLength;      //预留长度
public:
	    void setData(double bigArmLength,double smallArmLength,double weldingTorchLength,double reservedLength);
	};
#endif /* SRC_PARAMETER_H_ */
