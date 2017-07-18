/*
 * Parser.h
 *
 *  Created on: 2016年11月15日
 *      Author: Administrator
 */

#ifndef SRC_PARSER_H_
#define SRC_PARSER_H_
#include <iostream>
using namespace std;
#include <vector>
#include "FixedLengthMoving.h"
#include "RobotBack.h"
#include "Coordinates.h"
#include "RoctorBar.h"
#include "Welding.h"
#include "Parameter.h"
#include "IOparameter.h"
#include "Torchcalibration.h"
#include "ModifiedGearDemarcate.h"
#include "MD5.h"
#include "moto.h"
#include <string>    //用这种可以调节速度控制
class Parser
{
public:
	Parser();
	virtual ~Parser();
	//总的解析函数
	void DateParser(const string&);
	void run();
	bool  back_finished;
	bool  IsConnnect;
	bool runing;//标志位
private:
	//数据接收
    void Datereceive();
    //校验函数
    bool check();
    //cmd解析函数
    void cmdparser();

    string sdata;

    bool IsClear;
    bool IsCutting;//是否能够切割
    vector<double> parserdata;        //数据
    string cmd;
    string checkValue;
    string checkstr;
    ArrayXd initXyzrpw;   //清零后的坐标值

    /***********关节式解析函数***************/
    void parserReceviedCoordPoints();      //接收解析上位机下传的点位数据

public:
    FixedLengthMoving  fixposmove;    //定长移动
    Robot_Back       robotback;          //回零
    Coordinates      coord;  //坐标
    IOparameter   ioparameter;//IO参数
    RoctorBar        roctorbar;   //摇杆
    Welding          welding;     //焊接
    Torchcalibration torchcalibration;
    ModifiedGearDemarcate modifiedGearDemarcate; //变位机构
    bool  IsSend;    //是否处于数据下传中
    void changeRPW(ArrayXd &xyzrpw);
};

#endif /* SRC_PARSER_H_ */
