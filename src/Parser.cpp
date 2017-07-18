/*
 * Parser.cpp
 *
 *  Created on: 2016年11月15日
 *      Author: Administrator
 */
#include "Parser.h"
#include <semaphore.h>
#include <math.h>
#include <cstring>
#include "DlyCommon.h"
#include "Parameter.h"

extern string lastcmd;
extern Parameter structParameter;
Parser::Parser() :
		sdata(""), back_finished(false), IsConnnect(true), runing(false), IsClear(
				false), IsCutting(false) {
	IsSend = false;
}

Parser::~Parser() {

}

void Parser::Datereceive() {
	if (*(sdata.begin()) == '[')
		sdata.erase(sdata.begin()); //去掉头'['
	if (*(sdata.end() - 1) == ']')
		sdata.erase(sdata.end() - 1); //去掉尾']'
	vector<string> strArray = DylCommon::split(sdata, ","); //拆分
	for (int i = 0; i < (int) strArray.size() - 2; i++)
		checkstr += (strArray[i] + ",");
	checkstr += strArray[strArray.size() - 2];
	cmd = strArray[0]; //得到命令字符
	for (int i = 1; i < (int) strArray.size() - 1; i++)
		parserdata.push_back(strtod(strArray[i].c_str(), NULL)); //string转float
	checkValue = *(strArray.end() - 1);
}

//校验函数
bool Parser::check() {
	if (parserdata.size() < 1)
		return false;
	MD5 md5;
	md5.reset();
	md5.update(checkstr);
	string str = md5.toString();
	checkstr = "";
	if ((parserdata[0] == parserdata.size() + 2) && str == checkValue)
		return true;
	else
		return false;
}
void Parser::run() {
	updateRun();	// 升级更新程序
	if (sdata == "" || cmd == "")
		return;
	//没有握手
	if (runing == false)
		return;
	robotback.run();  //回零执行
	fixposmove.run(); //定长移动执行
	roctorbar.run();  //摇杆这里只是不断计算数值，运动在另外一个线程
	welding.run();    //焊接运行
	torchcalibration.run(); //焊枪标定
	modifiedGearDemarcate.run(); //变为机构运行；
}

void Parser::cmdparser() {

	//解析数据
	/**********************E系列指令解析*******************/
if (cmd == "E0")
{
	cout << "握手成功" << endl;
	DylCommon::protocol_send("E0,3");
	runing = true;
	return;
}
else if (cmd == "E1")
{
	cout << "结构参数下传" << endl;
	//+结构参数设置数据
	structParameter.setData(parserdata[1], parserdata[2], parserdata[3],
			parserdata[4]);
	return;
} else if (cmd == "E2")
{

	cout << "脉冲当量下传" << endl;
	//脉冲当量设置数据
	J1PUPR = parserdata[1];
	J2PUPR = parserdata[2];
	J3PUPR = parserdata[3];
	J4PUPR = parserdata[4];
	J5PUPR = parserdata[5];
	J7PUPR = parserdata[6];
	J8PUPR = parserdata[7];

	return;
} else if (cmd == "E3") {
	PointType pointtype;
	pointtype = PointType((int) parserdata[1]);
	// debug
	cout << "TCF标定:" << endl;
	cout << "标定点：" << pointtype << endl;
	//TCF设置数据
	torchcalibration.setData(pointtype);
	return;
} else if (cmd == "E5") {
	int h = 1;
	Matrix4d mat = Matrix4d::Zero();
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			double s = parserdata[h++];
			if (finite(s) == 0)   //如果传入的值为非数值，则返回出去
			{
				return;
			}
			mat(i, j) = s;
		}
	}
	cout << "TCF矩阵下传" << endl;
	//tcf测试屏蔽
	torchcalibration.setData(mat);

} else if (cmd == "EE") {
	cout << "重传指令" << endl;
	//重传上一条指令
	DylCommon::protocol_send2(lastcmd);
} else if (cmd == "EF") {
	DylCommon::protocol_send1("EF,3");
	IsConnnect = true;

}
else if(cmd == "EB") {
	PointType pointtype;
	pointtype = PointType((int) parserdata[1]);
	// debug
	cout << "POS标定:" << endl;
	cout << "标定点：" << pointtype << endl;
	//TCF设置数据
	modifiedGearDemarcate.setData(pointtype);
	return;
}
else if(cmd == "EC") {
	int h = 1;
	Matrix4d mat = Matrix4d::Zero();
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			double s = parserdata[h++];
			if (finite(s) == 0)   //如果传入的值为非数值，则返回出去
			{
				return;
			}
			mat(i, j) = s;
		}
	}
	cout << "POS矩阵下传" << endl;
	//tcf测试矩阵
	modifiedGearDemarcate.setData(mat);
}

/**********************A系列指令解析----定长移动*******************/

//XAxis,YAxis,BigArm,SmallArm,UpDownAxis,RotorAxis,SwingAxis,ModifiedGear1,ModifiedGear2,FixedPointRotation,
//FixedPointSwing,UnKnown
//定长移动
else if (cmd == "A0")
{
	fixposmove.setData(XAxis, parserdata[1], parserdata[2]);
	return;
}
else if (cmd == "A1")
{
	fixposmove.setData(YAxis, parserdata[1], parserdata[2]);
	return;
}
else if (cmd == "A2")
{
	fixposmove.setData(BigArm, parserdata[1], parserdata[2]);
	return;
}
else if (cmd == "A3")
{
	fixposmove.setData(SmallArm, parserdata[1], parserdata[2]);
	return;
}
else if (cmd == "A4")
{
	fixposmove.setData(UpDownAxis, parserdata[1], parserdata[2]);
	return;
}
else if (cmd == "A5")
{
	fixposmove.setData(RotorAxis, parserdata[1], parserdata[2]);
	return;
}
else if (cmd == "A6")
{
	fixposmove.setData(SwingAxis, parserdata[1], parserdata[2]);
	return;
}
else if (cmd == "A7")
{
	fixposmove.setData(ModifiedGear1, parserdata[1], parserdata[2]);
	return;
}
else if (cmd == "A8")
{
	fixposmove.setData(ModifiedGear2, parserdata[1], parserdata[2]);
	return;
}

/******************B系列指令解析***********************/
else if (cmd == "B0")   //记住此时改过
{
	roctorbar.setData(XAxis, parserdata[1]);
//		if (fabs(parserdata[1]) < 100)
//		Variable::IsStop = true;
	return;
}

else if (cmd == "B1")
{
	roctorbar.setData(YAxis, parserdata[1]);
//		if (fabs(parserdata[1]) < 100)
//		Variable::IsStop = true;
	return;
}

else if (cmd == "B2")
{
	roctorbar.setData(BigArm, parserdata[1]);
	return;
}

else if (cmd == "B3")
{
	roctorbar.setData(SmallArm, parserdata[1]);
	return;
}
else if (cmd == "B4")
{
	roctorbar.setData(ZAxis, parserdata[1]);
//		if (fabs(parserdata[1]) < 100)
//		Variable::IsStop = true;
	return;
}

else if (cmd == "B5")
{
	roctorbar.setData(RotorAxis, parserdata[1]);  //同一速度，摆动轴和旋转轴是有区别滴
	return;
}

else if (cmd == "B6")
{
	roctorbar.setData(SwingAxis, parserdata[1]);
	return;
}

else if (cmd == "B7")
{
	roctorbar.setData(ModifiedGear1, parserdata[1]);
	return;
}
else if (cmd == "B8")
{
	roctorbar.setData(ModifiedGear2, parserdata[1]);
	return;
}
else if (cmd == "B9")
{

	roctorbar.setData(FixedPointRotation, parserdata[1]);
//		if (fabs(parserdata[1]) < 100)
//		Variable::IsStop = true;
	return;
}
else if (cmd == "BA")
{
	roctorbar.setData(FixedPointSwing, parserdata[1]);
//		if (fabs(parserdata[1]) < 100)
//		Variable::IsStop = true;
	return;
}
//升降轴
else if (cmd == "BB")
{
	roctorbar.setData(UpDownAxis, parserdata[1]);
	return;
}

else if(cmd == "BE")   //变位机构1摇杆指令
{
	roctorbar.setData(ModifiedGear1Joy, parserdata[1]);
	return;
}
else if(cmd == "BF")  //变位机构2摇杆指令
{
	roctorbar.setData(ModifiedGear2Joy, parserdata[1]);
	return;
}
//接收到BC指令后数据清零示教过程中
else if (cmd == "BC")
{
	moto_XYZclear();  //XYZ数据清零
	usleep(400000);
	DylCommon::protocol_send("BC,3");//接受到BC后就下传坐标
}
//焊接过程中需要清零
else if (cmd == "BD")
{
	IsClear = true;
	IsCutting = true;
}


/**********************C系列指令解析*******************/
//回零指令解析
else if (cmd == "C0")
{
	robotback.setData(1, parserdata[1], parserdata[2]);
	return;
}

else if (cmd == "C1")
{
	robotback.setData(2, parserdata[1], parserdata[2]);
	return;
}
else if (cmd == "C2")
{
	robotback.setData(3, parserdata[1], parserdata[2]);
	return;
}
else if (cmd == "C3")
{
	robotback.setData(4, parserdata[1], parserdata[2]);
	return;
}
//回零指令解析
else if (cmd == "C4")
{
	robotback.setData(5, parserdata[1], parserdata[2]);
	return;
}
else if (cmd == "C5")
{
	robotback.setData(7, parserdata[1], parserdata[2]);
	return;
}
else if (cmd == "C6")
{
	robotback.setData(8, parserdata[1], parserdata[2]);
	return;
}
else if (cmd == "C7")
{
	cout << "全部回零成功后的点位坐标" << endl;
	moto_init();
	back_finished = true;
	return;
}
else if (cmd == "C8")
{
	if (parserdata[1] == 0)
	Variable::RotorAxisDirectionChange = false;
	else if (parserdata[1] == 1)
	Variable::RotorAxisDirectionChange = true;
}
else if (cmd == "C9")
{
	if (parserdata[1] == 0)
	Variable::SwingAxisDirectionChange = false;
	else if (parserdata[1] == 1)
	Variable::SwingAxisDirectionChange = true;
}
else if (cmd == "CA")
{
	if (parserdata[1] == 0)
	Variable::ModifiedGear1DirectionChange = false;
	else if (parserdata[1] == 1)
	Variable::ModifiedGear1DirectionChange = true;
}
else if (cmd == "CB")
{
	if (parserdata[1] == 0)
	Variable::ModifiedGear2DirectionChange = false;
	else if (parserdata[1] == 1)
	Variable::ModifiedGear2DirectionChange = true;
}

/************************D系列指令解析*************************/
else if (cmd == "D1")
{
	parserReceviedCoordPoints();   //接收解析上位机下传的点位数据
	return;
}
else if (cmd == "D2")
{
	cout << "起弧行弧指令" << endl;
	//把起弧行弧参数设置进去   延时和标号的作用
	welding.receiveArcStrickData(parserdata[1],parserdata[6]);
	return;
}
else if (cmd == "D3")
{
	cout << "熄弧指令" << endl;
	welding.receiveArcQuenchData(parserdata[3]);
	return;
}
else if (cmd == "D4")    //开始下传的指令
{
	cout << "开始下传图形数据" << endl;
	IsSend = true;    //发送过程中是不进行校验心跳指令的
	welding.init();
	DylCommon::protocol_send("D4,3");
	return;
}
else if (cmd == "D5")
{
	//下传结束可以开始解析
	if(IsClear)
	{
		initXyzrpw = welding.graph[0].startXyzrpw;     //把文件中第一个点保存下来
		IsClear = false;
	}
	welding.getInterpolations(); //解析数据点
	IsSend = false;
	DylCommon::protocol_send("D5,3");
	return;
}
else if (cmd == "D7")
{

	cout << "开始焊接" << endl;
	if (IsCutting)
	{
		Joint j;
		Coint c;
		DylCommon::getCurrentPosition(j, c);
		Matrix4d mat = fksolution(j);
		ArrayXd xyzrpw = pose_2_xyzrpw(mat);
		for (int i = 3; i < 6; i++)
		xyzrpw[i] = initXyzrpw[i];  //把初始值的rpw复制过来
		Matrix4d mat1 = xyzrpw_2_pose(xyzrpw);
		j = NewPositionJointssolution(mat1);
		cout << "改变姿态" << endl;
		moto_runJAbs(j, c, 4000);
		cout << "改变姿态完成" << endl;
		moto_XYZclear();//XYZ数据清零
		IsCutting = false;
		usleep(1000);
	}
	welding.runing = true;  //开始运行
	return;
}
else if (cmd == "D8") //暂停指令
{
	Variable::IsStop = true;
	return;
}
else if (cmd == "D9") //停止指令
{
	Variable::IsStop = true;
	cout << "停止运动" << endl;
	return;
}
else if (cmd == "DA")
{
	cout << "速度调整指令" << endl;
	welding.rate = parserdata[1] / 9999.0;
	return;
}
else if (cmd == "DG")
{
	cout << "模式切换" << endl;
	welding.IsFireMode = parserdata[1];   //焊接模式的状态
	return;
}
else if (cmd == "DH")
{
	cout << "后退指令" << endl;
	welding.backruning = true;
	return;
}
/***************F系列指令解析***********************/
else if (cmd == "F0")
{
	cout << "IO口指令下传" << endl;
	ioparameter.setData(parserdata[1]);
	return;
}
else if (cmd == "F5")
{
	cout << "系统恢复指令" << endl;
	DylCommon::protocol_send("F5,3");
	return;
} else if (cmd == "F7")
{
	cout << "版本号显示指令" << endl;
	DylCommon::protocol_send("F7,4," + version);
	return;
}

//************************** G 指令解析 ************************* /
//最大速度的测试 传到底层进行处理的

else if (cmd == "G1")
{
	Parameter::SingleAxisMaxJspeed[0] = parserdata[1];
	cout << "X轴最大速度下传：" << parserdata[1] << endl;
	return;
}

else if (cmd == "G2")
{
	Parameter::SingleAxisMaxJspeed[1] = parserdata[1];
	cout << "Y轴最大速度下传：" << parserdata[1] << endl;
	return;
}
else if (cmd == "G3")
{
	Parameter::SingleAxisMaxJspeed[2] = parserdata[1];
	cout << "Z轴最大速度下传：" << parserdata[1] << endl;
	return;
}
else if (cmd == "G4")
{
	Parameter::SingleAxisMaxJspeed[3] = parserdata[1];
	cout << "A轴最大速度下传：" << parserdata[1] << endl;
	return;
}
else if (cmd == "G5")
{
	Parameter::SingleAxisMaxJspeed[4] = parserdata[1];
	cout << "B轴最大速度下传：" << parserdata[1] << endl;
	return;
}

else if (cmd == "G6")
{
	for (int i = 1; i <= 5; i++)
	{
		if (parserdata[i] <= 0 && finite(parserdata[i]))
		{
			cout << "最大速度下传数据不对" << endl;
			return;
		}
		Parameter::SingleAxisMaxJspeed[i - 1] = parserdata[i];
	}
	for (int i = 6; i <= 10; i++)
	{
		if (parserdata[i] <= 0 && finite(parserdata[i]))
		{
			cout << "摇杆合适速度下传数据不对" << endl;
			return;
		}
		Parameter::JoyMaxJspeed[i - 6] = parserdata[i];
	}
	return;
}

else if (cmd == "G7")
{
	Parameter::JoyMaxJspeed[0] = parserdata[1];

	cout << "X轴摇杆合适速度下传：" << parserdata[1] << endl;
	return;
}
else if (cmd == "G8")
{
	Parameter::JoyMaxJspeed[1] = parserdata[1];
	cout << "y轴摇杆合适速度下传：" << parserdata[1] << endl;
	return;
}
else if (cmd == "G9")
{
	Parameter::JoyMaxJspeed[2] = parserdata[1];
	cout << "z轴摇杆合适速度下传：" << parserdata[1] << endl;
	return;
}
else if (cmd == "GA")
{
	Parameter::JoyMaxJspeed[3] = parserdata[1];
	cout << "定点转摇杆合适速度下传：" << parserdata[1] << endl;
	return;
}
else if (cmd == "GB")
{
	Parameter::JoyMaxJspeed[4] = parserdata[1];
	cout << "定点摆摇杆合适速度下传：" << parserdata[1] << endl;
	return;
}

else if(cmd == "GC")
{
	Parameter::SingleAxisMaxJspeed[6] = parserdata[1];
	cout << "变位机构1轴最大速度下传：" << parserdata[1] << endl;
	return;
}

else if(cmd == "GD")
{
	Parameter::SingleAxisMaxJspeed[7] = parserdata[1];
	cout << "变位机构2轴最大速度下传：" << parserdata[1] << endl;
	return;
}

else if(cmd == "GE")
{
	Parameter::JoyMaxJspeed[6] = parserdata[1];
	cout << "变位机构1速度下传：" << parserdata[1] << endl;
    return;
}  //变位机构2速度
else if(cmd == "GF")
{
	Parameter::JoyMaxJspeed[7] = parserdata[1];
	cout << "变位机构2速度下传：" << parserdata[1] << endl;
    return;
}
}

//数据解析
void Parser::DateParser(const string &str) {
sdata = str;
//接收数据
Datereceive();
	if (cmd != "EF")
	{
		cout << cmd << " ";
	    for (int i = 0; i < (int) parserdata.size(); i++)
		cout << parserdata[i] << " ";   //把数据打印出来
	    cout << endl;
	}
if (!check()) //校验不通过
{
	string str;
	if (cmd == "D1")    //图形下传中校验失败
			{
		str = "D6,3";
		welding.init();
		cout << "图形数据错误，请求重传 " << endl;
	} else {
		str = "EE,3";    //一般数据校验失败
		cout << "其他数据错误，请求重传" << endl;
	}
	usleep(100);
	DylCommon::protocol_send1(str);
	cmd = "";
	return;
}

//解析数据
Variable::IsStop = false;
cmdparser();
parserdata.clear();
}

/***********关节式解析函数***************/
//接收解析上位机下传的点位数据
void Parser::parserReceviedCoordPoints() {

//直线     D1 0   X1 Y1 Z1 R1 P1 W1 U1 V1     X2 Y2 Z2 R2 P2 W2 U2 V2 S N
//圆弧     D1 4   X1 Y1 Z1 R1 P1 W1 U1 V1  X Y Z R P W U V  X2 Y2 Z2 R2 P2 W2 U2 V2 S N
ArrayXd xyzrpw1 = ArrayXd::Zero(6);      //起点
Coint c1 = { 0, 0 };
ArrayXd xyzrpw2 = ArrayXd::Zero(6);
Coint c = { 0, 0 };
ArrayXd xyzrpw = ArrayXd::Zero(6);
Coint c2 = { 0, 0 };
for (int i = 2; i < 8; i++)          //起点
	xyzrpw1[i - 2] = parserdata[i];
c1.c1 = parserdata[8] * pi / 180;
c1.c2 = parserdata[9] * pi / 180;
for (int i = 10; i < 16; i++)        //第二个点
	xyzrpw2[i - 10] = parserdata[i];
c2.c1 = parserdata[16] * pi / 180;
c2.c2 = parserdata[17] * pi / 180;
double speed = parserdata[18];
int num = parserdata[19];
if (parserdata[1] == 4 || parserdata[1] == 5)   //圆弧
		{
	xyzrpw = xyzrpw2;  //把第二点定为中点
	c = c2;
	for (int i = 18; i < 24; i++)    //第三个点 为末点
		xyzrpw2[i - 18] = parserdata[i];
	c2.c1 = parserdata[24] * pi / 180;
	c2.c2 = parserdata[25] * pi / 180;
	speed = parserdata[26];
	num = parserdata[27];
}
switch ((int) parserdata[1])  //根据所传类型来保存数据点       0--空移线   1--插补线  4--圆弧线
{
case 0:
	welding.receiveLinePoints(xyzrpw1, c1, xyzrpw2, c2, speed, num, false);
	break;
case 1:
	welding.receiveLinePoints(xyzrpw1, c1, xyzrpw2, c2, speed, num, true);
	break;
case 4:
	welding.receiveArcPoints(xyzrpw1, c1, xyzrpw, c, xyzrpw2, c2, speed, num);
	break;
case 5:
	welding.receiveCirclePoints(xyzrpw1, c1, xyzrpw, c, xyzrpw2, c2, speed, num);
//	welding.receiveCircle1Points(xyzrpw1, c1, xyzrpw, c, xyzrpw2, c2, speed, num);
	break;
}
}

void Parser::changeRPW(ArrayXd &xyzrpw) {
for (int i = 3; i < 6; i++) {
	if (xyzrpw[i] < 0)
		xyzrpw[i] += 360;
}

}
