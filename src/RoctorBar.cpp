/*
 * RoctorBar.cpp
 *
 *  Created on: 2016年11月16日
 *      Author: Administrator
 */

#include "RoctorBar.h"
#include "Parameter.h"
RoctorBar::RoctorBar() :
		axistype(UnKnown), speed(0), runing(false), Joyruning(false), joyruntime(
				0), lastspeed(0), rate(1) {
	targetJ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	targetC = {0,0};
	lastV = {0,0,0};
	lastA = {0,0,0};
	IsFixedPoint = false;

	x = 0;
	y = 0;
	z = 0;
}

RoctorBar::~RoctorBar() {

}
void RoctorBar::clearSpeed() {

	XAxisSpeed = 0;
	YAxisSpeed = 0;
	ZAxisSpeed = 0;
	FixedPointRotationSpeed = 0;
	FixedPointSwingSpeed = 0;
	ModifiedGear1JoySpeed = 0;
	ModifiedGear2JoySpeed = 0;
	this->speed = 0;

}
void RoctorBar::setData(AxisType axistype, double speed) {
	clearSpeed();
	this->axistype = axistype;
	rate = fabs(speed / 6000.0);  //以前是6000
	if (rate > 1)
		rate = 1;
	//开始就轴进去，这样好开始运动
	if (axistype == XAxis) {
		XAxisSpeed = speed;
	}
	if (axistype == YAxis) {
		YAxisSpeed = speed;
	}
	if (axistype == ZAxis) {
		ZAxisSpeed = speed;
	}
	if (axistype == FixedPointRotation) {
		FixedPointRotationSpeed = speed;
}
if (axistype == FixedPointSwing)
{
	FixedPointSwingSpeed = speed;

}
if(axistype == FixedPointSwing || axistype == FixedPointRotation)
{

	if(!IsFixedPoint)
		{
		    IsFixedPoint = true;
			Joint j;
			Coint c;
			DylCommon::getCurrentPosition(j,c);
			ArrayXd xyzrpw = pose_2_xyzrpw(fksolution(j));
			x = xyzrpw[0];
			y = xyzrpw[1];
			z = xyzrpw[2];
		}
}
else
{
	IsFixedPoint = false;
}
if (axistype == ModifiedGear1Joy)
{
	ModifiedGear1JoySpeed = speed;
}
if (axistype == ModifiedGear2Joy)
{
	ModifiedGear2JoySpeed = speed;   //速度也留下来的
}
else
{
	this->speed = speed;    //可以及时暂停下来
	if (speed == 0)
	{
		cout << "收到0" << endl;
		Variable::IsStop = true;
	}
}
}

void RoctorBar::RoctorMove()
{

cout << "摇杆移动开始" << endl;
Joyruning = true;
targetJ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  //这个是否有问题
double ss = 1000.0;
//保证不卡可以停
//测出摇杆最大速度     x y z 定点摆 定点转   到50%左右
int ADDTIM1 = 1;
int VVVSSS1 = 0;
long int VVVCCC1;
if (XAxisSpeed != 0)
	VVVCCC1 = Parameter::JoyMaxJspeed[0];
if (YAxisSpeed != 0)
	VVVCCC1 = Parameter::JoyMaxJspeed[1];
if (ZAxisSpeed != 0)
	VVVCCC1 = Parameter::JoyMaxJspeed[2];
if (FixedPointRotationSpeed != 0)
	VVVCCC1 = Parameter::JoyMaxJspeed[3];
if (FixedPointSwingSpeed != 0)
	VVVCCC1 = Parameter::JoyMaxJspeed[4];
if (ModifiedGear1JoySpeed != 0)
	VVVCCC1 = Parameter::JoyMaxJspeed[6];
if (ModifiedGear2JoySpeed != 0)
	VVVCCC1 = Parameter::JoyMaxJspeed[7];
int VADDTIMES1 = 1000;  //加速步数
double VVVAAA1 = ((double) (VVVCCC1 - VVVSSS1) / (VADDTIMES1 - 1) / ADDTIM1);//加速度
while (true) {
	Joint currentJ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	Coint currentC = { 0.0, 0.0 };
	if(robotType == CoordRobot)
	{
	  currentJ.j1 = (double) (MOT->J1step) / J1PUPR;
	  currentJ.j2 = (double) (MOT->J2step) / J2PUPR;
	}
	else
	{
	currentJ.j1 = (double)(MOT->J1step) * pi / 180.0 / J1PUPR;
	currentJ.j2 = (double)(MOT->J2step) * pi / 180.0 / J2PUPR;
	}
	currentJ.j3 = (double) (MOT->J3step) / J3PUPR;
	currentJ.j4 = (double) (MOT->J4step) * pi / 180.0 / J4PUPR;
	currentJ.j5 = (double) (MOT->J5step) * pi / 180.0 / J5PUPR;

	currentC.c1 = (double) (MOT->J7step) * pi / 180.0 / J7PUPR;
	currentC.c2 = (double) (MOT->J8step) * pi / 180.0 / J8PUPR;
	targetC = currentC;
	Vector3d lastV;
	lastV << XAxisSpeed / ss, YAxisSpeed / ss, ZAxisSpeed / ss ;
	Vector3d lastA;
	lastA << 0, FixedPointSwingSpeed / ss, FixedPointRotationSpeed / ss;
	Matrix4d new_robot_position = transl(lastV) * fksolution(currentJ);
	ArrayXd xyzrpw(6);
	usleep(100);
	xyzrpw = pose_2_xyzrpw(new_robot_position);
	xyzrpw[3] += lastA(0);
	xyzrpw[4] += lastA(1);
	xyzrpw[5] += lastA(2);
	if(FixedPointRotationSpeed != 0 || FixedPointSwingSpeed != 0)
	{
		xyzrpw[0] = x;
		xyzrpw[1] = y;
		xyzrpw[2] = z;
	}
	new_robot_position = xyzrpw_2_pose(xyzrpw);
    targetJ = NewPositionJointssolution(new_robot_position);
	if (ModifiedGear1JoySpeed != 0) {
		targetC.c1 += ModifiedGear1JoySpeed / ss;   //掌握这个度
	}
	if (ModifiedGear2JoySpeed != 0) {
		targetC.c2 += ModifiedGear2JoySpeed / ss;   //掌握这个度
	}   //或者一开始就开始加速运动
	if (Variable::IsStop || (XAxisSpeed == 0 && YAxisSpeed == 0 && ZAxisSpeed == 0&& FixedPointRotationSpeed == 0 && FixedPointSwingSpeed == 0&& ModifiedGear1JoySpeed == 0 && ModifiedGear2JoySpeed == 0))
	{
		double hh = 30;
		nowSpeed -= hh;
		if (nowSpeed <= 30)
		{
			robot_stop();
			cout << "stop stop stop" << endl;
			robot_stop();
			break;
		}
	}
		nowSpeed += VVVAAA1 * 0.5;		//0.2 * VVVAAA;
		if (nowSpeed >= VVVCCC1 * rate)
		{
			nowSpeed = VVVCCC1 * rate;
		}
}
cout << "摇杆移动结束" << endl;
targetJ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
targetC = {0,0};
Joyruning = false;
robot_stop();
}

//摇杆的运行
void RoctorBar::RoctorRun() {
while (targetJ.ISOK && Joyruning) {
	moto_runJoy(targetJ, targetC);
}
}

//单轴移动
void RoctorBar::SingleAxisMove() {
double speed1 = fabs(speed);
if (axistype == BigArm)  //大臂
{
	if(robotType == CoordRobot)
	moto_SettingJ(1, speed / (fabs(speed) + 1) * 9999, speed1);
	else
	moto_SettingJ(1, speed / (fabs(speed) + 1) * 720, speed1);
	return;
}
else if (axistype == SmallArm)  //小臂
{
	if(robotType == CoordRobot)
	moto_SettingJ(2, speed / (fabs(speed) + 1) * 9999, speed1);
	else
	moto_SettingJ(2, speed / (fabs(speed) + 1) * 720, speed1);
	return;
}
else if (axistype == UpDownAxis)  //升降轴

{
	moto_SettingJ(3, speed / (fabs(speed) + 1) * 9999, speed1);
	return;
} else if (axistype == RotorAxis)  //旋转轴
		{
	moto_SettingJ(4, speed / (fabs(speed) + 1) * 720, speed1);
	return;
} else if (axistype == SwingAxis)  //摆抢
		{
	moto_SettingJ(5, speed / (fabs(speed) + 1) * 720, speed1);
	return;
} else if (axistype == ModifiedGear1)  //变为机构1
		{
	moto_SettingJ(7, speed / (fabs(speed) + 1) * 720, speed1);
	return;
} else if (axistype == ModifiedGear2)  //变位机构2
		{
	moto_SettingJ(8, speed / (fabs(speed) + 1) * 720, speed1);
	return;
}
}

void RoctorBar::run() {

if (axistype == UnKnown)
	return;
//定长移动
else if (axistype == XAxis || axistype == YAxis || axistype == ZAxis
		|| axistype == FixedPointRotation || axistype == FixedPointSwing
		|| axistype == ModifiedGear1Joy || axistype == ModifiedGear2Joy) {
	RoctorMove();
	axistype = UnKnown;
	return;
} else
	SingleAxisMove();
axistype = UnKnown;
}
