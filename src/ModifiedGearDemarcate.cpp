/*
 * ModifiedGearDemarcate.cpp
 *
 *  Created on: 2017年6月16日
 *      Author: deng
 */

#include "ModifiedGearDemarcate.h"

ModifiedGearDemarcate::ModifiedGearDemarcate() :pointType(Error), running(false), POS_BOOL(0)
{
	// TODO Auto-generated constructor stub

}

ModifiedGearDemarcate::~ModifiedGearDemarcate() {
	// TODO Auto-generated destructor stub
}

MJCoint ModifiedGearDemarcate::getCurrentJCoint()
{
	MJCoint mjc;
	DylCommon::getCurrentPosition(mjc.j, mjc.c);
	mjc.m = fksolutionPose(mjc.j);
	return mjc;
}

void ModifiedGearDemarcate::setData(PointType pointType) {
	this->pointType = pointType;
	running = true;
}
void ModifiedGearDemarcate::setData(const Matrix4d &pos) {

	cout << "recv pos" << pos<< endl;
	POSITIONER = Matrix4d::Identity();
	POSITIONER(0, 3) = pos(0, 3);
	POSITIONER(1, 3) = pos(1, 3);
	POSITIONER(2, 3) = pos(2, 3);
}
//通过标定6点获取POS
//Matrix4d getPOSMatrix4d(Matrix4d Moe1, Matrix4d Moe2, Matrix4d Moe3, Matrix4d Moe4, Matrix4d Moe5, Matrix4d Moe6);

bool ModifiedGearDemarcate::run()
{
	string strtemp, str;
	if (pointType == Error)
		return false;
	if (!running)
		return false;
	cout << pointType << "POS执行" << endl;
	//SingleAxismove();//单轴移动
	implementGetPOSMatrix();
	cout << pointType << "POS执行完成" << endl;
	running = false;
	return true;
}
void ModifiedGearDemarcate::implementGetPOSMatrix()
{

	string strtemp, str;
	if (pointType == FirstPoint) {
		P1 = getCurrentJCoint().m;
		cout << "P1" << P1 << endl;
		POS_BOOL |= 0x01;
		if (POS_BOOL == 15) //1111位标志
			Iscoplane();

	} else if (pointType == SecondPoint) {
		P2 = getCurrentJCoint().m;
		cout << "P2" << P2 << endl;
		POS_BOOL |= 0x02;
		if (POS_BOOL == 15) //1111位标志
			Iscoplane();

	} else if (pointType == ThirdPoint) {
		P3 = getCurrentJCoint().m;
		cout << "P3" << P3 << endl;
		POS_BOOL |= 0x04;
		if (POS_BOOL == 15) //1111位标志
			Iscoplane();

	} else if (pointType == FourthPoint) {
		P4 = getCurrentJCoint().m;
		cout << "P4" << P4 << endl;
		POS_BOOL |= 0x08;
		if (POS_BOOL == 15) //1111位标志
			Iscoplane();

	} else if (pointType == FifthPoint) {
		if (POS_BOOL != 15) //1111位标志
				{
			cout << "发送弹框5" << endl;
			DylCommon::protocol_send("E7,3");
		} else {
			P5 = getCurrentJCoint().m;
			cout << "P5" << P5 << endl;
			MJCoint mjc;
		}

	} else if (pointType == SixthPoint) {
		if (POS_BOOL != 15) //1111位标志
				{
			cout << "发送弹框6" << endl;
			DylCommon::protocol_send("E7,3");
		} else {
			P6 = getCurrentJCoint().m;
			cout << "P6" << P6 << endl;
		}
		// debug
//		cout << "pos:"<< getCHPosAttitude(P1, P2, P3, P4, P5, P6) << endl;
	}
	else if (pointType == Complete)
	{
		cout << " Complete6" << endl;
		POSITIONER = getCHPosAttitude(P1, P2, P3, P4, P5, P6);
		// debug
		cout << "get pos:" << POSITIONER << endl;
		// 需要修改   发送 POS
		ostringstream os;
		string str = "EC,19";
		string strtemp;
		POS_BOOL = 0;
		P1 = Matrix4d::Zero();
		P2 = Matrix4d::Zero();
		P3 = Matrix4d::Zero();
		P4 = Matrix4d::Zero();
		P5 = Matrix4d::Zero();
		P6 = Matrix4d::Zero();
		int checkValue = 229 + 19;
		for (int i = 0; i < 4; i++) {
			os.clear();
			for (int j = 0; j < 4; j++) {
				os.str("");
				os << POSITIONER(i, j);
				str.append(",").append(os.str());
				checkValue += (int) POSITIONER(i, j);
			}
		}
		cout << "POS上传" << endl;
		cout << str << endl;
		DylCommon::protocol_send(str);
	}
	pointType = Error;
}

void ModifiedGearDemarcate::Iscoplane()
{
	Matrix4d mat = Matrix4d::Ones();
	MatrixXd tmp;
	tmp = P1.block(0, 3, 3, 1);
	mat.block(0, 0, 1, 3) = tmp.transpose();
	tmp = P2.block(0, 3, 3, 1);
	mat.block(1, 0, 1, 3) = tmp.transpose();
	tmp = P3.block(0, 3, 3, 1);
	mat.block(2, 0, 1, 3) = tmp.transpose();
	tmp = P4.block(0, 3, 3, 1);
	mat.block(3, 0, 1, 3) = tmp.transpose();

	if (mat.determinant() - 0.0 < ACCURACY
			&& mat.determinant() - 0.0 > -ACCURACY) {
		// 发送 警告: 同一平面  数据 不可用
		POS_BOOL=3;
		cout << "error : P1,P2,P3,P4 in the same plane" << endl;
		DylCommon::protocol_send("ED,3,");
	}

}
