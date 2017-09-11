/*
 * Torchcalibration.cpp
 *
 *  Created on: 2016年11月26日
 *      Author: Bing_yao
 */

#include "Torchcalibration.h"
#include "moto.h"
Torchcalibration::Torchcalibration() :
		pointType(Error), running(false), conn(0), TCF_BOOL(0) {
	// TODO Auto-generated constructor stub

}

Torchcalibration::~Torchcalibration() {
	// TODO Auto-generated destructor stub
}

MJCoint Torchcalibration::getCurrentJCoint() {
	MJCoint mjc;
	DylCommon::getCurrentPosition(mjc.j, mjc.c);
	mjc.m = fksolutionPose(mjc.j);
	return mjc;
}

void Torchcalibration::setData(PointType pointType) {
	this->pointType = pointType;
	running = true;
}
void Torchcalibration::setData(const Matrix4d &tcf) {

	cout << "recv tcf" << tcf << endl;
	TCFmatrix = Matrix4d::Identity();
	TCFmatrix(0, 3) = tcf(0, 3);
	TCFmatrix(1, 3) = tcf(1, 3);
	TCFmatrix(2, 3) = tcf(2, 3);
}
//通过标定6点获取TCF
//Matrix4d getTCFMatrix4d(Matrix4d Moe1, Matrix4d Moe2, Matrix4d Moe3, Matrix4d Moe4, Matrix4d Moe5, Matrix4d Moe6);

bool Torchcalibration::run() {

	string strtemp, str;

	if (pointType == Error)
		return false;
	if (!running)
		return false;
	cout << pointType << "TCF执行" << endl;
	//SingleAxismove();//单轴移动
	implementGetTCFMatrix();
	cout << pointType << "TCF执行完成" << endl;
	running = false;

	return true;
}
void Torchcalibration::implementGetTCFMatrix() {

	string strtemp, str;
//	Matrix4d TCF = Matrix4d::Zero();

	// debug
//	TCFmatrix = Matrix4d::Identity();

	if (pointType == FirstPoint) {
		P1 = getCurrentJCoint().m;
		cout << "P1" << P1 << endl;
		TCF_BOOL |= 0x01;
		if (TCF_BOOL == 15) //1111位标志
			Iscoplane();

	} else if (pointType == SecondPoint) {
		P2 = getCurrentJCoint().m;
		cout << "P2" << P2 << endl;
		TCF_BOOL |= 0x02;
		if (TCF_BOOL == 15) //1111位标志
			Iscoplane();

	} else if (pointType == ThirdPoint) {
		P3 = getCurrentJCoint().m;
		cout << "P3" << P3 << endl;
		TCF_BOOL |= 0x04;
		if (TCF_BOOL == 15) //1111位标志
			Iscoplane();

	} else if (pointType == FourthPoint) {
		P4 = getCurrentJCoint().m;
		cout << "P4" << P4 << endl;
		TCF_BOOL |= 0x08;
		if (TCF_BOOL == 15) //1111位标志
			Iscoplane();

	} else if (pointType == FifthPoint) {
		if (TCF_BOOL != 15) //1111位标志
				{
			cout << "发送弹框5" << endl;
			DylCommon::protocol_send("E7,3");
		} else {
			P5 = getCurrentJCoint().m;
			cout << "P5" << P5 << endl;
			MJCoint mjc;
//		  DlyCommon::getCurrentPosition(mjc.j, mjc.c);
//		  //快速移动到第四点进行第六点标定
//		  moto_runJAbs(mjc.j,mjc.c);
		}

	}
	else if (pointType == SixthPoint)
	{
		if (TCF_BOOL != 15) //1111位标志
		{
			cout << "发送弹框6" << endl;
			DylCommon::protocol_send("E7,3");
		}
		else
		{
			P6 = getCurrentJCoint().m;
			cout << "P6" << P6 << endl;
		}
		// debug
//		cout << "tcf:"<< getTCFMatrix(P1, P2, P3, P4, P5, P6) << endl;
	}
	else if (pointType == Complete)
	{
		cout << " Complete6" << endl;
		TCFmatrix = getTCFMatrix(P1, P2, P3, P4, P5, P6);
		// debug
		cout << "get tcf:" << TCFmatrix << endl;
		double x = TCFmatrix(3);
		double y = TCFmatrix(7);
		double z = TCFmatrix(11);
		TCFmatrix <<    1, 0, 0,  x,
						0, 1, 0,  y,
						0, 0, 1,  z,
						0, 0, 0,  1;
		cout << "get tcf:" << TCFmatrix << endl;
		// 需要修改   发送 TCF
		ostringstream os;
		string str = "E5,19";
		string strtemp;
		TCF_BOOL = 0;
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
				os << TCFmatrix(i, j);
				str.append(",").append(os.str());
				checkValue += (int) TCFmatrix(i, j);
			}
		}
		cout << "TCF上传" << endl;
		cout << str << endl;
		DylCommon::protocol_send(str);
	}
	pointType = Error;
}

void Torchcalibration::Iscoplane()
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
		TCF_BOOL=3;
		cout << "error : P1,P2,P3,P4 in the same plane" << endl;
		DylCommon::protocol_send("E4,3");
	}

}
