/*
 * Torchcalibration.h
 *
 *  Created on: 2016年11月26日
 *      Author: Bing_yao
 */

#ifndef LANSUNV2_0_SRC_TORCHCALIBRATION_H_
#define LANSUNV2_0_SRC_TORCHCALIBRATION_H_

#include "common.h"
#include "DlyCommon.h"
//#define 	ACCURACY		0.0000001
#define 	ACCURACY		0.01



enum PointType
{
	 FirstPoint = 1, SecondPoint, ThirdPoint, FourthPoint, FifthPoint, SixthPoint, Complete, Error
};

/*
 * 焊枪标定
 *
 **/
class Torchcalibration {
public:
	Torchcalibration();
	virtual ~Torchcalibration();

	Matrix4d m_TCF = Matrix4d::Identity();
	MJCoint	CurrentMJCoint;

	// 获取当前JC数据
	MJCoint getCurrentJCoint();
//	Matrix4d implementGetTCFMatrix();
	void implementGetTCFMatrix();
	void setData(PointType);
	void setData(const Matrix4d &tcf);
	bool run();
	char TCF_BOOL;

private:
	void Iscoplane();
	PointType pointType;
	int conn;
	bool running;
	Matrix4d P1, P2, P3, P4, P5, P6;
};

#endif /* LANSUNV2_0_SRC_TORCHCALIBRATION_H_ */
