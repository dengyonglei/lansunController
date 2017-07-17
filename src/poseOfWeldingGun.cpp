/*
 *  poseOfWeldingGun.cpp
 *
 *  Created on: 2016年11月21日
 *      Author: Bing_yao
 */
#include "poseOfWeldingGun.h"

using namespace std;

Matrix4d CHPA = Matrix4d::Zero();

// 正解： 包含 焊枪标定值
Matrix4d fksolution(Joint j)
{
    if(robotType == CoordRobot)
	return coo_fk(j) * TCFmatrix;
    else
    return fk(j) * TCFmatrix;
}

Matrix4d fksolutionPose(Joint j)
{
	if(robotType == CoordRobot)
	return coo_fk(j);
	else
	return fk(j);
}
// 逆解： 包含 焊枪标定值
Joint iksolution(Matrix4d Matrix, bool choose_j2){

	Matrix4d temp = Matrix4d::Zero();
	temp = Matrix * TCFmatrix.inverse();
	if(robotType == CoordRobot)
	return coo_ik(temp);
	else
    return NewPositionJoints(temp);
}
Joint NewPositionJointssolution(Matrix4d Matrix,Joint lastJ)
{
	Matrix4d temp = Matrix4d::Zero();
	temp = Matrix * TCFmatrix.inverse();
	if(robotType == CoordRobot)
	return NewPositionJoints_coo(temp,lastJ);
	else
	return NewPositionJoints(temp);
}
Joint NewPositionJointssolution(Matrix4d Matrix)
{
	Matrix4d temp = Matrix4d::Zero();
	temp = Matrix * TCFmatrix.inverse();
	if(robotType == CoordRobot)
	return coo_ik(temp);
	else
	return NewPositionJoints(temp);
}
Matrix4d fksolutionBar(Joint j){
	// 这里到底是 谁先谁后
//	return fk(j) * TCF;
	Matrix4d TCP = Matrix4d::Identity(4,4);
	TCP.block(0,3,3,1) = TCFmatrix.block(0,3,3,1);
	if(robotType == CoordRobot)
	return coo_fk(j) * TCFmatrix;
	else
	return fk(j) * TCFmatrix;
}
// 逆解： 包含 焊枪标定值
Joint iksolutionBar(Matrix4d Matrix, bool choose_j2){

	Matrix4d temp = Matrix4d::Zero();
	Matrix4d TCP = Matrix4d::Identity(4,4);
	TCP.block(0,3,3,1) = TCFmatrix.block(0,3,3,1);
	temp = Matrix * TCP.inverse();
	if(robotType == CoordRobot)
	return coo_ik(temp);
	else
	return NewPositionJoints(temp);
}
Joint NewPositionJointssolutionBar(Matrix4d Matrix){
	Matrix4d temp = Matrix4d::Zero();
	Matrix4d TCP = Matrix4d::Identity(4,4);
	TCP.block(0,3,3,1) = TCFmatrix.block(0,3,3,1);
	temp = Matrix * TCP.inverse();
	if(robotType == CoordRobot)
	return coo_ik(temp);
	else
	return NewPositionJoints(temp);
}


void getCHPosAttitudesolution(Matrix4d Moe1, Matrix4d Moe2, Matrix4d Moe3, Matrix4d Moe4, Matrix4d Moe5, Matrix4d Moe6){

	CHPA = getCHPosAttitude(Moe1, Moe2, Moe3, Moe4, Moe5, Moe6);
}

// 位置旋转矩阵
Matrix4d transl(Vector3d p){
	Matrix4d xyz = Matrix4d::Identity(4,4);
	xyz(0, 3) = p[0];
	xyz(1, 3) = p[1];
	xyz(2, 3) = p[2];
	return xyz;
}
list<MJCoint> PositionAttitudeLinearInterpolationsolution(Matrix4d A, Matrix4d B){

	A = A * TCFmatrix.inverse();
	B = B * TCFmatrix.inverse();

	list<MJCoint> positionAttitudeLinearList = PositionAttitudeLinearInterpolation(A,B);

	list<MJCoint>::iterator it = positionAttitudeLinearList.begin();
	for (; it != positionAttitudeLinearList.end(); it++)
	{
		(*it).m = (*it).m * TCFmatrix;
	}
	return positionAttitudeLinearList;
}
