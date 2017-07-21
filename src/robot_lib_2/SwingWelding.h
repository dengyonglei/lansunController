/*
 * SwingWelding.h
 *
 *  Created on: 2017年7月20日
 *      Author: deng
 */

#ifndef ROBOT_LIB_2_SWINGWELDING_H_
#define ROBOT_LIB_2_SWINGWELDING_H_
#include "scaralib.h"
#include <list>
#include <vector>
class SwingWelding {
public:
	SwingWelding();
	 ~SwingWelding();
	 static vector<Quaterniond> Position3DInterpolationTri(const Vector3d& pa,const Vector3d& pb,const Vector3d& pc,const Vector3d& pd,const Vector3d& pe,const Vector3d& pf,vector<Vector3d> & pMN,int n, Quaterniond * qa, Quaterniond * qc);
	 static vector<Quaterniond> PositionLinearInterpolationTri(const Vector3d& pa,const Vector3d& pb,const Vector3d& pc,const Vector3d& pd,vector<Vector3d> & pMN,int n, Quaterniond * qa, Quaterniond * qd);
	 //输入A,B两点位姿，得到AB两点间直线的位姿序列
	 static list<MJCoint> PositionAttitudeLinearInterpolationTri(const Matrix4d& A, const Matrix4d& B,const Matrix4d& C, const Matrix4d& D,int n);
	 static list<MJCoint> PositionAttitude3DInterpolationTri(const Matrix4d& A, const Matrix4d& B,const Matrix4d& C, const Matrix4d& D,const Matrix4d& E, const Matrix4d&F ,int n);
};

#endif /* ROBOT_LIB_2_SWINGWELDING_H_ */
