/*
 *  poseOfWeldingGun.h
 *
 *  Created on: 2016年11月21日
 *      Author: Bing_yao
 */
/*
 * 此文件  针对  由底层库v1.0版本 升级为底层库 V2.0版本
 * 焊枪 标定模块 独立 所做的处理
 * 注：底层库v1.0 正逆解中位姿 为焊枪末端
 * 	  底层库v2.0     正逆解中位姿 为地五关节轴末端
 * */

#ifndef LANSUNV2_0_SRC_POSEOFWELDINGGUN_H_
#define LANSUNV2_0_SRC_POSEOFWELDINGGUN_H_

#include "scaralib.h"
#include "Torchcalibration.h"

extern Matrix4d CHPA;
// 正解： 包含 焊枪标定值
Matrix4d fksolution(Joint j);
// 逆解： 包含 焊枪标定值
Matrix4d fksolutionPose(Joint j);//标定逆解
Joint iksolution(Matrix4d Matrix, bool choose_j2);
// 封装逆解： 包含 焊枪标定值
Joint NewPositionJointssolution(Matrix4d Matrix);
// 正解： 包含 焊枪标定值
Matrix4d fksolutionBar(Joint j);
// 逆解： 包含 焊枪标定值
Joint iksolutionBar(Matrix4d Matrix, bool choose_j2);
// 封装逆解： 包含 焊枪标定值
Joint NewPositionJointssolutionBar(Matrix4d Matrix);
// 位置 旋转矩阵
Matrix4d transl(Vector3d p);
// 变位体原点
void getCHPosAttitudesolution(Matrix4d Moe1, Matrix4d Moe2, Matrix4d Moe3, Matrix4d Moe4, Matrix4d Moe5, Matrix4d Moe6);

list<MJCoint> PositionAttitudeLinearInterpolationsolution(Matrix4d A, Matrix4d B);

#endif /* LANSUNV2_0_SRC_POSEOFWELDINGGUN_H_ */
