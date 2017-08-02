/*
 * Variable.h
 *
 *  Created on: 2017年5月3日
 *      Author: deng
 */

#ifndef ROBOT_CLASS_VARIABLE_H_
#define ROBOT_CLASS_VARIABLE_H_
class Variable
{
public:
	static bool IsStop;                         //暂停
	static bool IsSingleAxisStop;
	static bool RotorAxisDirectionChange;       //回零方向改变
	static bool SwingAxisDirectionChange;       //旋转轴方向改变
	static bool ModifiedGear1DirectionChange;
	static bool ModifiedGear2DirectionChange;
	static bool BigArmDirectionChange;
	static bool SmallArmDirectionChange;
	static bool UpDownAxisDirectionChange;       //旋转轴方向改变
	static bool Islimit;                        //是否在限位位置

};

#endif /* ROBOT_CLASS_VARIABLE_H_ */
