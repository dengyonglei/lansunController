/*
 * Variable.cpp
 *
 *  Created on: 2017年5月3日
 *      Author: deng
 */

#include "Variable.h"
bool Variable::IsStop = false;
bool Variable::IsSingleAxisStop = false;
bool Variable::RotorAxisDirectionChange = false;
bool Variable::SwingAxisDirectionChange = false;
bool Variable::ModifiedGear1DirectionChange = false;
bool Variable::ModifiedGear2DirectionChange = false;
bool Variable::BigArmDirectionChange = false;
bool Variable::SmallArmDirectionChange = false;
bool Variable::UpDownAxisDirectionChange = false;       //旋转轴方向改变
bool Variable::Islimit = false;
bool Variable::IsAxisStop = false;

