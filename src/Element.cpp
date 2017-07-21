/*
 * Element.cpp
 *
 *  Created on: 2017年5月3日
 *      Author: deng
 */

#include "Element.h"
#include "robot_lib_2/SwingWelding.h"
#include <fstream>
Element::Element()
{
	// TODO Auto-generated constructor stub
	angle = 180;
	num = 0;
	speed = 3000;
	type = MoveLine;
	startC = {0.0,0.0};
	endC = {0.0,0.0};
	midC = {0.0,0.0};
	startXyzrpw = ArrayXd::Zero(6); //起点
	endXyzrpw = ArrayXd::Zero(6); ;  //末点
	midXyzrpw = ArrayXd::Zero(6); ;  //末点
	interpolationPoints.clear();
	interpolationPointsIndexs.clear();
    Index = 0;
    IsArcStric = false;
    IsQuench = false;
    swingpoints.clear();
   }

Element::~Element()
{
	// TODO Auto-generated destructor stub
}

//直线插补
bool Element::getLineInterpolations()
{
	list<MJCoint> abcdef = RobotAndCHPositionAttitudeLinearInterpolation(startC,endC,xyzrpw_2_pose(startXyzrpw),xyzrpw_2_pose(endXyzrpw));
	    if(checkInterpolations(abcdef))  //校验正确
	    {
	    	interpolationPoints = abcdef;
	    	return true;
	    }
	    else
	    	return false;
}

//圆弧插补
bool Element::getArcInterpolations()
{
	list<MJCoint> abcdef = RobotAndCHPositionAttitude3DInterpolation(startC,midC,endC,xyzrpw_2_pose(startXyzrpw),xyzrpw_2_pose(midXyzrpw),xyzrpw_2_pose(endXyzrpw));
		if(checkInterpolations(abcdef))  //校验正确
		{
			interpolationPoints = abcdef;
			return true;
		}
		else
			return false;

}


//验证插补点是否正确
bool Element::checkInterpolations(list<MJCoint> abcdef)
{
	int i = 0;
	list<MJCoint>::iterator iter = abcdef.begin();
    if(!lastJIsinit)
    {
       lastJ = NewPositionJointssolution((*iter).m);
	   lastJIsinit = true;
    }
	for (; iter != abcdef.end();iter++)
	{
		(*iter).j = NewPositionJointssolution((*iter).m,lastJ);
		lastJ = (*iter).j;
		i++;
		if ((!(*iter).j.ISOK))
		{
			throw MobileTransfinite;  //抛出移动超限的异常
			return false;
		}
		MJCPoint p;
		p.mj = *iter;
		p.index = i;
		interpolationPointsIndexs.push_back(p);
		}
		return true;
}
void Element::getMoveLineInterpolations()
{
	MJCoint startmj,endmj; //把起点和末点的插补线段都弄进去
	startmj.j = NewPositionJointssolution(xyzrpw_2_pose(startXyzrpw)); //把最后一个点弄进去
	startmj.c = startC;
	endmj.j = NewPositionJointssolution(xyzrpw_2_pose(endXyzrpw)); //把最后一个点弄进去
	endmj.c = endC;
	 if(!lastJIsinit)
	{
	   lastJ = endmj.j;
	   lastJIsinit = true;
	}
	MJCPoint p1,p2;
	p1.mj = startmj;
	p1.index = 1;
	p2.mj = endmj;
	p2.index = 2;
	list<MJCPoint> abcdef;
	abcdef.clear();
	abcdef.push_back(p1);       //把第一个点加进去
	abcdef.push_back(p2);         //把最后一个点加进去
	interpolationPointsIndexs = abcdef;   //当前的任务数

}

bool Element::getInterpolations()   //得到插补点
{
	try
    {
		switch(type)
		{
		  case MoveLine: getMoveLineInterpolations(); break;     //空移线
		  case FireLine: getLineInterpolations();     break;     //焊接线
		  case Arc:      getArcInterpolations();      break;     //得到圆弧插补线
		  case SwingLine:getSwingLineInterpolations();break;   //得到摆焊线
		  default: break;
		}
    }
	catch(...)
	{
		cout << "移动超限" << endl;                               //移动超限
		DylCommon::protocol_send("E6,3");                        //发送数据
	    return false;
	}
	return true;
}


double Element::getTwoLinesAngle(Element &line1,Element &line2)      //得到两条线的夹角
{
		Vector3f v1(line1.endXyzrpw[0] - line1.startXyzrpw[0],line1.endXyzrpw[1] - line1.startXyzrpw[1],line1.endXyzrpw[2] - line1.startXyzrpw[2]);
		Vector3f v2(line2.endXyzrpw[0] - line2.startXyzrpw[0],line2.endXyzrpw[1] - line2.startXyzrpw[1],line2.endXyzrpw[2] - line2.startXyzrpw[2]);
		return fabs(getTwoVector3AngleValue(v1, v2));
}

bool Element::getSwingLineInterpolations()
{

	list<MJCoint> abcdef = SwingWelding::PositionAttitudeLinearInterpolationTri(xyzrpw_2_pose(swingpoints[0]),xyzrpw_2_pose(swingpoints[1]),xyzrpw_2_pose(swingpoints[2]),xyzrpw_2_pose(swingpoints[3]),5);
	 if(checkInterpolations(abcdef))  //校验正确
		{
			interpolationPoints = abcdef;
			return true;
		}
		else
			return false;
}



