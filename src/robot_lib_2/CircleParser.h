/*
 * CircleParser.h
 *
 *  Created on: 2017Äê7ÔÂ21ÈÕ
 *      Author: deng
 */

#ifndef ROBOT_LIB_2_CIRCLEPARSER_H_
#define ROBOT_LIB_2_CIRCLEPARSER_H_
#include "scaralib.h"
class CircleParser {
public:
	CircleParser();
	~CircleParser();
	static bool getCirclePoint(const ArrayXd& p1,const ArrayXd& p2,const ArrayXd& p3,ArrayXd& p4,ArrayXd& p5,ArrayXd& p6);
	static bool isThreePointInLine(double x1, double y1, double z1,double x2, double y2, double z2, double x3, double y3, double z3);
	static void getMidPoint(double x0,double y0,double z0,double r,double x1,double y1,double z1,double x2,double y2,double z2,double &x,double &y,double &z);
	static int  isSplitArc(const ArrayXd& p1,const ArrayXd& p2,const ArrayXd& p3,ArrayXd& p4,ArrayXd& p5);
	static bool getCenterCoordAndRadius(double x1, double y1, double z1,double x2, double y2, double z2, double x3, double y3, double z3,double &x0,double &y0,double &z0,double &r);

};

#endif /* ROBOT_LIB_2_CIRCLEPARSER_H_ */
