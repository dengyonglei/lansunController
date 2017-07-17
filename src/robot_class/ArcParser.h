/*
 * ArcParser.h
 *
 *  Created on: 2017年6月28日
 *      Author: deng
 */

#ifndef ROBOT_CLASS_ARCPARSER_H_
#define ROBOT_CLASS_ARCPARSER_H_
#include <vector>
#include <list>
using namespace std;
class ArcParser {
public:
	ArcParser();
	virtual ~ArcParser();
	//判断是否三点共线
	static bool isEndPoint(double x1, double y1, double z1, double x2, double y2, double z2);
	static double divideLength;
	static bool isThreePointInLine(double x1, double y1, double z1,double x2, double y2, double z2, double x3, double y3, double z3);
	static bool getArcDividePoint(double x1, double y1, double z1, double r1, double p1, double w1,double x2,double y2, double z2, double x3, double y3, double z3, double r3, double p3,double w3, int num,vector<vector<double>>& pointList);
	static bool getCircleDividePoint(double x1, double y1, double z1, double r1, double p1,double w1, double x2,
	                                    double y2, double z2, double x3, double y3, double z3, double r3,
	                                    double p3,double w3,vector<vector<double>>& midPoint);
	static bool getCircleDivide4Point(double x1, double y1, double z1, double r1, double p1,double w1, double x2,
		                                    double y2, double z2, double x3, double y3, double z3, double r3,
		                                    double p3,double w3,double x4, double y4, double z4, double r4,
		                                    double p4,double w4,vector<vector<double>>& midPoint);
	static bool getCircleDividePoint1(double x1, double y1, double z1, double r1, double p1,double w1, double x2,double y2, double z2, double x3, double y3, double z3, double r3,double p3,double w3,vector<vector<double>>& arcDividePoint);
};

#endif /* ROBOT_CLASS_ARCPARSER_H_ */
