/*
 * CircleParser.cpp
 *
 *  Created on: 2017年7月21日
 *      Author: deng
 */

#include <CircleParser.h>

CircleParser::CircleParser() {
	// TODO Auto-generated constructor stub

}

CircleParser::~CircleParser() {
	// TODO Auto-generated destructor stub
}


bool CircleParser::getCirclePoint(const ArrayXd& p1,const ArrayXd& p2,const ArrayXd& p3,ArrayXd& p4,ArrayXd& p5,ArrayXd& p6)
{
	   double x1 = p1[0];
	   double y1 = p1[1];
	   double z1 = p1[2];
	   double x2 = p2[0];
	   double y2 = p2[1];
	   double z2 = p2[2];
	   double x3 = p3[0];
	   double y3 = p3[1];
	   double z3 = p3[2];
	   double x0, y0, z0,r;
	   if(!getCenterCoordAndRadius(x1,y1,z1,x2,y2,z2,x3,y3,z3,x0,y0,z0,r))
		   return false;
	   double x,y,z;
	   getMidPoint(x0,y0,z0,r,x1,y1,z1,x2,y2,z2,x,y,z);
	   p4[0] = x;
	   p4[1] = y;
	   p4[2] = z;
	   getMidPoint(x0,y0,z0,r,x2,y2,z2,x3,y3,z3,x,y,z);
	   p5[0] = x;
	   p5[1] = y;
	   p5[2] = z;
	   getMidPoint(x0,y0,z0,r,x3,y3,z3,x1,y1,z1,x,y,z);
	   p6[0] = x;
	   p6[1] = y;
	   p6[2] = z;
	   return true;
}
//是否三点共线
bool CircleParser::isThreePointInLine(double x1, double y1, double z1,double x2, double y2, double z2, double x3, double y3, double z3)
{
	 //法向量的值 n = AB x AC  (叉乘)
	  double i, j, k;
	  i = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
	  j = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
	  k = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
	  //先判断是否共线，根据三角形面积，S = 1/2|AB x AC| ,向量叉乘的模
	  return i == 0.0 && j == 0.0 && k == 0.0;
}

void CircleParser::getMidPoint(double x0,double y0,double z0,double r,double x1,double y1,double z1,double x2,double y2,double z2,double &x,double &y,double &z)
{
   	Vector3d v1(x1 - x0,y1 - y0,z1 - z0);
   	Vector3d v2(x2 - x0,y2 - y0,z2 - z0);
   	v1.normalize();
   	v2.normalize();
   	Vector3d v = v1 + v2;
   	v = v.normalized() * r;
   	x = v[0] + x0;
   	y = v[1] + y0;
   	z = v[2] + z0;
}

int CircleParser::isSplitArc(const ArrayXd& p1,const ArrayXd& p2,const ArrayXd& p3,ArrayXd& p4,ArrayXd& p5)
{
	   double x1 = p1[0];
	   double y1 = p1[1];
	   double z1 = p1[2];
	   double x2 = p2[0];
	   double y2 = p2[1];
	   double z2 = p2[2];
	   double x3 = p3[0];
	   double y3 = p3[1];
	   double z3 = p3[2];
	   double x0, y0, z0,r;
	   if(!getCenterCoordAndRadius(x1,y1,z1,x2,y2,z2,x3,y3,z3,x0,y0,z0,r))
		   return -1;
	   double OAOB = (x1 - x0) * (x2 - x0) + (y1 - y0) * (y2 - y0) + (z1 - z0) * (z2 - z0);
	   double OBOC = (x2 - x0) * (x3 - x0) + (y2 - y0) * (y3 - y0) + (z2 - z0) * (z3 - z0);
	   double angle1 = acos(OAOB / (r * r));
	   double angle2 = acos(OBOC / (r * r));
       double angle = (angle1 + angle2) * 180 / pi;
       if(angle < 180)
    	   return 1;
       else
       {
    	   double x,y,z;
		   getMidPoint(x0,y0,z0,r,x1,y1,z1,x2,y2,z2,x,y,z);
		   p4[0] = x;
		   p4[1] = y;
		   p4[2] = z;
		   cout << "中点1：" << x << "  " << y << "  " << z << endl;
		   getMidPoint(x0,y0,z0,r,x2,y2,z2,x3,y3,z3,x,y,z);
		   p5[0] = x;
		   p5[1] = y;
		   p5[2] = z;
		   cout << "中点2：" << x << " " << y << " " << z << endl;
		   return 2;
       }
}

bool CircleParser::getCenterCoordAndRadius(double x1, double y1, double z1,double x2, double y2, double z2, double x3, double y3, double z3,double &x0,double &y0,double &z0,double &r)
{
   if (isThreePointInLine(x1, y1, z1, x2, y2, z2, x3, y3, z3))
   {
	   cout << "三点共线" << endl;
	   return false;
   }
   double ABAB, ABAC, ACAC, a, b;
   ABAB = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1) + (z2 - z1) * (z2 - z1);
   ABAC = (x2 - x1) * (x3 - x1) + (y2 - y1) * (y3 - y1) + (z2 - z1) * (z3 - z1);
   ACAC = (x3 - x1) * (x3 - x1) + (y3 - y1) * (y3 - y1) + (z3 - z1) * (z3 - z1);
   a = ACAC * (ABAB - ABAC) / (ACAC * ABAB - ABAC * ABAC) / 2.0;
   b = (ABAB * ABAC - ABAB * ACAC) / (ABAC * ABAC - ACAC * ABAB) / 2.0;
   //圆心坐标和半径
   x0 = x1 + a * (x2 - x1) + b * (x3 - x1);
   y0 = y1 + a * (y2 - y1) + b * (y3 - y1);
   z0 = z1 + a * (z2 - z1) + b * (z3 - z1);
   r = sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1) + (z0 - z1) * (z0 - z1));
   return true;
}

double CircleParser::getArcLength(double x1, double y1, double z1,double x2, double y2, double z2, double x3, double y3, double z3)
{
	double x0,y0,z0,r;
	getCenterCoordAndRadius(x1,y1,z1,x2,y2,z2,x3,y3,z3,x0,y0,z0,r);
	 double OAOB = (x1 - x0) * (x2 - x0) + (y1 - y0) * (y2 - y0) + (z1 - z0) * (z2 - z0);
	 double OBOC = (x2 - x0) * (x3 - x0) + (y2 - y0) * (y3 - y0) + (z2 - z0) * (z3 - z0);
	 double angle1 = acos(OAOB / (r * r));
	 double angle2 = acos(OBOC / (r * r));
	 double angle = (angle1 + angle2);
	 cout << "圆心角：" << angle * 180 / pi << endl;
	 cout << "半径：" << r << endl;
	 return angle * r;
}

