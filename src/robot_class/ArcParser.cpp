/*
 * ArcParser.cpp
 *
 *  Created on: 2017年6月28日
 *      Author: deng
 */

#include "ArcParser.h"
#include "math.h"
#define PI 3.1415926
double ArcParser::divideLength = 4;
#include <iostream>
using namespace std;
ArcParser::ArcParser() {
	// TODO Auto-generated constructor stub

}

ArcParser::~ArcParser() {
	// TODO Auto-generated destructor stub
}

//是否三点共线
bool ArcParser::isThreePointInLine(double x1, double y1, double z1,double x2, double y2, double z2, double x3, double y3, double z3)
{
	 //法向量的值 n = AB x AC  (叉乘)
	      double i, j, k;
	      i = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
	      j = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
	      k = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
	      //先判断是否共线，根据三角形面积，S = 1/2|AB x AC| ,向量叉乘的模
	      return i == 0.0 && j == 0.0 && k == 0.0;
}

//得到圆的分割点
bool ArcParser::getArcDividePoint(double x1, double y1, double z1, double r1, double p1, double w1,double x2,double y2, double z2, double x3, double y3, double z3, double r3, double p3,double w3, int num,vector<vector<double>>& pointList)
{
	           vector<double> point;
	           if (isThreePointInLine(x1, y1, z1, x2, y2, z2, x3, y3, z3))
	           {
	        	   cout << "三点共线" << endl;
	               return false;
	           }
	    //不共线表示可以构成三角形，用平面基向量 AB AC 来表示 AO， AO = aAB + bAC; AO·AB = 1/2 * AB^2, AO·AC = 1/2 * AC^2;
	           double ABAB, ABAC, ACAC, a, b;
	           ABAB = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1) + (z2 - z1) * (z2 - z1);
	           ABAC = (x2 - x1) * (x3 - x1) + (y2 - y1) * (y3 - y1) + (z2 - z1) * (z3 - z1);
	           ACAC = (x3 - x1) * (x3 - x1) + (y3 - y1) * (y3 - y1) + (z3 - z1) * (z3 - z1);
	           a = ACAC * (ABAB - ABAC) / (ACAC * ABAB - ABAC * ABAC) / 2.0;
	           b = (ABAB * ABAC - ABAB * ACAC) / (ABAC * ABAC - ACAC * ABAB) / 2.0;
	           //圆心坐标和半径
	           double x0, y0, z0, r;
	           x0 = x1 + a * (x2 - x1) + b * (x3 - x1);
	           y0 = y1 + a * (y2 - y1) + b * (y3 - y1);
	           z0 = z1 + a * (z2 - z1) + b * (z3 - z1);
	           r = sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1) + (z0 - z1) * (z0 - z1));
	           //考虑到起点、末点之间的圆心角可能大于180度，把角度分成起点与中点，中点与末点对应的圆心角之和
	           //使用向量积来求，cos在0-pi内是单调的，选用点积
	           double OAOB = (x1 - x0) * (x2 - x0) + (y1 - y0) * (y2 - y0) + (z1 - z0) * (z2 - z0);
	           double OBOC = (x2 - x0) * (x3 - x0) + (y2 - y0) * (y3 - y0) + (z2 - z0) * (z3 - z0);
	           double angle1 = acos(OAOB / (r * r));
	           double angle2 = acos(OBOC / (r * r));
	           //每一等分对应的圆心角
	           double angle = divideLength / r;
	   //        angle = 0.15707963267948966192313216916398;       //测试用
	           //等分数
	           int divideNum = (int) ((angle1 + angle2) / angle);
	           cout << " divideNum: " << divideNum << endl;
	           if(divideNum < 1)
	           {
	               vector<double> point;
	               point.push_back(x1);
	               point.push_back(y1);
	               point.push_back(z1);
	               point.push_back(r1);
	               point.push_back(p1);
	               point.push_back(w1);
	               point.push_back(1 + num);
	               pointList.push_back(point);
	               point.clear();
	               point.push_back(x2);
	               point.push_back(y2);
	               point.push_back(z2);
	               point.push_back((r1 + r3) / 2.0);
	               point.push_back((p1 + p3) / 2.0);
	               point.push_back((w1 + p3) / 2.0);
	               point.push_back(2 + num);
	               pointList.push_back(point);
	               point.clear();
	               point.push_back(x3);
	               point.push_back(y3);
	               point.push_back(z3);
	               point.push_back(r3);
	               point.push_back(p3);
	               point.push_back(w3);
	               point.push_back(3 + num);
	               pointList.push_back(point);
	               return true;
	           }
	           //姿态的等分
	           double rDivide = (r3 - r1) / divideNum;
	           double pDivide = (p3 - p1) / divideNum;
	           double wDivide = (w3 - w1) / divideNum;

	           //同样根据向量积来求等分点的坐标(x, y, z)
	           //先判断A、O、B是否共线，共线就用A、O、C来求解，因为A、O、B和A、O、C不可能同时共线
	           //A、O、B不共线，用平面基向量 OA OB 来表示 OD， OD = aOA + bOB; OA·OD = r*r*cos(angle), OB·OD = r*r*cos(angle1);
	           //A、O、B共线，用平面基向量 OA OC 来表示 OD， OD = aOA + bOC; OA·OD = r*r*cos(angle), OC·OD = r*r*cos(angle1 + angle2);
	           //A、O、B不共线解得 a = sin(angle)/sin(angle1), b = sin(angle1 - angle)/sin(angle1);
	           //A、O、B共线解得 a = sin(angle)/sin(angle1 + angle2), b = sin(angle1 + angle2 - angle)/sin(angle1 + angle2);
	           //判断A、O、B是否共线，可以用之前的判断共线
	           double x, y, z;     //等分点的坐标
	           if (fabs(angle1 - PI) < 0.01 || fabs(angle1 - 0.0) < 0.01)
	           {            //共线,
	               // 不一定用isThreePointInLine来判断，用 angle1 与 pi 的关系也可以判断的
	               for (int i = 0; i < divideNum; i++)
	               {
	                   //等分点的属性，x, y, z和点的序号, 必须每次都初始化对象，不然之前point的也会跟着变化
	                   vector<double> point;
	                   point.clear();
	                   b = sin(angle * (i + 1)) / sin(angle1 + angle2);
	                   a = sin(angle1 + angle2 - angle * (i + 1)) / sin(angle1 + angle2);
	                   point.push_back(x0 + a * (x1 - x0) + b * (x3 - x0));        //x
	                   point.push_back(y0 + a * (y1 - y0) + b * (y3 - y0));        //y
	                   point.push_back(z0 + a * (z1 - z0) + b * (z3 - z0));        //z
	                   point.push_back(r1 + rDivide * (i + 1));                    //r
	                   point.push_back(p1 + pDivide * (i + 1));                    //p
	                   point.push_back(w1 + wDivide * (i + 1));                    //w
	                   point.push_back(i + 1 + num);              //序号，从 1 开始，到时候获取的时候直接强制转换成int就可以了
	                   pointList.push_back(point);
	               }
	           }
	           else {
	               for (int i = 0; i < divideNum; i++)
	               {
	                   vector<double> point;
	                   point.clear();
	                   b = sin(angle * (i + 1)) / sin(angle1);
	                   a = sin(angle1 - angle * (i + 1)) / sin(angle1);
	                   point.push_back(x0 + a * (x1 - x0) + b * (x2 - x0));
	                   point.push_back(y0 + a * (y1 - y0) + b * (y2 - y0));
	                   point.push_back(z0 + a * (z1 - z0) + b * (z2 - z0));
	                   point.push_back( r1 + rDivide * (i + 1));                    //r
	                   point.push_back( p1 + pDivide * (i + 1));                    //p
	                   point.push_back( w1 + wDivide * (i + 1));                    //w
	                   point.push_back(i + 1 + num);
	                   pointList.push_back(point);
	               }
	           }
	           point = pointList.at(divideNum - 1);
	           if (!isEndPoint(point[0], point[1], point[2], x3, y3, z3))
	           {    //判断是否最后一点
	               vector<double> point;
	               point.clear();
	               point.push_back(x3);        //x
	               point.push_back(y3);        //y
	               point.push_back(z3);        //z
	               point.push_back(r3);                    //r
	               point.push_back(p3);                    //p
	               point.push_back(w3);                    //w
	               point.push_back(divideNum + 1 + num);            //序号，从 1 开始
	               pointList.push_back(point);
	           }
	           cout << "打断完成" << endl;
	           return true;



}

//判断最后一点是否是第三点
bool ArcParser::isEndPoint(double x1, double y1, double z1, double x2, double y2, double z2)
{
    return fabs(x1 - x2) < 0.000001 && fabs(y1 - y2) < 0.000001 && fabs(z1 - z2) < 0.000001;
}
//得到圆弧的等分点 通过三个点得到     圆的函数
bool ArcParser::getCircleDividePoint(double x1, double y1, double z1, double r1 , double p1 ,double w1 , double x2,
                                    double y2, double z2, double x3, double y3, double z3, double r3,
                                    double p3,double w3, vector<vector<double>>& midPoint )
{
   vector<vector<double>> arcDividePoint;
   vector<vector<double>> newArcDivide;     //新的圆的等分函数
   newArcDivide.clear();                   //圆的清零
   if(!getArcDividePoint(x1, y1, z1, r1, p1,w1, x2, y2, z2, x3, y3, z3, r3, p3,w3,0,arcDividePoint))   //
   {
	cout << "1圆打断失败" << endl;
    return false;
   }
      int num = arcDividePoint.size();
          //倒数第一个点就是x3, y3, z3,这里是取倒数第三个点，然后以这个点为起点，x3, y3, z3为中点，x1, y1, z1为终点再次等分圆弧
	  vector<double> preLast = arcDividePoint.at(num - 5);
	  //计算新的圆弧等分
	  if(!getArcDividePoint(preLast[0], preLast[1], preLast[2], preLast[3],
			  preLast[4], preLast[5],x3, y3, z3, x1, y1, z1, r1, p1,w1, num - 1,newArcDivide))
	  {
		  cout << "2圆打断失败" << endl;
		  return false;
	  }
	  vector<double> pos1 = newArcDivide[newArcDivide.size() * 0.25 + 1];
	  vector<double> pos2 = newArcDivide[newArcDivide.size() * 0.5  + 1];
	  vector<double> pos3 = newArcDivide[newArcDivide.size() * 0.75 + 1];
	  midPoint.push_back(pos1);
	  midPoint.push_back(pos2);
	  midPoint.push_back(pos3);
      return true;
}
//得到圆弧的等分点 通过三个点得到     圆的函数
bool ArcParser::getCircleDivide4Point(double x1, double y1, double z1, double r1 , double p1 ,double w1 , double x2,
                                    double y2, double z2, double x3, double y3, double z3, double r3,
                                    double p3,double w3, double x4, double y4, double z4, double r4,
                                    double p4,double w4, vector<vector<double>>& midPoint )
{
   vector<vector<double>> arcDividePoint;
   vector<vector<double>> newArcDivide;     //新的圆的等分函数
   vector<vector<double>> newArc1Divide;     //新的圆的等分函数
   arcDividePoint.clear();
   newArcDivide.clear();
   newArc1Divide.clear();//圆的清零
   if(!getArcDividePoint(x1, y1, z1, r1, p1,w1, x2, y2, z2, x3, y3, z3, r3, p3,w3,0,arcDividePoint))   //
   {
	cout << "1圆打断失败" << endl;
    return false;
   }
     int num = arcDividePoint.size();
	  //倒数第一个点就是x3, y3, z3,这里是取倒数第三个点，然后以这个点为起点，x3, y3, z3为中点，x1, y1, z1为终点再次等分圆弧
	  vector<double> preLast = arcDividePoint.at(num - 3);

	  if(!getArcDividePoint(preLast[0], preLast[1], preLast[2], preLast[3],
				  preLast[4], preLast[5],x3, y3, z3, x1, y1, z1, r1, p1,w1, num - 1,newArcDivide))
	  {
		  cout << "后打断失败" << endl;
		  return false;
	  }
	  //计算新的圆弧等分
	  if(!getArcDividePoint(preLast[0], preLast[1], preLast[2], preLast[3],
			  preLast[4], preLast[5],x3, y3, z3, x4, y4, z4, r4, p4,w4, num - 1,newArcDivide))
	  {
		  cout << "2圆打断失败" << endl;
		  return false;
	  }
	  vector<double> pos = newArcDivide[newArcDivide.size() * 0.5  + 1];
	  midPoint.push_back(pos);

	  num = newArcDivide.size();
	  //倒数第一个点就是x3, y3, z3,这里是取倒数第三个点，然后以这个点为起点，x3, y3, z3为中点，x1, y1, z1为终点再次等分圆弧
	   preLast = newArcDivide.at(num - 3);
	  //计算新的圆弧等分
	  if(!getArcDividePoint(preLast[0], preLast[1], preLast[2], preLast[3],
			  preLast[4], preLast[5],x4, y4, z4, x1, y1, z1, r1, p1,w1, num - 1,newArc1Divide))
	  {
		  cout << "3圆打断失败" << endl;
		  return false;
	  }
	  pos.clear();
	  pos = newArc1Divide[newArc1Divide.size() * 0.5  + 1];
	  midPoint.push_back(pos);
	  return true;
}


//得到圆弧的等分点 通过三个点得到
bool ArcParser::getCircleDividePoint1(double x1, double y1, double z1, double r1, double p1,double w1, double x2,
                                    double y2, double z2, double x3, double y3, double z3, double r3,
                                    double p3,double w3,vector<vector<double>>& arcDividePoint )
{

   vector<vector<double>>newArcDivide;     //新的圆的等分函数
   newArcDivide.clear();                   //圆的清零
   if(!getArcDividePoint(x1, y1, z1, r1, p1,w1, x2, y2, z2, x3, y3, z3, r3, p3,w3,0,arcDividePoint))   //
    return false;
   int num = arcDividePoint.size();
          //倒数第一个点就是x3, y3, z3,这里是取倒数第三个点，然后以这个点为起点，x3, y3, z3为中点，x1, y1, z1为终点再次等分圆弧
          vector<double> preLast = arcDividePoint.at(num - 2);
          //计算新的圆弧等分
          if(!getArcDividePoint(preLast[0], preLast[1], preLast[2], preLast[3],
                  preLast[4], preLast[5],x3, y3, z3, x1, y1, z1, r1, p1,w1, num - 1,newArcDivide))
              return false;
          arcDividePoint.pop_back();
//          arcDividePoint.removeAt(num - 1);        //原来的去掉末尾
//          arcDividePoint.removeAt(num - 2);
//          arcDividePoint.push_back(newArcDivide);
          for(int i = 0; i < (int)newArcDivide.size();i++)
        	  arcDividePoint.push_back(newArcDivide[i]);
//          arcDividePoint += newArcDivide;
//        arcDividePoint.addAll(newArcDivide);    //加上新的
          return true;


}

