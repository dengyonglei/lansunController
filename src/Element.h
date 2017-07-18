/*
 * Element.h
 *
 *  Created on: 2017年5月3日
 *      Author: deng
 */

#ifndef ROBOT_CLASS_ELEMENT_H_
#define ROBOT_CLASS_ELEMENT_H_
#include "moto.h"
const int MobileTransfinite = -1;
enum ElementType
{
	MoveLine,  //空移线
	FireLine,  //焊接线
	Arc        //圆弧
};
struct MJCPoint
{
	MJCoint mj;
	long int index;
};

struct arcStrickObject     //起弧类
{

	double arcStricDelay;   //起弧延时有时候不需要延时
	int num; //序号数
};
struct arcQuenchObject    //熄弧类
{
	int num;       //序号数
};
class Element
{
public:
	Element();
	virtual ~Element();
	double angle; //角度
	int num;      //序号
	double speed; //速度
	ElementType type;   //线的类型
	ArrayXd startXyzrpw; //起点
	ArrayXd endXyzrpw;   //末点
	ArrayXd midXyzrpw;   //末点
	Coint startC;        //起点的变位机构
	Coint endC;          //终点的变位机构
	Coint midC;          //中点的变位机构
	list<MJCoint> interpolationPoints;  //当前的插补点
	list<MJCPoint> interpolationPointsIndexs;
	arcStrickObject arcStrickObj;   //起弧类
	arcQuenchObject arcQuenchObj;   //熄弧类
	void getMoveLineInterpolations();   //得到空移线的插补点 其实就是末点 为了统一
	bool getLineInterpolations();       //得到直线的插补点
	bool getArcInterpolations();        //得到圆弧的插补点
	bool checkInterpolations(list<MJCoint> abcdef);    //校验插补点是否正确
	bool getInterpolations();             //得到插补点
	static double getTwoLinesAngle(Element &line1,Element &line2);   //得到两条线之间的夹角
	int Index; //线段的索引值  与序号区别开
	bool IsArcStric;
	bool IsQuench;

};

#endif /* ROBOT_CLASS_ELEMENT_H_ */
