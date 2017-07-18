/*

 3 * Welding.h
 *
 *  Created on: 2016年11月21日
 *      Author: Administrator
 */

#ifndef SRC_WELDING_H_
#define SRC_WELDING_H_
#include "common.h"
#include "moto.h"
#include "Element.h"
#include "DlyCommon.h"
#include "robot_class/ArcParser.h"
#include <list>
#include <vector>
class Welding
{
public:
	Welding();
	virtual ~Welding();
	bool runing;
	bool backruning;
private:
	bool arcStrick(int currentLineNum, double arcStrictime);
	void arcQuench(int currentLineNum);  //运行过程中熄弧

public:
	void move();
	void run();
	void stopStaticChange();
	double rate;  //当前倍率
	bool IsPreheatingFished;
	bool IsFireMode;    //开火的状态
	bool IsFireFinishedTime;  //是否增加延时时间
	/*************************************关节式机器人更改*******************************************/
	vector<Element> graph;   //图形数据
	int graphInex;                          //线的序号
	long int interpolationIndex;            //插补序号
	bool IsParserSuccess;    //是否解析成功
	bool moveFinished;
	bool backFinished;
	bool IsFireLinecPause;
	bool IsModeChangeArcStrick;
	arcStrickObject lastArcStrickObject;
	//直线接收点
	void receiveLinePoints(const ArrayXd &p1, const Coint& c1,const ArrayXd &p2, const Coint& c2, double speed, int num,bool IsFire);
	//圆弧接收点
	void receiveArcPoints(const ArrayXd &p1, const Coint& c1, const ArrayXd &p,const Coint& c, const ArrayXd &p2, const Coint& c2, double speed,int num);
	//圆接收数据
	void receiveCirclePoints(const ArrayXd &p1, const Coint& c1,const ArrayXd &p, const Coint& c, const ArrayXd &p2, const Coint& c2,double speed, int num);
	void receiveCircle1Points(const ArrayXd &p1, const Coint& c1,const ArrayXd &p, const Coint& c, const ArrayXd &p2, const Coint& c2,double speed, int num);
	void receiveArcStrickData(double delay, int num);   //接收起弧数据
	void receiveArcQuenchData(int num);                //接收熄弧数据
	bool getInterpolations();                          //得到插补点
	void init();
	void back();  //后退
	double lastSpeed;

};

#endif /* SRC_WELDING_H_ */
