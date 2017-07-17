/*
 * Welding.cpp
 *
 *  Created on: 2016年11月21日
 *      Author: Administrator
 */
#include <sys/time.h>
#include "hps_0.h"
#include "DlyCommon.h"
#include "Welding.h"
#define  ARCSTRICK   0xfffffff7       //起弧
#define  ARCQUENCH   0xffffffff       //熄弧
Welding::Welding() :runing(false), rate(1), IsFireMode(false)
{
	init();
	lastSpeed = 0;
}
Welding::~Welding()
{

}
//运行函数开始运动函数
void Welding::move()
{
	if (!runing)  //没有收到焊接运行的指令
		return;
	double minSpeed = 200;          //最小速度
	double currentSpeed = minSpeed; //当前速度
	double speed;                   //目前速度
	bool arcStrickStatic = false;   //目前焊接状态，是否起了弧
	//开始运动
	if (moveFinished)
	{
		moveFinished = false;
		interpolationIndex = 0;
		graphInex = 0;
	}
	if (backFinished)
	{
		backFinished = false;
		interpolationIndex = 0;
		graphInex = 0;
	}
	for (int i = graphInex; i < (int) graph.size(); i++) //遍历整个图形数据
	{
		double acctimes = 200;          //加速次数
		double dectimes = 100;	        //减速次数
		speed = graph[i].speed;   //获取当前速度
//		acctimes = dectimes = 50 * (speed / 1000.0) * rate;   //获取加减速步数
		if (acctimes < 1)
			acctimes = dectimes = 1;
		//起弧条件

		list<MJCPoint> & abcdef = graph[i].interpolationPointsIndexs;    //获得序列点
		long int linesize = abcdef.size();        //每隔0.1循环的次数
		list<MJCPoint>::iterator iter = abcdef.begin();
		if (graph[i].Index - 1 == graphInex && graph[i].type != MoveLine) //遍历到断点去
		{
			for (int m = 0; m < interpolationIndex; m++)
				iter++;
			linesize -= interpolationIndex;
		}
		double anglerate = 1;
		anglerate = graph[i].angle / 180.0;
		if (linesize < (acctimes + dectimes))
		{
			acctimes = dectimes = linesize / 2.0;
		}
		if (acctimes < 1)
			acctimes = 1;
		if (dectimes < 1)
			dectimes = 1;
		double speedrate = 0;    //速度倍率
		//拐角加减速控制
		speedrate = 1 - anglerate;
		//某一条线段起点
		long int runtaskNum = 0;   //在插补里面的任务数
		if(IsFireMode && graph[i].type != MoveLine && !arcStrickStatic)
		{
			if(IsFireLinecPause || IsModeChangeArcStrick)
			{
				arcStrick(lastArcStrickObject.num, lastArcStrickObject.arcStricDelay);                //开始起弧
				arcStrickStatic = true;  //更新焊接状态
				if(IsFireLinecPause)
				IsFireLinecPause = false;
				if(IsModeChangeArcStrick)
			    IsModeChangeArcStrick = false;
			}
		}
		char str[100];
		sprintf(str, "DE,4,%d", graph[i].num);
		cout << "*********" << graph[i].Index << "线运动*********" << endl;
		DylCommon::protocol_send(str);
		for (; iter != abcdef.end(); iter++)
		{
			//表明是第一条
			if (graph[i].type == MoveLine)   //如果是空移线
			{
				iter = abcdef.end();
				iter--;
				cout << graph[i].num << "线段快速移动" << endl;
				moto_runJAbs((iter->mj).j, (iter->mj).c, 5000);
				if (Variable::IsStop) //暂停
				{

					Variable::IsStop = false;
					interpolationIndex = 0;
					graphInex = graph[i].Index - 1;        //线的序号保存下来
					runing = false;
					cout << "graphInex: " << graphInex << endl;
					return;
				}
				cout << graph[i].num << "线段快速移动完成 " << endl;
				break;
			}
			if (Variable::IsStop)   //暂停
			{
				if(arcStrickStatic)
					IsFireLinecPause = true;
				IOM->DATA = ARCQUENCH;       //熄弧
				interpolationIndex = iter->index - 1;  //把当前的插补数保存下来
				graphInex = graph[i].Index - 1;        //线的序号保存下来
				Variable::IsStop = false;
				runing = false;
				cout << "graphInex: " << graphInex << "  interpolationIndex: "
						<< interpolationIndex << endl;
				return;
			}
//            if(currentSpeed != lastSpeed)
//            {
//            	cout << "speed: " << currentSpeed << endl;
//            	lastSpeed = currentSpeed;
//            }
			moto_runInterpolationAbs((iter->mj).j, (iter->mj).c, currentSpeed);
			runtaskNum++;
			//加速过程
			double speed1 = speed * rate; //速度旋转按钮来改变速度的大小得到的一个初始速度
			//当前速度小于初始设定速度时而且未在减速范围内会一直加速，直到加速为止
			if ((currentSpeed < speed1) && (linesize - runtaskNum) > dectimes)
			{//两种方式的加减速都要考虑
				currentSpeed += ((speed1 - minSpeed) / acctimes);
				if (currentSpeed > speed1)
					currentSpeed = speed1;
			}
			//速度特别小，就不做加减速控制啦
			if (speed1 <= minSpeed)
			{   //不做加减速控制
				if (speed1 < 1)
					speed1 = 1;
				currentSpeed = speed1;
			}
			//减速过程(两种情况需要减速控制)
			//速度调节按钮让当前速度大于设定速度
			// 后面的判断应该不需要吧
			if ((currentSpeed > speed1) && (speed1 > minSpeed))
			{
				currentSpeed -= (speed1 / acctimes);
				if (currentSpeed <= speed1)
					currentSpeed = speed1;
			}
			//在最后线段快走完的末端进行减速控制
			if (((linesize - runtaskNum) <= dectimes) && (speed1 > minSpeed))
			{

				currentSpeed -= (speed1 - minSpeed) / dectimes;
				if (currentSpeed <= speedrate * speed1)
					currentSpeed = speedrate * speed1;
				if (currentSpeed < minSpeed)
					currentSpeed = minSpeed;
			}
		}
		//1.处于焊接模式 2.没有起弧的状态 3.是插补线
		if (!arcStrickStatic)
		{
			if(IsFireMode)
			{
				if (graph[i].IsArcStric)
				{
					arcStrickObject obj = graph[i].arcStrickObj;          //获得起弧信息
					lastArcStrickObject = obj;
					arcStrick(obj.num, obj.arcStricDelay);                //开始起弧
					arcStrickStatic = true;                              //更新焊接状态
				}

			}
			else
			{
				if (graph[i].IsArcStric)
				{
					arcStrickObject obj = graph[i].arcStrickObj;   //获得起弧信息
					lastArcStrickObject = obj;
					IsModeChangeArcStrick = true;
				}

		    }

		}

		//熄弧条件
		//1.焊接模式 2.焊接状态中 3.需要熄弧
		if (graph[i].IsQuench)
		{
			if(IsFireMode)
			{
			   if(arcStrickStatic)
			   {
				arcQuenchObject obj = graph[i].arcQuenchObj;   //获取熄弧信息
				arcQuench(obj.num);                                  //开始熄弧
				arcStrickStatic = false;                             //序号+1
			   }
			}
			else
			{
				IsModeChangeArcStrick = false;
			}
		}

	}
	IOM->DATA = ARCQUENCH;       //熄弧;
	if (runing)
	{
		DylCommon::protocol_send("DC,3");
		moveFinished = true;
		cout << "焊接完成" << endl;

	}
	runing = false;
}

//开始起弧，工艺性要做一下
bool Welding::arcStrick(int currentLineNum, double arcStrictime)
{
	cout << "起弧号: " << currentLineNum << "  起弧时间：" << arcStrictime << endl;
	char str[100];
	sprintf(str, "DE,4,%d", currentLineNum);
	DylCommon::protocol_send(str);
	sleep(arcStrictime);
	IOM->DATA = ARCSTRICK;
	return true;
}

//熄弧指令
void Welding::arcQuench(int currentLineNum)
{
	cout << "熄弧号 " << currentLineNum << endl;
	char str[100];
	sprintf(str, "DE,4,%d", currentLineNum);
	DylCommon::protocol_send(str);
	IOM->DATA = ARCQUENCH;       //熄弧
}

void Welding::stopStaticChange()
{
	runing = false;
	IOM->DATA = ARCQUENCH;         //熄弧
	cout << "焊接暂停" << endl;
	Variable::IsStop = false;
}

/**********************************关节式机器人************************************/

//接收到直线点数据,这个时候不要解析 只是保存下传的点位数据
void Welding::receiveLinePoints(const ArrayXd &p1, const Coint& c1,const ArrayXd &p2, const Coint& c2, double speed, int num,bool IsFire)
{
	Element element;   //是一个元素点  接收到一条线就放在圆弧下面
	element.startXyzrpw = p1;  //起点
	element.startC = c1;
	element.endXyzrpw = p2;
	element.endC = c2;
	element.speed = speed;
	element.num = num;
	element.Index = graph.size() + 1;
	if (IsFire)
	{
		//上一条线是切割线 上一条也是切割线则就把上一条线的角度改变 使其减速不必要减太狠
		if(graph.size() > 0 && graph[graph.size() - 1].type == FireLine)
            graph[graph.size() - 1].angle = Element::getTwoLinesAngle(graph[graph.size() - 1],element);
		element.type = FireLine;   //切割线
	}
	else
		element.type = MoveLine;   //空移线
	graph.push_back(element);      //加入进去点进去

}

//圆弧接收数据
void Welding::receiveArcPoints(const ArrayXd &p1, const Coint& c1,const ArrayXd &p, const Coint& c, const ArrayXd &p2, const Coint& c2,double speed, int num)
{
	Element element;                 //元素线段
	element.startXyzrpw = p1;        //起点xyzrpw
	element.startC = c1;             //初始点
	element.endXyzrpw = p2;          //末点
	element.endC = c2;               //末点变位机构
	element.midXyzrpw = p;           //中点机构
	element.midC = c;                //中点
	element.speed = speed;           //速度
	element.num = num;               //序号
	element.type = Arc;              //弧
	element.Index = graph.size() + 1;
	graph.push_back(element);        //加入点进去
}
//圆弧接收数据
void Welding::receiveCircle1Points(const ArrayXd &p1, const Coint& c1,const ArrayXd &p, const Coint& c, const ArrayXd &p2, const Coint& c2,double speed, int num)
{
	vector<vector<double>>  Points;
	double x1 = p1[0];
	double y1 = p1[1];
	double z1 = p1[2];
	double rr1 = p1[3];
	double pp1 = p1[4];
	double ww1 = p1[5];
	double x2 = p[0];
	double y2 = p[1];
	double z2 = p[2];
	double x3 = p2[0];
	double y3 = p2[1];
	double z3 = p2[2];
	double rr3 = p2[3];
	double pp3 = p2[4];
	double ww3 = p2[5];
	ArcParser::getCircleDividePoint1(x1,y1,z1,rr1,pp1,ww1,x2,y2,z2,x3,y3,z3,rr3,pp3,ww3,Points);
//	ArrayXd point1 = p;
//	ArrayXd point2 = p;
	for(int i = 1; i < (int)Points.size(); i++){
		Element element;
		for(int j = 0; j < 6;j++)
		{
			element.startXyzrpw[j] = Points[i - 1][j];
			element.endXyzrpw[j] = Points[i][j];
	   }
		element.speed = speed;           //速度
		element.num = num;               //序号
		element.type = FireLine;              //弧
		element.Index = graph.size() + 1;
		graph.push_back(element);        //加入点进去
     }
}
//圆接收数据
void Welding::receiveCirclePoints(const ArrayXd &p1, const Coint& c1,const ArrayXd &p, const Coint& c, const ArrayXd &p2, const Coint& c2,double speed, int num)
{
	Element element;                 //元素线段
	element.startXyzrpw = p1;        //起点xyzrpw
	element.startC = c1;             //初始点
	element.endXyzrpw = p2;          //末点
	element.endC = c2;               //末点变位机构
	element.midXyzrpw = p;           //中点机构
	element.midC = c;                //中点
	element.speed = speed;           //速度
	element.num = num;               //序号
	element.type = Arc;              //弧
	element.Index = graph.size() + 1;
	graph.push_back(element);        //加入点进去
	vector<vector<double>> midPoint;
	double startPos[6];
	double midPos[6];
	double endPos[6];

	for(int i = 0; i < 6; i++)
	{
		startPos[i] = element.startXyzrpw[i];
		midPos[i] = element.midXyzrpw[i];
		endPos[i] = element.endXyzrpw[i];
	}
	ArcParser::getCircleDividePoint(startPos[0],startPos[1],startPos[2],startPos[3],startPos[4],startPos[5],
			midPos[0],midPos[1],midPos[2],endPos[0],endPos[1],endPos[2],endPos[3],endPos[4],endPos[5],midPoint);
	ArrayXd xyzrpwp1 = p;
	ArrayXd xyzrpwp2 = p;
	ArrayXd xyzrpwp3 = p;
	for(int i = 0; i < 6; i++)
	{
	    xyzrpwp1[i] = midPoint[0][i];
	    xyzrpwp2[i] = midPoint[1][i];
	    xyzrpwp3[i] = midPoint[2][i];
	}
	Element element1; //元素线段
	element1.startXyzrpw = p2;        //起点xyzrpw
	element1.startC = c2;             //初始点
	element1.midXyzrpw = xyzrpwp1;      //中点机构
	element1.midC = c;                //中点
	element1.endXyzrpw = xyzrpwp2;
	element1.endC = c1;               //末点变位机构
	element1.speed = speed;           //速度
	element1.num = num;               //序号
	element1.type = Arc;              //弧
	element1.Index = graph.size() + 1; //标号和序号不一致
	graph.push_back(element1);        //加入点进去
	Element element2; //元素线段
	element2.startXyzrpw = xyzrpwp2;        //起点xyzrpw
	element2.startC = c2;             //初始点
	element2.midXyzrpw = xyzrpwp3;      //中点机构
	element2.midC = c;                //中点
	element2.endXyzrpw = p1;
	element2.endC = c1;               //末点变位机构
	element2.speed = speed;           //速度
	element2.num = num;               //序号
	element2.type = Arc;              //弧
	element2.Index = graph.size() + 1; //标号和序号不一致
	graph.push_back(element2);        //加入点进去
}
//圆接收数据
void Welding::receiveCircle4Points(const ArrayXd &p1, const Coint& c1,const ArrayXd &p, const Coint& c, const ArrayXd &p2, const Coint& c2,const ArrayXd &p3, const Coint& c3,double speed, int num)
{
	Element element;                 //元素线段
	element.startXyzrpw = p1;        //起点xyzrpw
	element.startC = c1;             //初始点
	element.endXyzrpw = p2;          //末点
	element.endC = c2;               //末点变位机构
	element.midXyzrpw = p;           //中点机构
	element.midC = c;                //中点
	element.speed = speed;           //速度
	element.num = num;               //序号
	element.type = Arc;              //弧
	element.Index = graph.size() + 1;
	graph.push_back(element);        //加入点进去
	vector<vector<double>> midPoint;
	double startPos[6];
	double midPos[6];
	double endPos[6];
    double otherPos[6];
	for(int i = 0; i < 6; i++)
	{
		startPos[i] = element.startXyzrpw[i];
		midPos[i] = element.midXyzrpw[i];
		endPos[i] = element.endXyzrpw[i];
		otherPos[i] = p3[i];
	}
	ArcParser::getCircleDivide4Point(startPos[0],startPos[1],startPos[2],startPos[3],startPos[4],startPos[5],
			midPos[0],midPos[1],midPos[2],endPos[0],endPos[1],endPos[2],endPos[3],endPos[4],endPos[5],otherPos[0],otherPos[1],otherPos[2],otherPos[3],otherPos[4],otherPos[5],midPoint);
	ArrayXd xyzrpwp1 = p;
	ArrayXd xyzrpwp2 = p;
	for(int i = 0; i < 6; i++)
	{
	    xyzrpwp1[i] = midPoint[0][i];  //0.5d的点
	    xyzrpwp2[i] = midPoint[1][i];  //0.5d的点
	}
	Element element1; //元素线段
	element1.startXyzrpw = p2;        //起点xyzrpw
	element1.startC = c2;             //初始点
	element1.midXyzrpw = xyzrpwp1;      //中点机构
	element1.midC = c;                //中点
	element1.endXyzrpw = p3;
	element1.endC = c3;               //末点变位机构
	element1.speed = speed;           //速度
	element1.num = num;               //序号
	element1.type = Arc;              //弧
	element1.Index = graph.size() + 1; //标号和序号不一致
	graph.push_back(element1);        //加入点进去

	Element element2; //元素线段
	element2.startXyzrpw = p3;        //起点xyzrpw
	element2.startC = c3;             //初始点
	element2.midXyzrpw = xyzrpwp2;      //中点机构
	element2.midC = c;                //中点
	element2.endXyzrpw = p1;
	element2.endC = c1;               //末点变位机构
	element2.speed = speed;           //速度
	element2.num = num;               //序号
	element2.type = Arc;              //弧
	element2.Index = graph.size() + 1; //标号和序号不一致
	graph.push_back(element2);        //加入点进去
}


//得到每个Element的插补点
bool Welding::getInterpolations()     //得到插补点失败就解析中断
{
	int lastPercent = 0;
	for (int i = 0; i < (int) graph.size(); i++)
	{
		if (!(graph[i].getInterpolations()))
		{
			IsParserSuccess = false;   //解析失败
			return false;
		}
		if(i > 0)
		{
			if(((graph[i].type != MoveLine && graph[i-1].type == Arc)  || (graph[i].type == Arc && graph[i-1].type == FireLine))&& graph[i].interpolationPoints.size() > 5 && graph[i-1].interpolationPoints.size() > 5)
			{
				list<MJCoint>::iterator it1 = graph[i - 1].interpolationPoints.end();
				it1--;  //末点

				ArrayXd xyzrpw12 = pose_2_xyzrpw(it1->m);
				for(int i = 0 ; i < 3; i++)
				    it1--;
			    ArrayXd xyzrpw11 = pose_2_xyzrpw(it1->m);     //起点
				Vector3f v1(xyzrpw12[0] - xyzrpw11[0],xyzrpw12[1] - xyzrpw11[1],xyzrpw12[2] - xyzrpw11[2]);
				list<MJCoint>::iterator it2 = graph[i].interpolationPoints.begin();
				ArrayXd xyzrpw21 = pose_2_xyzrpw(it2->m);  //起点
				for(int i = 0 ; i < 3; i++)
						it2++;                             //末点
				ArrayXd xyzrpw22 = pose_2_xyzrpw(it2->m);
				Vector3f v2(xyzrpw22[0] - xyzrpw21[0],xyzrpw22[1]- xyzrpw21[1],xyzrpw22[2]- xyzrpw21[2]);
				graph[i - 1].angle = getTwoVector3AngleValue(v1,v2);
//				cout << "  角度  " << graph[i -1].angle << endl;
			}
		}
		//progress
		char progress[400];
		int pro = (i + 1.0) / graph.size() * 100;
		if(lastPercent != pro)
		{
		sprintf(progress,"DP,4,%d",pro);
		string str(progress);
		DylCommon::protocol_send(str);
		lastPercent = pro;
		}
	}
	IsParserSuccess = true;          //解析成功
	return true;
}

void Welding::receiveArcStrickData(double delay, int num)  //接收起弧数据
{
	arcStrickObject obj;                       //起弧类
	obj.arcStricDelay = delay;                 //起弧延时
	obj.num = num;                             //起弧号
	graph[graph.size() - 1].IsArcStric = true;
	graph[graph.size() - 1].arcStrickObj = obj;
}
void Welding::receiveArcQuenchData(int num)    //接收熄弧数据
{
	arcQuenchObject obj;                      //熄弧类
	obj.num = num;                            //熄弧序号
	graph[graph.size() - 1].IsQuench = true;
	graph[graph.size() - 1].arcQuenchObj = obj;

}

void Welding::init()                          //初始化函数
{
	graph.clear();
	graphInex = 0;
	interpolationIndex = 0;
	IsParserSuccess = true; //默认为true
	moveFinished = false;
	backFinished = false;
	IsFireLinecPause = false;
	IsModeChangeArcStrick = false;
	lastJIsinit = false;
}

void Welding::back()
{
	if (!backruning)  //没有收到焊接后退的指令
		return;
	double minSpeed = 100;          //最小速度
	double currentSpeed = minSpeed; //当前速度
	double speed;                   //目前速度
	//开始运动
	if (moveFinished)
	{
		moveFinished = false;
		interpolationIndex =  graph[graph.size() - 1].interpolationPointsIndexs.size() - 1;
		graphInex = graph.size() - 1;
	}
	if (backFinished)
	{
		backFinished = false;
		interpolationIndex = 0;
		graphInex = 0;
	}
	for (int i = graphInex; i >= 0; i--) //逆序遍历
	{
		double acctimes = 200;          //加速次数
		double dectimes = 100;	        //减速次数
		speed = graph[i].speed;   //获取当前速度
//	    acctimes = dectimes = 50 * (speed / 1000.0) * rate;   //获取加减速步数
		if (acctimes < 1)
		    acctimes = dectimes = 1;
		list<MJCPoint> & abcdef = graph[i].interpolationPointsIndexs;   //获得序列点
		long int linesize = abcdef.size();                       //每隔0.1循环的次数
		list<MJCPoint>::iterator iter = abcdef.end();             //末点
		iter--; //到最后一个点去
		if (graph[i].Index - 1 == graphInex && graph[i].type != MoveLine) //遍历到断点去
		{
			for (int m = linesize; m >= interpolationIndex; m--)
				iter--;
			linesize = interpolationIndex;
		}
		double anglerate = 1;
		if(i > 0)
		anglerate = graph[i -1].angle / 180.0;
		//当示教线段数很短时，没有匀速过程，直接加速后减速哈哈哈哈
		if (linesize < (acctimes + dectimes))
		{
			acctimes = (acctimes + dectimes) / 2.0;
		}
		if (acctimes < 1)
			acctimes = 1;
		if (dectimes < 1)
			dectimes = 1;
		double speedrate = 0;    //速度倍率
		//拐角加减速控制
		speedrate = 1 - anglerate;
		//某一条线段起点
		char str[100];
		sprintf(str, "DE,4,%d", graph[i].num);
		DylCommon::protocol_send(str);
		cout << "*********" << graph[i].Index << "线逆向运动************" << endl;
		long int runtaskNum = 0;   //在插补里面的任务数
		for (;; iter--)
		{
			//表明是第一条
			if (graph[i].type == MoveLine)   //如果是空移线
			{
				cout << graph[i].num << "线段逆向快速移动" << endl;
				iter = abcdef.begin();
				moto_runJAbs((iter->mj).j, (iter->mj).c, 5000);  //以起点为准
				if (Variable::IsStop) //暂停
				{
					IOM->DATA = ARCQUENCH;       //熄弧
					Variable::IsStop = false;
					interpolationIndex = 0;
					graphInex = graph[i].Index - 1;
					backruning = false;
					return;
				}
				cout << graph[i].num << "线段逆向快速移动完成" << endl;
				break;
			}
			if (Variable::IsStop)   //暂停
			{
				IOM->DATA = ARCQUENCH;       //熄弧
				interpolationIndex = iter->index - 1;  //把当前的插补数保存下来
				graphInex = graph[i].Index - 1;        //线的序号保存下来
				Variable::IsStop = false;
				backruning = false;
				IsFireLinecPause = true;
				return;
			}
			moto_runInterpolationAbs((iter->mj).j, (iter->mj).c, currentSpeed);
			runtaskNum++;
			//加速过程
			double speed1 = speed * rate; //速度旋转按钮来改变速度的大小得到的一个初始速度
			//当前速度小于初始设定速度时而且未在减速范围内会一直加速，直到加速为止
			if ((currentSpeed < speed1) && (linesize - runtaskNum) > dectimes) {//两种方式的加减速都要考虑
				currentSpeed += ((speed1 - minSpeed) / acctimes);
				if (currentSpeed > speed1)
					currentSpeed = speed1;
			}
			//速度特别小，就不做加减速控制啦
			if (speed1 <= minSpeed)
			{   //不做加减速控制
				if (speed1 < 1)
					speed1 = 1;
				currentSpeed = speed1;
			}
			//减速过程(两种情况需要减速控制)
			//速度调节按钮让当前速度大于设定速度
			// 后面的判断应该不需要吧
			if ((currentSpeed > speed1) && (speed1 > minSpeed))
			{
				currentSpeed -= (speed1 / acctimes);
				if (currentSpeed <= speed1)
					currentSpeed = speed1;
			}
			//在最后线段快走完的末端进行减速控制
			if (((linesize - runtaskNum) <= dectimes) && (speed1 > minSpeed))
			{

				currentSpeed -= (speed1 - minSpeed) / dectimes;
				if (currentSpeed <= speedrate * speed1)
					currentSpeed = speedrate * speed1;
				if (currentSpeed < minSpeed)
					currentSpeed = minSpeed;
			}
			if (iter == abcdef.begin())
				break;
		}
		if (i > 0 && graph[i - 1].IsArcStric)
		{
			arcStrickObject obj = graph[i].arcStrickObj;   //获得起弧信息
			lastArcStrickObject = obj;
			IsModeChangeArcStrick = true;
		}
		if (i > 0 && graph[i - 1].IsQuench)
		{

			IsModeChangeArcStrick = true;
		}
}
	if (backruning)
	{
		DylCommon::protocol_send("DC,3");
		backFinished = true;
		cout << "后退完成" << endl;
	}
	backruning = false;
}
void Welding::run()
{
	if (runing)
		move();
	if (backruning)
		back();
}
