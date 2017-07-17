/*
 * Coordinates.h
 *
 *  Created on: 2016年11月17日
 *      Author: Administrator
 */

#ifndef SRC_COORDINATES_H_
#define SRC_COORDINATES_H_
//XYZRPW1W2
#include "moto.h"
#include "common.h"
//坐标类
class Coordinates
{
public:
	Coordinates();
	virtual ~Coordinates();//在用基类指针指向继承类的对象时，如果不加虚函数，则删除基类指针则只会析构基类对象，从而造成内存的泄露


private:
    Joint curJ;   //目前的
    Coint curC;
    Joint lastcurJ;   //上一次的
    Coint lastcurC;
    ArrayXd xyzrpw;   //是一个数组类型
    void getPosition();
public:
    void run();
};

#endif /* SRC_COORDINATES_H_ */
