/*
 * IOparameter.h
 *
 *  Created on: 2016年11月26日
 *      Author: 邓永雷
 */

#ifndef LANSUNV2_0_SRC_IOPARAMETER_H_
#define LANSUNV2_0_SRC_IOPARAMETER_H_
class IOparameter {
public:
	IOparameter();
	virtual ~IOparameter();
	void run();
	int IOdata;
	int lastIOdata;
private:
public:
	void setData(int outputIO); //设置数据

};

#endif /* LANSUNV2_0_SRC_IOPARAMETER_H_ */
