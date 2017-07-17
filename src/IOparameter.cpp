/*
 * IOparameter.cpp
 *
 *  Created on: 2016年11月26日
 *      Author: 邓永雷
 */

#include "IOparameter.h"
#include "common.h"
#include "DlyCommon.h"
#include "moto.h"
#include <string>
using namespace std;
IOparameter::IOparameter() {
	lastIOdata = 0;
}

IOparameter::~IOparameter() {

}
//2.4号改变，上面传数据
//ToggleButton响应,2个字节，只能控制16位，不选中置1(关)，选中置0(开)
/**   0000 0000 0000 0000
 *  只保证当前位的准确性，其余位数据不变
 *  按钮序号        开（与运算）      关（或运算）
 *     1            0xFFFE          0x0001
 *     2            0xFFFD          0x0002
 *     3            0xFFFB          0x0004
 *     4            0xFFF7          0x0008
 *     5            0xFFEF          0x0010
 *     6            0xFFDF          0x0020
 *     7            0xFFBF          0x0040
 *     8            0xFF7F          0x0080
 *     9            0xFEFF          0x0100
 *    10            0xFDFF          0x0200
 *    11            0xFBFF          0x0400
 *    12            0xF7FF          0x0800
 *    13            0xEFFF          0x1000
 *    14            0xDFFF          0x2000
 *    15            0xBFFF          0x4000
 *    16            0x7FFF          0x8000
 */

void IOparameter::setData(int outputIO) {
	IOM->DATA = outputIO;
}
void IOparameter::run() {
	if (robotType == CoordRobot) {
		IOdata = (bool) EmergencyIO * 1 + (bool) J1LMT * 2 + (bool) XLMT * 4
				+ (bool) J2LMT * 8 + (bool) YLMT * 16 + (bool) J3LMT * 32
				+ (bool) ZLMT * 64 + (bool) J4LMT * 128 + (bool) J5LMT * 256
				+ (bool) J6LMT * 512 + (bool) J7LMT * 1024
				+ (bool) J8LMT * 2048;
	} else {
		IOdata = (bool) EmergencyIO * 1 + (bool) J1LMT * 2 + (bool) J2LMT * 4
				+ (bool) J3LMT * 8 + (bool) ZLMT * 16 + (bool) J4LMT * 32
				+ (bool) J5LMT * 64 + (bool) J6LMT * 128 + (bool) J7LMT * 256
				+ (bool) J8LMT * 512;

	}
	if (ZLMT)
		Variable::IsStop = true;
	if (IOdata != lastIOdata) {
		lastIOdata = IOdata;
		char str[100];
		sprintf(str, "F1,4,%d", IOdata);
		string str1(str);
		DylCommon::protocol_send1(str1);
		if (ZLMT)
			DylCommon::protocol_send("F2,3");
	}
}

