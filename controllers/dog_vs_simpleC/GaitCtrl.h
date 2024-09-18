#ifndef _GAITCTRL_H
#define _GAITCTRL_H

#include "Leg.h"
#include "Body.h"

typedef struct _GaitS {
	double Tgait;// 步态总周期
	double Tstance;// 足端触地周期
	double Tswing;// 足端摆动周期
	double k;// 触底占比系数[0，1]
	double bias[4];// 步态偏移相位(相对于总周期而言)
	double phase[4];// 步态当前相位(相对于子模式而言)

	int contact[4];// 是否触地
	int last_contact[4];// 上一次触地情况

	double cmd[4];// 期望运动指令，二维平移+一维旋转+一维步态高度
	double Pstart[4][3];// 足端起始位置
	double Pend[4][3];// 足端终止位置
	Body* robot;// 机器人对象

	double t;// 当前系统时间
	double t_start;// 步态周期开始时间

	double End_Pos[4][3];// 输出末端位置
	double End_Vel[4][3];// 输出末端速度

	bool is_first;// 是否首次运行步态

	double R[4];// 腿的转弯半径
	double theta[4];// 腿的位置极坐标角度
}GaitS;

void InitGaitCtrl(GaitS* _gait, Body* _robot, double _Tgait, double _k, double _bias[4]);
void SetGaitCmd(GaitS* _gait, double _cmd[4], double _slope[3]);
void PhaseGen(GaitS* _gait, double real_t);
void StanceGait(GaitS* _gait, int idx);
void SwingGait(GaitS* _gait, int idx);
void GaitGen(GaitS* _gait);
void GaitRestart(GaitS* _gait, double real_t);


double StraightXY(double _end, double _start, double _phase);
double StraightVxy(double _end, double _star, double _period);
double CycloidXY(double _end, double _start, double _phase);
double CycloidZ(double _height, double _phase);
double CycloidVxy(double _end, double _start, double _phase, double _period);
double CycloidVz(double _height, double _phase, double _period);


#endif
