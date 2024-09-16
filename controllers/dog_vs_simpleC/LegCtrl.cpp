#include "LegCtrl.h"
#include <math.h>
#include "mathTool.h"
#include "main.h"

/* 初始化腿部控制参数 */
void InitLegCtrl(Leg_Ctrl_Param* _param, double _kpp[3], double _kdp[3], double _kpt[3], double _t)
{
	memcpy(_param->kp_p, _kpp, sizeof(_param->kp_p));
	memcpy(_param->kd_p, _kdp, sizeof(_param->kd_p));
	memcpy(_param->kp_t, _kpt, sizeof(_param->kp_t));
	_param->ctrl_period = _t;
}

/* 启动伪反馈VMC */
void StartVirtualVMC(double _init_p[3], LegS* _leg)
{
	// 设置初始目标位置
	memcpy(_leg->targetEnd.Position, _init_p, sizeof(_leg->targetEnd.Position));
	/*printV("initEp", _leg->targetEnd.Position, 3);*/
	// 逆解更新初始目标角度
	//printV("initJa", _leg->targetEnd.Position, 3);
	LegIkCal(&_leg->targetJoint, &_leg->targetEnd, &_leg->legPara);
	// 更新初始当前位置和角度
	memcpy(_leg->currentEnd.Position, _leg->targetEnd.Position, sizeof(_leg->currentEnd.Position));
	memcpy(_leg->currentJoint.Angle, _leg->targetJoint.Angle, sizeof(_leg->currentJoint.Angle));
}

/* 执行伪反馈VMC */
void LegVirtualVMC(LegS* _leg, Leg_Ctrl_Param* _p)
{
	/*printV("tarE", _leg->targetEnd.Position, 3);
	printV("curE", _leg->currentEnd.Position, 3);*/
	// 使用伪反馈计算虚拟力
	for (int i = 0; i < 3; i++)
	{
		_leg->targetEnd.Force[i] = _p->kp_p[i] * (_leg->targetEnd.Position[i] - _leg->currentEnd.Position[i]) + _p->kd_p[i] * _leg->targetEnd.Velocity[i];
	}
	/*printV("tarF", _leg->targetEnd.Force, 3);*/
	// 雅可比矩阵转换为输出力矩
	LegInvJacobiCal(_leg, &_leg->currentJoint);
	// 力矩转化成舵机角速度，并更新到期望关节角度
	double W3d[3] = {0};
	for (int j = 0; j < 3; j++)
	{
		_leg->targetJoint.Torque[j] = dot_v3d_v3d(_leg->jacobiI[j], _leg->targetEnd.Force);
		W3d[j] = _p->kp_t[j] * _leg->targetJoint.Torque[j];
		_leg->targetJoint.Angle[j] += W3d[j] * _p->ctrl_period;
	}
	/*printV("tarT", _leg->targetJoint.Torque, 3);
	printV("tarA", _leg->targetJoint.Angle, 3);*/
	// 构造伪反馈
	memcpy(_leg->currentJoint.Angle, _leg->targetJoint.Angle, sizeof(_leg->currentJoint.Angle));
	LegFkCal(&_leg->currentEnd, &_leg->currentJoint, &_leg->legPara);
	/*printV("curA", _leg->currentJoint.Angle, 3);
	printV("curE", _leg->currentEnd.Position, 3);*/
}