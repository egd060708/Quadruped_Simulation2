#include <iostream>
#include <math.h>
#include "GaitCtrl.h"
#include "mathTool.h"

#define M_PI 3.14159265358979323846

/* 初始化步态数据 */
void InitGaitCtrl(GaitS* _gait, Body* _robot, double _Tgait, double _k, double _bias[4])
{
	_gait->is_first = true;
	_gait->robot = _robot;
	_gait->Tgait = _Tgait;
	_gait->k = _k;
	_gait->Tstance = _k * _Tgait;
	_gait->Tswing = (1 - _k) * _Tgait;
	memcpy(_gait->bias, _bias, 4 * sizeof(double));
	for (int i = 0; i < 4; i++)
	{
		_gait->contact[i] = 1;
		_gait->last_contact[i] = 0;
	}
}

/* 配置运动指令 */
void SetGaitCmd(GaitS* _gait, double _cmd[4], double _slope[3])
{
	for (int i = 0; i < 4; i++)
	{
		if (i != 3)
		{
			_gait->cmd[i] = slopeConstrain(_cmd[i], _gait->cmd[i], abs(_slope[i]), -abs(_slope[i]));
		}
		else
		{
			_gait->cmd[i] = _cmd[i];
		}
	}
}

/* 步态相位生成 */
void PhaseGen(GaitS* _gait, double real_t)
{
	// 生成总相位
	double all_phase[4] = { 0 };
	for (int i = 0; i < 4; i++)
	{
		all_phase[i] = fmod(real_t - _gait->t_start + _gait->bias[i] * _gait->Tgait, _gait->Tgait) / _gait->Tgait;
		if (all_phase[i] < _gait->k)
		{
			_gait->contact[i] = 1;
			_gait->phase[i] = all_phase[i] / _gait->k;
		}
		else
		{
			_gait->contact[i] = 0;
			_gait->phase[i] = (all_phase[i] - _gait->k) / (1 - _gait->k);
		}
	}
}

/* 触地步态生成 */
void StanceGait(GaitS* _gait, int idx)
{
	_gait->End_Pos[idx][0] = StraightXY(_gait->Pend[idx][0], _gait->Pstart[idx][0], _gait->phase[idx]);
	_gait->End_Pos[idx][1] = StraightXY(_gait->Pend[idx][1], _gait->Pstart[idx][1], _gait->phase[idx]);
	_gait->End_Pos[idx][2] = 0;
	_gait->End_Vel[idx][0] = StraightVxy(_gait->Pend[idx][0], _gait->Pstart[idx][0], _gait->Tstance);
	_gait->End_Vel[idx][1] = StraightVxy(_gait->Pend[idx][1], _gait->Pstart[idx][1], _gait->Tstance);
	_gait->End_Vel[idx][2] = 0;
}

/* 摆动步态生成 */
void SwingGait(GaitS* _gait, int idx)
{
	_gait->End_Pos[idx][0] = CycloidXY(_gait->Pend[idx][0], _gait->Pstart[idx][0], _gait->phase[idx]);
	_gait->End_Pos[idx][1] = CycloidXY(_gait->Pend[idx][1], _gait->Pstart[idx][1], _gait->phase[idx]);
	_gait->End_Pos[idx][2] = CycloidZ(_gait->cmd[3], _gait->phase[idx]);
	_gait->End_Vel[idx][0] = CycloidVxy(_gait->Pend[idx][0], _gait->Pstart[idx][0], _gait->phase[idx], _gait->Tswing);
	_gait->End_Vel[idx][1] = CycloidVxy(_gait->Pend[idx][1], _gait->Pstart[idx][1], _gait->phase[idx], _gait->Tswing);
	_gait->End_Vel[idx][2] = CycloidVz(_gait->cmd[3], _gait->phase[idx], _gait->Tswing);
}

/* 摆线生成 */
double CycloidXY(double _end, double _start, double _phase)
{
	return (_start + (_end - _start) * (2 * M_PI * _phase - sin(2 * M_PI * _phase)) / (2 * M_PI));
}

double CycloidZ(double _height, double _phase)
{
	return (_height * (1 - cos(2 * M_PI * _phase)) / 2);
}

/* 摆线速度 */
double CycloidVxy(double _end, double _start, double _phase, double _period)
{
	return ((_end - _start) * (1 - cos(2 * M_PI * _phase)) / _period);
}

double CycloidVz(double _height, double _phase, double _period)
{
	return M_PI * _height * sin(2 * M_PI * _phase) / _period;
}

/* 直线生成 */
double StraightXY(double _end, double _start, double _phase)
{
	return ((1 - _phase) * _start + _phase * _end);
}

/* 直线速度 */
double StraightVxy(double _end, double _start, double _period)
{
	return (_end - _start) / _period;
}

/* 步态生成 */
void GaitGen(GaitS* _gait)
{
	for (int i = 0; i < 4; i++)
	{
		// 更新每条腿的转向半径
		_gait->R[i] = sqrt(pow(_gait->robot->targetWorldState.legP_base[i][0], 2) + pow(_gait->robot->targetWorldState.legP_base[i][1], 2));
		_gait->theta[i] = atan2(_gait->robot->targetWorldState.legP_base[i][1], _gait->robot->targetWorldState.legP_base[i][0]);
		// 刚启动时和切换状态时需要更新规划目标
		if (_gait->contact[i] != _gait->last_contact[i] || _gait->is_first == true)
		{
			if (_gait->contact[i] == 1)
			{
				// 使用当前位置为起点
				_gait->Pstart[i][0] = _gait->robot->currentWorldState.leg_s[i].Position[0];
				_gait->Pstart[i][1] = _gait->robot->currentWorldState.leg_s[i].Position[1];
				// 平动规划
				_gait->Pend[i][0] = -0.5 * _gait->cmd[0] * _gait->Tstance;
				_gait->Pend[i][1] = -0.5 * _gait->cmd[1] * _gait->Tstance;
				// 转动规划
				_gait->Pend[i][0] += _gait->R[i] * cos(_gait->theta[i] - 0.5 * _gait->cmd[2] * _gait->Tstance);
				_gait->Pend[i][1] += _gait->R[i] * sin(_gait->theta[i] - 0.5 * _gait->cmd[2] * _gait->Tstance);
				// 纵向规划
				_gait->Pend[i][2] = 0;
				_gait->Pstart[i][2] = 0;
			}
			else
			{
				// 使用当前位置为起点
				_gait->Pstart[i][0] = _gait->robot->currentWorldState.leg_s[i].Position[0];
				_gait->Pstart[i][1] = _gait->robot->currentWorldState.leg_s[i].Position[1];
				// 平动规划
				_gait->Pend[i][0] = 0.5 * _gait->cmd[0] * _gait->Tstance;
				_gait->Pend[i][1] = 0.5 * _gait->cmd[1] * _gait->Tstance;
				// 转动规划
				_gait->Pend[i][0] += _gait->R[i] * cos(_gait->theta[i] + 0.5 * _gait->cmd[2] * _gait->Tstance);
				_gait->Pend[i][1] += _gait->R[i] * sin(_gait->theta[i] + 0.5 * _gait->cmd[2] * _gait->Tstance);
				// 纵向规划
				_gait->Pend[i][2] = 0;
				_gait->Pstart[i][2] = 0;
			}
		}
		_gait->last_contact[i] = _gait->contact[i];
		if (_gait->contact[i] == 1)
		{
			StanceGait(_gait, i);
		}
		else
		{
			SwingGait(_gait, i);
		}
	}
	_gait->is_first = false;
}

/* 步态重置 */
void GaitRestart(GaitS* _gait, double real_t)
{
	_gait->is_first = true;
	for (int i = 0; i < 4; i++)
	{
		_gait->phase[i] = 0;
		_gait->contact[i] = 1;
		_gait->last_contact[i] = 0;
	}
	_gait->t_start = real_t;
}

