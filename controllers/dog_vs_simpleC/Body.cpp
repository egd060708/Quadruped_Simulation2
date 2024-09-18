#include "Body.h"
#include <math.h>
#include "mathTool.h"
#include <iostream>

/* 初始化机身参数 */
void InitBody(Body* _body, LegS* _legObj[4], double _timeStep)
{
	memcpy(_body->legs, _legObj, sizeof(_body->legs));
	_body->timeStep = _timeStep;
}

/* 设置机身相关参数 */
void SetBodyParam(Body* _body, double _leg2Body[3])
{
	memcpy(_body->leg2body, _leg2Body, 3 * sizeof(double));
}

/* 更新机身坐标系在世界坐标系下的旋转矩阵 */
void UpdateRsb(double _R[3][3], double _ang[3])
{
	double Mx[3][3];
	double My[3][3];
	double Mz[3][3];
	rotation_x(Mx, _ang[0]);
	rotation_y(My, _ang[1]);
	rotation_z(Mz, _ang[2]);
	double Mxy[3][3];
	mult_m3d_m3d(Mxy, Mx, My);
	mult_m3d_m3d(_R, Mxy, Mz);
}

/* 更新机身坐标系在世界坐标系下的逆旋转矩阵 */
void UpdateRsbI_Ang(double _RI[3][3], double _ang[3])
{
	double Mz[3][3];
	double My[3][3];
	double Mx[3][3];
	rotation_z(Mz, -_ang[2]);
	rotation_y(My, -_ang[1]);
	rotation_x(Mx, -_ang[0]);
	double Mzy[3][3];
	mult_m3d_m3d(Mzy, Mz, My);
	mult_m3d_m3d(_RI, Mzy, Mx);
}

/* 更新机身坐标系在世界坐标系下的逆旋转矩阵 */
void UpdateRsbI_R(double _RI[3][3], double _R[3][3])
{
	// 旋转矩阵的逆就是其转置
	m3d_transpose2(_RI, _R);
}

/* 更新机身坐标系在世界坐标系下的位移向量 */
void UpdatePsb(double _P[3], double _dist[3])
{
	memcpy(_P, _dist, 3 * sizeof(double));
}

/* 机身坐标系腿部末端参数转换到单腿坐标系 */
void Leg2BodyP(EndS* _bodyLeg, EndS* _Leg, double _leg2Body[3])
{
	_bodyLeg->Position[0] = _Leg->Position[0] + _leg2Body[0];
}

/* 单腿坐标系转换到机身坐标系腿部末端参数 */
void Body2LegP(EndS* _Leg, EndS* _bodyLeg, double _leg2Body[3])
{
	_Leg->Position[0] = _bodyLeg->Position[0] - _leg2Body[0];
}

/* 全身的转换 */
void Leg2BodyAll(EndS* _bl[4], EndS* _l[4], double _leg2Body[3])
{
	double temp[4][3] = { {_leg2Body[0],_leg2Body[1],_leg2Body[2]},
							{_leg2Body[0],-_leg2Body[1],_leg2Body[2]},
							{-_leg2Body[0],_leg2Body[1],_leg2Body[2]},
							{-_leg2Body[0],-_leg2Body[1],_leg2Body[2]} };
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			_bl[i]->Position[j] = _l[i]->Position[j] + temp[i][j];
		}
	}
}

/* 全身的转换 */
void Body2LegAll(EndS* _l[4], EndS* _bl[4], double _leg2Body[3])
{
	double temp[4][3] = { {_leg2Body[0],_leg2Body[1],_leg2Body[2]},
							{_leg2Body[0],-_leg2Body[1],_leg2Body[2]},
							{-_leg2Body[0],_leg2Body[1],_leg2Body[2]},
							{-_leg2Body[0],-_leg2Body[1],_leg2Body[2]} };
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			_l[i]->Position[j] = _bl[i]->Position[j] - temp[i][j];
			_l[i]->Velocity[j] = _bl[i]->Velocity[j];
		}
	}
}

/* 机身坐标系腿部末端转换到世界坐标系 */
void Body2WorldP(worldFrame* _w, bodyFrame* _b, double _R[3][3], double _P[3])
{
	for (int i = 0; i < 4; i++)
	{
		transform_calc(_w->leg_s[i].Position, _b->leg_b[i].Position, _R, _P);
	}
}

/* 世界坐标系腿部末端转换机身坐标系 */
void World2BodyP(bodyFrame* _b, worldFrame* _w, double _invR[3][3], double _P[3])
{
	for (int i = 0; i < 4; i++)
	{
		invTransform_calc(_b->leg_b[i].Position, _w->leg_s[i].Position, _invR, _P);
		mult_m3d_v3d(_b->leg_b[i].Velocity, _invR, _w->leg_s[i].Velocity);
	}
}

/* 设置目标立足点（相对于机体中心投影到地面的位置） */
void UpdateFootPoint(worldFrame* _wf, double _point[4][3])
{
	for (int i = 0; i < 4; i++)
	{
		memcpy(_wf->leg_s[i].Position, &_point[i][0], 3 * sizeof(double));
	}
}

/* 设置基础立足点（相对于机体中心投影到地面的位置） */
void UpdateFootPointBase(worldFrame* _wf, double _point[4][3])
{
	for (int i = 0; i < 4; i++)
	{
		memcpy(_wf->legP_base[i], &_point[i][0], 3 * sizeof(double));
	}
}

/* 设置足端速度 */
void UpdateFootVel(worldFrame* _wf, double _vel[4][3])
{
	for (int i = 0; i < 4; i++)
	{
		memcpy(_wf->leg_s->Velocity, &_vel[i][0], 3 * sizeof(double));
	}
}

/* 更新机体姿态 */
void UpdateBodyPos(worldFrame* _wf, double _ang[3], double _dist[3])
{
	memcpy(_wf->dist, _dist, 3 * sizeof(double));
	memcpy(_wf->Ang_xyz, _ang, 3 * sizeof(double));
}