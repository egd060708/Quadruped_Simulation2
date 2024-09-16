#include "Body.h"
#include <math.h>
#include "mathTool.h"
#include <iostream>

/* ��ʼ��������� */
void InitBody(Body* _body, LegS* _legObj[4], double _timeStep)
{
	memcpy(_body->legs, _legObj, sizeof(_body->legs));
	_body->timeStep = _timeStep;
}

/* ���û�����ز��� */
void SetBodyParam(Body* _body, double _leg2Body[3])
{
	memcpy(_body->leg2body, _leg2Body, 3 * sizeof(double));
}

/* ���»�������ϵ����������ϵ�µ���ת���� */
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

/* ���»�������ϵ����������ϵ�µ�����ת���� */
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

/* ���»�������ϵ����������ϵ�µ�����ת���� */
void UpdateRsbI_R(double _RI[3][3], double _R[3][3])
{
	// ��ת������������ת��
	m3d_transpose2(_RI, _R);
}

/* ��������ϵ�Ȳ�ĩ�˲���ת������������ϵ */
void Leg2BodyP(EndS* _bodyLeg, EndS* _Leg, double _leg2Body[3])
{
	_bodyLeg->Position[0] = _Leg->Position[0] + _leg2Body[0];
}

/* ��������ϵת������������ϵ�Ȳ�ĩ�˲��� */
void Body2LegP(EndS* _Leg, EndS* _bodyLeg, double _leg2Body[3])
{
	_Leg->Position[0] = _bodyLeg->Position[0] - _leg2Body[0];
}

/* ȫ���ת�� */
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

/* ȫ���ת�� */
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
		}
	}
}

/* ��������ϵ�Ȳ�ĩ��ת������������ϵ */
void Body2WorldP(worldFrame* _w, bodyFrame* _b, double _R[3][3])
{
	for (int i = 0; i < 4; i++)
	{
		transform_calc(_w->leg_s[i].Position, _b->leg_b[i].Position, _R, _w->dist);
	}
}

/* ��������ϵ�Ȳ�ĩ��ת����������ϵ */
void World2BodyP(bodyFrame* _b, worldFrame* _w, double _invR[3][3])
{
	for (int i = 0; i < 4; i++)
	{
		invTransform_calc(_b->leg_b[i].Position, _w->leg_s[i].Position, _invR, _w->dist);
	}
}

/* ����Ŀ������㣨����ڻ�������ͶӰ�������λ�ã� */
void UpdateFootPoint(worldFrame* _wf, double _point[4][3])
{
	for (int i = 0; i < 4; i++)
	{
		memcpy(_wf->leg_s[i].Position, &_point[i][0], 3 * sizeof(double));
	}
}

/* ���»�����̬ */
void UpdateBodyPos(worldFrame* _wf, double _ang[3], double _dist[3])
{
	memcpy(_wf->dist, _dist, 3 * sizeof(double));
	memcpy(_wf->Ang_xyz, _ang, 3 * sizeof(double));
}