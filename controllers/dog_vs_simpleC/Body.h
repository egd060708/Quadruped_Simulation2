#ifndef __BODY_H
#define __BODY_H

#include "Leg.h"

typedef struct _worldFrame
{
	EndS leg_s[4];
	double legP_base[4][3];
	double dist[3];
	double Ang_xyz[3];
}worldFrame;

typedef struct _bodyFrame
{
	EndS leg_b[4];
}bodyFrame;

typedef struct _Body
{
	double timeStep;
	LegS* legs[4];
	worldFrame targetWorldState;
	worldFrame currentWorldState;
	bodyFrame targetBodyState;
	bodyFrame currentBodyState;
	double leg2body[3];
	double Rsb_c[3][3];
	double RsbI_c[3][3];
	double Psb_c[3];
	double Rsb_t[3][3];
	double RsbI_t[3][3];
	double Psb_t[3];
}Body;

void InitBody(Body* _body, LegS* _legObj[4], double _timeStep);
void SetBodyParam(Body* _body, double _leg2Body[3]);
void UpdateRsb(double _R[3][3], double _ang[3]);
void UpdateRsbI_Ang(double _RI[3][3], double _ang[3]);
void UpdateRsbI_R(double _RI[3][3], double _R[3][3]);
void UpdatePsb(double _P[3], double _dist[3]);
void Leg2BodyP(EndS* _bodyLeg, EndS* _Leg, double _leg2Body[3]);
void Body2LegP(EndS* _Leg, EndS* _bodyLeg, double _leg2Body[3]);
void Leg2BodyAll(EndS* _bl[4], EndS* _l[4], double _leg2Body[3]);
void Body2LegAll(EndS* _l[4], EndS* _bl[4], double _leg2Body[3]);
void Body2WorldP(worldFrame* _w, bodyFrame* _b, double _R[3][3], double _P[3]);
void World2BodyP(bodyFrame* _b, worldFrame* _w, double _invR[3][3], double _P[3]);

void UpdateFootPoint(worldFrame* _wf, double _point[4][3]);
void UpdateFootPointBase(worldFrame* _wf, double _point[4][3]);
void UpdateFootVel(worldFrame* _wf, double _vel[4][3]);
void UpdateBodyPos(worldFrame* _wf, double _ang[3], double _dist[3]);

#endif
