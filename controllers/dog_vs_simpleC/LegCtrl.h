#ifndef __LEGCTRL_H
#define __LEGCTRL_H

#include "Leg.h"

typedef struct _Leg_Ctrl_Param {
	double kp_p[3];
	double kd_p[3];
	double kp_t[3];
	double ctrl_period;
}Leg_Ctrl_Param;

void InitLegCtrl(Leg_Ctrl_Param* _param, double _kpp[3], double _kdp[3], double _kpt[3], double _t);
void StartVirtualVMC(double _init_p[3], LegS* _leg);
void LegVirtualVMC(LegS* _leg, Leg_Ctrl_Param* _p);

#endif
