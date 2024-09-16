#ifndef __LEG_H
#define __LEG_H

enum leg_id
{
    LF = 0,
    RF = 1,
    LB = 2,
    RB = 3,
};

// 单腿数据结构
typedef struct _EndS
{
    double Position[3];
    double Velocity[3];
    double Force[3];
} EndS;
// 关节数据结构
typedef struct _JointS
{
    double Angle[3];
    double Velocity[3];
    double Torque[3];
} JointS;
// 单腿结构参数
typedef struct _LegParamS
{
    double L[3];
    double ratio;// 该参数用于分辨极性，为正负1
}LegParamS;
// 完整单腿数据结构
typedef struct _LegS
{
    LegParamS legPara;
    EndS targetEnd;
    EndS currentEnd;
    JointS targetJoint;
    JointS currentJoint;
    double jacobi[3][3];
    double jacobiI[3][3];
}LegS;

void InitLeg(LegS* _leg, double _l1, double _l2, double _l3, double _ratio);
void LegJacobiCal(LegS* _leg, JointS* _joint);
void LegInvJacobiCal(LegS* _leg, JointS* _joint);
void LegFkCal(EndS* _end, JointS* _joint, LegParamS* _para);
void LegIkCal(JointS* _joint, EndS* _end, LegParamS* _para);

#endif
