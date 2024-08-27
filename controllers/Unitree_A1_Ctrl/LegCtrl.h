#pragma once

#include "Leg.h"
#include "../../libraries/myLibs/PIDmethod.h"

namespace Quadruped
{

    class LegCtrl
    {
    private:
        Leg *legObject;
        PIDmethod jPid[3]; // 用于关节位控的控制器
        PIDmethod lPid[3]; // 用于末端力控的控制器
        Vector3f jPidParams[3];// 关节位控pid参数(一组三个参数分别是kp,kd,o_max)
        Vector3f lPidParams[3];// 末端力控pid参数(一组三个参数分别是kp,kd,o_max)
        void loadPidParams(PIDmethod _pidObj[3],Vector3f _pidParam[3]);
    public:
        LegCtrl(Leg *_l, int timeStep);
        // 更新电机观测值
        void updateMotorAng(Vector3f _a);
        void updateMotorVel(Vector3f _w);
        void updateMotorTau(Vector3f _t);
        // 更新末端目标值
        void setEndPositionTar(Vector3f _p);
        void setEndVelocityTar(Vector3f _v);
        void setEndForceTar(Vector3f _f);
        // 对腿部整体进行状态计算
        void legStateCal();
        // 腿部控制执行
        void legCtrlPosition(); // 直接对腿部电机进行位控
        void legCtrlForce();    // 对腿部末端位置进行力控
        void legCtrlMix();      //力位混合控制
    };

    LegCtrl::LegCtrl(Leg *_l, int timeStep) : legObject(_l)
    {
        for (auto p : jPid)
        {
            p.PID_Init(Common, static_cast<double>(0.001 * timeStep));
        }
        for (auto p : lPid)
        {
            p.PID_Init(Common, static_cast<double>(0.001 * timeStep));
        }

        lPidParams[0] << 150,-2.5,100;
        lPidParams[1] << 250,-5,100;
        lPidParams[2] << 80,-3,100;
        loadPidParams(lPid,lPidParams);

        jPidParams[0] << 30,-0.5,30;
        jPidParams[1] << 50,-1,30;
        jPidParams[2] << 16,-0.6,30;
        loadPidParams(jPid,jPidParams);

    }

    void LegCtrl::loadPidParams(PIDmethod _pidObj[3],Vector3f _pidParam[3]){
        for(int i=0;i<3;i++){
            _pidObj[i].Params_Config(_pidParam[i](0),0,_pidParam[i](1),0,_pidParam[i](2),-_pidParam[i](2));
        }
    }

    void LegCtrl::updateMotorAng(Vector3f _a)
    {
        legObject->updateJointAng(_a);
    }

    void LegCtrl::updateMotorVel(Vector3f _w)
    {
        legObject->updateJointVel(_w);
    }

    void LegCtrl::updateMotorTau(Vector3f _t)
    {
        legObject->updateJointTau(_t);
    }

    void LegCtrl::setEndPositionTar(Vector3f _p)
    {
        legObject->setTargetLegPositon(_p);
    }

    void LegCtrl::setEndVelocityTar(Vector3f _v)
    {
        legObject->setTargetLegVelocity(_v);
    }

    void LegCtrl::setEndForceTar(Vector3f _f)
    {
        legObject->setTargetLegForce(_f);
    }

    void LegCtrl::legStateCal()
    {
        legObject->legJacobi_Cal();
        legObject->legFK_Cal();
        legObject->legIK_Cal();
    }

    void LegCtrl::legCtrlPosition()
    {
        for (int i = 0; i < 3; i++)
        {
            jPid[i].target = legObject->targetJoint.Angle(i);
            jPid[i].current = legObject->currentJoint.Angle(i);
            jPid[i].Adjust(0, legObject->currentJoint.Velocity(i));
            legObject->targetJoint.Torque(i) = jPid[i].out;
        }
    }

    void LegCtrl::legCtrlForce()
    {
        Vector3f tmp;
        for (int i = 0; i < 3; i++)
        {
            lPid[i].target = legObject->targetLeg.Position(i);
            lPid[i].current = legObject->currentLeg.Position(i);
            lPid[i].Adjust(0, legObject->currentLeg.Velocity(i));
            tmp(i) = lPid[i].out;
        }
        legObject->setTargetLegForce(tmp);
    }

    void LegCtrl::legCtrlMix()
    {
        Vector3f tmp;
        for (int i = 0; i < 3; i++)
        {
            lPid[i].target = legObject->targetLeg.Position(i);
            lPid[i].current = legObject->currentLeg.Position(i);
            lPid[i].Adjust(0, legObject->currentLeg.Velocity(i));
            tmp(i) = lPid[i].out;

            jPid[i].target = legObject->targetJoint.Angle(i);
            jPid[i].current = legObject->currentJoint.Angle(i);
            jPid[i].Adjust(0, legObject->currentJoint.Velocity(i));
        }
        legObject->setTargetLegForce(tmp);
        legObject->targetJoint.Torque(0) += jPid[0].out;
        legObject->targetJoint.Torque(1) += jPid[1].out;
        legObject->targetJoint.Torque(2) += jPid[2].out;
    }
}