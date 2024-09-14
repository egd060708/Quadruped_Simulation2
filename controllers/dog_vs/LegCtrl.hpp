#pragma once

#include "Leg.hpp"
#include "../../libraries/myLibs/PIDmethod.h"

namespace Quadruped
{

    class LegCtrl
    {
    private:
        Leg* legObject;
        PIDmethod jPid[3];          // 用于关节位控的控制器
        PIDmethod jPid_pv[6];       // 用于末端位置和速度的关节位控制器
        PIDmethod lPid[3];          // 用于末端力控的控制器
        PIDmethod lPid_pv[6];       // 用于末端位置和速度的力控制器
        Vector3d jPidParams[3];     // 关节位控pid参数(一组三个参数分别是kp,kd,o_max)
        Vector2d jPid_pvParams[6];  // 关节位置速度位控参数(p_kp,v_kp,o_max)
        Vector3d lPidParams[3];     // 末端力控pid参数(一组三个参数分别是kp,kd,o_max)
        Vector2d lPid_pvParams[6];  // 关节位置速度力控参数(p_kp,v_kp,o_max)
        void loadPidParams(PIDmethod _pidObj[3], Vector3d _pidParam[3]);
        void loadPid_pvParams(PIDmethod _pidObj[6], Vector2d _pidParam[6]);
    public:
        LegCtrl(Leg* _l, int timeStep);
        // 更新电机观测值
        void updateMotorAng(Vector3d _a);
        void updateMotorVel(Vector3d _w);
        void updateMotorTau(Vector3d _t);
        // 更新末端目标值
        void setEndPositionTar(Vector3d _p);
        void setEndVelocityTar(Vector3d _v);
        void setEndForceTar(Vector3d _f);
        // 对腿部整体进行状态计算
        void legStateCal();
        // 腿部控制执行
        void legCtrlPosition();  // 直接对腿部电机进行位控
        void legCtrlForce();     // 对腿部末端位置进行力控
        void legCtrlMix();       // 力位混合控制
        void legPvCtrlPosition();//末端位置速度位控
        void legPvCtrlForce();   // 末端位置速度力控

        Vector3d legCtrlPositionR();  // 直接对腿部电机进行位控(返回输出)
        Vector3d legCtrlForceR();     // 对腿部末端位置进行力控
        Vector3d legPvCtrlPositionR();//末端位置速度位控
        Vector3d legPvCtrlForceR();   // 末端位置速度力控
    };

    LegCtrl::LegCtrl(Leg* _l, int timeStep) : legObject(_l)
    {
        for (auto p : jPid)
        {
            p.PID_Init(Common, static_cast<double>(0.001 * timeStep));
        }
        for (auto p : lPid)
        {
            p.PID_Init(Common, static_cast<double>(0.001 * timeStep));
        }

        lPidParams[0] << 20, -0.1, 200;
        lPidParams[1] << 20, -0.1, 200;
        lPidParams[2] << 50, -0.3, 200;
        loadPidParams(lPid, lPidParams);

        jPidParams[0] << 30, -0.5, 30;
        jPidParams[1] << 50, -1, 30;
        jPidParams[2] << 16, -0.6, 30;
        loadPidParams(jPid, jPidParams);

        lPid_pvParams[0] << 40, 100;
        lPid_pvParams[1] << 1, 100;
        lPid_pvParams[2] << 60, 100;
        lPid_pvParams[3] << 1.2, 100;
        lPid_pvParams[4] << 80, 100;
        lPid_pvParams[5] << 2, 100;
        loadPid_pvParams(lPid_pv, lPid_pvParams);

    }

    void LegCtrl::loadPidParams(PIDmethod _pidObj[3], Vector3d _pidParam[3]) {
        for (int i = 0; i < 3; i++) {
            _pidObj[i].Params_Config(_pidParam[i](0), 0, _pidParam[i](1), 0, _pidParam[i](2), -_pidParam[i](2));
        }
    }

    void LegCtrl::loadPid_pvParams(PIDmethod _pidObj[6], Vector2d _pidParam[6]) {
        for (int i = 0; i < 6; i++) {
            _pidObj[i].Params_Config(_pidParam[i](0), 0, _pidParam[i](1), -_pidParam[i](1));
        }
    }

    void LegCtrl::updateMotorAng(Vector3d _a)
    {
        legObject->updateJointAng(_a);
    }

    void LegCtrl::updateMotorVel(Vector3d _w)
    {
        legObject->updateJointVel(_w);
    }

    void LegCtrl::updateMotorTau(Vector3d _t)
    {
        legObject->updateJointTau(_t);
    }

    void LegCtrl::setEndPositionTar(Vector3d _p)
    {
        legObject->setTargetLegPositon(_p);
    }

    void LegCtrl::setEndVelocityTar(Vector3d _v)
    {
        legObject->setTargetLegVelocity(_v);
    }

    void LegCtrl::setEndForceTar(Vector3d _f)
    {
        legObject->setTargetLegForce(_f);
    }

    void LegCtrl::legStateCal()
    {
        legObject->jacobi = legObject->legJacobi_Cal(legObject->currentJoint);
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

    Vector3d LegCtrl::legCtrlPositionR()
    {
        Vector3d out;
        for (int i = 0; i < 3; i++)
        {
            jPid[i].target = legObject->targetJoint.Angle(i);
            jPid[i].current = legObject->currentJoint.Angle(i);
            jPid[i].Adjust(0, legObject->currentJoint.Velocity(i));
            out(i) = jPid[i].out;
        }
        return out;
    }

    void LegCtrl::legPvCtrlPosition()
    {
        for (int i = 0; i < 3; i++)
        {
            jPid_pv[2 * i].target = legObject->targetJoint.Angle(i);
            jPid_pv[2 * i].current = legObject->currentJoint.Angle(i);
            jPid_pv[2 * i + 1].target = legObject->targetJoint.Velocity(i);
            jPid_pv[2 * i + 1].current = legObject->currentJoint.Velocity(i);
            jPid_pv[2 * i].Adjust(0);
            jPid_pv[2 * i + 1].Adjust(0);
            legObject->targetJoint.Torque(i) = jPid_pv[2 * i].out + jPid_pv[2 * i + 1].out;
        }
    }

    Vector3d LegCtrl::legPvCtrlPositionR()
    {
        Vector3d out;
        for (int i = 0; i < 3; i++)
        {
            jPid_pv[2 * i].target = legObject->targetJoint.Angle(i);
            jPid_pv[2 * i].current = legObject->currentJoint.Angle(i);
            jPid_pv[2 * i + 1].target = legObject->targetJoint.Velocity(i);
            jPid_pv[2 * i + 1].current = legObject->currentJoint.Velocity(i);
            jPid_pv[2 * i].Adjust(0);
            jPid_pv[2 * i + 1].Adjust(0);
            out(i) = jPid_pv[2 * i].out + jPid_pv[2 * i + 1].out;
        }
        return out;
    }

    void LegCtrl::legCtrlForce()
    {
        Vector3d tmp;
        for (int i = 0; i < 3; i++)
        {
            lPid[i].target = legObject->targetLeg.Position(i);
            lPid[i].current = legObject->currentLeg.Position(i);
            lPid[i].Adjust(0, legObject->currentLeg.Velocity(i));
            tmp(i) = lPid[i].out;
        }
        legObject->setTargetLegForce(tmp);
    }

    Vector3d LegCtrl::legCtrlForceR()
    {
        Vector3d out;
        for (int i = 0; i < 3; i++)
        {
            lPid[i].target = legObject->targetLeg.Position(i);
            lPid[i].current = legObject->currentLeg.Position(i);
            lPid[i].Adjust(0, legObject->currentLeg.Velocity(i));
            out(i) = lPid[i].out;
        }
        return out;
    }

    void LegCtrl::legPvCtrlForce()
    {
        Vector3d tmp;
        for (int i = 0; i < 3; i++)
        {
            lPid_pv[2*i].target = legObject->targetLeg.Position(i);
            lPid_pv[2*i].current = legObject->currentLeg.Position(i);
            lPid_pv[2 * i + 1].target = legObject->targetLeg.Velocity(i);
            lPid_pv[2 * i + 1].current = legObject->currentLeg.Velocity(i);
            lPid_pv[2 * i].Adjust(0);
            lPid_pv[2 * i + 1].Adjust(0);
            tmp(i) = lPid_pv[2 * i].out + lPid_pv[2 * i + 1].out;
        }
        legObject->setTargetLegForce(tmp);
    }

    Vector3d LegCtrl::legPvCtrlForceR()
    {
        Vector3d out;
        for (int i = 0; i < 3; i++)
        {
            lPid_pv[2 * i].target = legObject->targetLeg.Position(i);
            lPid_pv[2 * i].current = legObject->currentLeg.Position(i);
            lPid_pv[2 * i + 1].target = legObject->targetLeg.Velocity(i);
            lPid_pv[2 * i + 1].current = legObject->currentLeg.Velocity(i);
            lPid_pv[2 * i].Adjust(0);
            lPid_pv[2 * i + 1].Adjust(0);
            out(i) = lPid_pv[2 * i].out + lPid_pv[2 * i + 1].out;
        }
        return out;
    }

    void LegCtrl::legCtrlMix()
    {
        Vector3d tmp;
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