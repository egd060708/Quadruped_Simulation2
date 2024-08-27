#pragma once

#include <Eigen/Dense>
#include "math.h"

using namespace Eigen;

namespace Quadruped
{
    // 单腿数据结构
    typedef struct _LegS
    {
        Vector3f Position;
        Vector3f Velocity;
        Vector3f Force;
    } LegS;
    // 关节数据结构
    typedef struct _JointS
    {
        Vector3f Angle;
        Vector3f Velocity;
        Vector3f Torque;
    } JointS;

    // 单腿类型
    class Leg
    {
    public:
        LegS targetLeg;
        LegS currentLeg;
        JointS targetJoint;
        JointS currentJoint;
        Matrix3f jacobi;
        float L1 = 0, L2 = 0, L3 = 0;
        float ratio = 0; // 该参数用于分辨极性，为正负1

    public:
        Leg(float _l1, float _l2, float _l3, float ratio) : L1(ratio * _l1), L2(_l2), L3(_l3) {}
        void updateJointAng(Vector3f _jAngle);          // 更新各关节观测角度
        void updateJointVel(Vector3f _jVel);            // 更新关节观测角速度
        void updateJointTau(Vector3f _jTorque);         // 更新关节观测李军
        void legJacobi_Cal();                           // 根据当前电机角速度计算雅可比矩阵
        void legFK_Cal();                               // 包括由当前角度映射末端位姿，由当前角速度映射末端速度，由当前力矩映射当前末端力
        void setTargetLegPositon(Vector3f _lPosition);  // 更新腿部末端位置目标值
        void setTargetLegVelocity(Vector3f _lVelocity); // 更新腿部末端速度目标值
        void setTargetLegForce(Vector3f _lForce);       // 更新腿部末端力目标值
        void legIK_Cal();                               // 包括由末端目标位姿映射关节目标角度，由末端目标速度映射到关节目标速度，由末端目标力映射到当前关节力矩

        const LegS &getLegCurrent(); // 获取单腿模型参数观测值
    };

    void Leg::updateJointAng(Vector3f _jAngle)
    {
        currentJoint.Angle = _jAngle;
    }

    void Leg::updateJointVel(Vector3f _jVel)
    {
        currentJoint.Velocity = _jVel;
    }

    void Leg::updateJointTau(Vector3f _jTorque)
    {
        currentJoint.Torque = _jTorque;
    }

    void Leg::legJacobi_Cal()
    {
        // 雅可比矩阵计算
        float s1 = sinf(currentJoint.Angle(0));
        float c1 = cosf(currentJoint.Angle(0));
        float s2 = sinf(currentJoint.Angle(1));
        float c2 = cosf(currentJoint.Angle(1));
        float s23 = sinf(currentJoint.Angle(1) + currentJoint.Angle(2));
        float c23 = cosf(currentJoint.Angle(1) + currentJoint.Angle(2));
        jacobi(0, 0) = 0;
        jacobi(0, 1) = -L2 * c2 - L3 * c23;
        jacobi(0, 2) = -L3 * c23;
        jacobi(1, 0) = -L1 * s1 + L2 * c1 * c2 + L3 * c1 * c23;
        jacobi(1, 1) = -L2 * s1 * s2 - L3 * s1 * s23;
        jacobi(1, 2) = -L3 * s1 * s23;
        jacobi(2, 0) = L1 * c1 + L2 * s1 * c2 + L3 * s1 * c23;
        jacobi(2, 1) = L2 * c1 * s2 + L3 * c1 * s23;
        jacobi(2, 2) = L3 * c1 * s23;
    }

    void Leg::legFK_Cal()
    {
        // 正运动学更新当前末端位置
        currentLeg.Position(0) = -L2 * sinf(currentJoint.Angle(1)) - L3 * sinf(currentJoint.Angle(1) + currentJoint.Angle(2));
        currentLeg.Position(1) = L1 * cosf(currentJoint.Angle(0)) + L2 * sinf(currentJoint.Angle(0)) * cosf(currentJoint.Angle(1)) + L3 * sinf(currentJoint.Angle(0)) * cosf(currentJoint.Angle(1) + currentJoint.Angle(2));
        currentLeg.Position(2) = -L2 * cosf(currentJoint.Angle(0)) * cosf(currentJoint.Angle(1)) - L3 * cosf(currentJoint.Angle(0)) * cosf(currentJoint.Angle(1) + currentJoint.Angle(2));
        // 雅可比矩阵完成从关节速度到末端速度的映射
        currentLeg.Velocity = jacobi * currentJoint.Velocity;
        // 雅可比矩阵完成从当前关节力矩到当前末端虚拟力的映射
        currentLeg.Force = jacobi * currentJoint.Torque;
    }

    void Leg::setTargetLegPositon(Vector3f _lPosition)
    {
        targetLeg.Position = _lPosition;
    }

    void Leg::setTargetLegVelocity(Vector3f _lVelocity)
    {
        targetLeg.Velocity = _lVelocity;
    }

    void Leg::setTargetLegForce(Vector3f _lForce)
    {
        targetLeg.Force = _lForce;
        targetJoint.Torque = jacobi.inverse() * targetLeg.Force;
    }

    void Leg::legIK_Cal()
    {
        // 解析几何法逆运动学
        float x = targetLeg.Position(0);
        float y = targetLeg.Position(1);
        float z = targetLeg.Position(2);

        float L = sqrtf(y * y + z * z - L1 * L1);// 首先是二三级连杆投影到yz平面的映射长度
        float theta1 = atan2f((L1 * z + L * y), -L * z + L1 * y);
        L = sqrtf(x * x + y * y + z * z - L1 * L1);// 随后是投影到三维空间的等效长度
        float theta3 = -3.1415926 + acosf((L2 * L2 + L3 * L3 - L * L) / 2 / L2 / L3);

        float a1 = y * sinf(theta1) - z * cos(theta1);
        float m1 = -L3 * sinf(theta3);
        float m2 = -L3 * cosf(theta3) - L2;
        float theta2 = atan2f(a1 * m1 + x * m2, x * m1 - a1 * m2);

        targetJoint.Angle(0) = theta1;
        targetJoint.Angle(1) = theta2;
        targetJoint.Angle(2) = theta3;
    }

    const LegS &Leg::getLegCurrent()
    {
        return currentLeg;
    }
}
