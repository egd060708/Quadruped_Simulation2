#pragma once

#include <Eigen/Dense>
#include "math.h"

using namespace Eigen;

namespace Quadruped
{
    // 单腿数据结构
    typedef struct _LegS
    {
        Vector3d Position;
        Vector3d Velocity;
        Vector3d Force;
    } LegS;
    // 关节数据结构
    typedef struct _JointS
    {
        Vector3d Angle;
        Vector3d Velocity;
        Vector3d Torque;
    } JointS;

    // 单腿类型
    class Leg
    {
    public:
        LegS targetLeg;
        LegS currentLeg;
        JointS targetJoint;
        JointS currentJoint;
        Matrix3d jacobi;
        double L1 = 0, L2 = 0, L3 = 0;
        double ratio = 0; // 该参数用于分辨极性，为正负1

    public:
        Leg(double _l1, double _l2, double _l3, double ratio) : L1(ratio* _l1), L2(_l2), L3(_l3) {}
        void updateJointAng(Vector3d _jAngle);          // 更新各关节观测角度
        void updateJointVel(Vector3d _jVel);            // 更新关节观测角速度
        void updateJointTau(Vector3d _jTorque);         // 更新关节观测力矩
        Matrix3d legJacobi_Cal(JointS &_joint);         // 根据当前电机角速度计算雅可比矩阵
        void legFK_Cal();                               // 包括由当前角度映射末端位姿，由当前角速度映射末端速度，由当前力矩映射当前末端力
        void setTargetLegPositon(Vector3d _lPosition);  // 更新腿部末端位置目标值
        void setTargetLegVelocity(Vector3d _lVelocity); // 更新腿部末端速度目标值
        void setTargetLegForce(Vector3d _lForce);       // 更新腿部末端力目标值
        void legIK_Cal();                               // 包括由末端目标位姿映射关节目标角度，由末端目标速度映射到关节目标速度，由末端目标力映射到当前关节力矩

        const LegS& getLegCurrent(); // 获取单腿模型参数观测值
    };

    void Leg::updateJointAng(Vector3d _jAngle)
    {
        currentJoint.Angle = _jAngle;
    }

    void Leg::updateJointVel(Vector3d _jVel)
    {
        currentJoint.Velocity = _jVel;
    }

    void Leg::updateJointTau(Vector3d _jTorque)
    {
        currentJoint.Torque = _jTorque;
    }

    Matrix3d Leg::legJacobi_Cal(JointS& _joint)
    {
        // 雅可比矩阵计算
        Matrix3d J = Matrix3d::Zero();
        double s1 = sin(_joint.Angle(0));
        double c1 = cos(_joint.Angle(0));
        double s2 = sin(_joint.Angle(1));
        double c2 = cos(_joint.Angle(1));
        double s23 = sin(_joint.Angle(1) + _joint.Angle(2));
        double c23 = cos(_joint.Angle(1) + _joint.Angle(2));
        J(0, 0) = 0;
        J(0, 1) = -L2 * c2 - L3 * c23;
        J(0, 2) = -L3 * c23;
        J(1, 0) = -L1 * s1 + L2 * c1 * c2 + L3 * c1 * c23;
        J(1, 1) = -L2 * s1 * s2 - L3 * s1 * s23;
        J(1, 2) = -L3 * s1 * s23;
        J(2, 0) = L1 * c1 + L2 * s1 * c2 + L3 * s1 * c23;
        J(2, 1) = L2 * c1 * s2 + L3 * c1 * s23;
        J(2, 2) = L3 * c1 * s23;
        return J;
    }

    void Leg::legFK_Cal()
    {
        // 正运动学更新当前末端位置
        currentLeg.Position(0) = -L2 * sin(currentJoint.Angle(1)) - L3 * sin(currentJoint.Angle(1) + currentJoint.Angle(2));
        currentLeg.Position(1) = L1 * cos(currentJoint.Angle(0)) + L2 * sin(currentJoint.Angle(0)) * cos(currentJoint.Angle(1)) + L3 * sin(currentJoint.Angle(0)) * cos(currentJoint.Angle(1) + currentJoint.Angle(2));
        currentLeg.Position(2) = L1 * sin(currentJoint.Angle(0)) - L2 * cos(currentJoint.Angle(0)) * cos(currentJoint.Angle(1)) - L3 * cos(currentJoint.Angle(0)) * cos(currentJoint.Angle(1) + currentJoint.Angle(2));
        // 雅可比矩阵完成从关节速度到末端速度的映射
        currentLeg.Velocity = jacobi * currentJoint.Velocity;
        // 雅可比矩阵完成从当前关节力矩到当前末端虚拟力的映射
        currentLeg.Force = jacobi * currentJoint.Torque;
    }

    void Leg::setTargetLegPositon(Vector3d _lPosition)
    {
        targetLeg.Position = _lPosition;
    }

    void Leg::setTargetLegVelocity(Vector3d _lVelocity)
    {
        targetLeg.Velocity = _lVelocity;
    }

    void Leg::setTargetLegForce(Vector3d _lForce)
    {
        targetLeg.Force = _lForce;
        targetJoint.Torque = jacobi.inverse() * targetLeg.Force;
    }

    void Leg::legIK_Cal()
    {
        // 解析几何法逆运动学
        double x = targetLeg.Position(0);
        double y = targetLeg.Position(1);
        double z = targetLeg.Position(2);

        double L = sqrt(y * y + z * z - L1 * L1);// 首先是二三级连杆投影到yz平面的映射长度
        double theta1 = atan2((L1 * z + L * y), -L * z + L1 * y);
        L = sqrt(x * x + y * y + z * z - L1 * L1);// 随后是投影到三维空间的等效长度
        double theta3 = -3.1415926 + acos((L2 * L2 + L3 * L3 - L * L) / 2 / L2 / L3);

        double a1 = y * sin(theta1) - z * cos(theta1);
        double m1 = -L3 * sin(theta3);
        double m2 = -L3 * cos(theta3) - L2;
        double theta2 = atan2(a1 * m1 + x * m2, x * m1 - a1 * m2);

        targetJoint.Angle(0) = theta1;
        targetJoint.Angle(1) = theta2;
        targetJoint.Angle(2) = theta3;
    }

    const LegS& Leg::getLegCurrent()
    {
        return currentLeg;
    }
}
