#pragma once

#include <Eigen/Dense>
#include "math.h"

using namespace Eigen;

namespace Quadruped
{
    // �������ݽṹ
    typedef struct _LegS
    {
        Vector3d Position;
        Vector3d Velocity;
        Vector3d Force;
    } LegS;
    // �ؽ����ݽṹ
    typedef struct _JointS
    {
        Vector3d Angle;
        Vector3d Velocity;
        Vector3d Torque;
    } JointS;

    // ��������
    class Leg
    {
    public:
        LegS targetLeg;
        LegS currentLeg;
        JointS targetJoint;
        JointS currentJoint;
        Matrix3d jacobi;
        double L1 = 0, L2 = 0, L3 = 0;
        double ratio = 0; // �ò������ڷֱ漫�ԣ�Ϊ����1

    public:
        Leg(double _l1, double _l2, double _l3, double ratio) : L1(ratio* _l1), L2(_l2), L3(_l3) {}
        void updateJointAng(Vector3d _jAngle);          // ���¸��ؽڹ۲�Ƕ�
        void updateJointVel(Vector3d _jVel);            // ���¹ؽڹ۲���ٶ�
        void updateJointTau(Vector3d _jTorque);         // ���¹ؽڹ۲�����
        Matrix3d legJacobi_Cal(JointS &_joint);         // ���ݵ�ǰ������ٶȼ����ſɱȾ���
        void legFK_Cal();                               // �����ɵ�ǰ�Ƕ�ӳ��ĩ��λ�ˣ��ɵ�ǰ���ٶ�ӳ��ĩ���ٶȣ��ɵ�ǰ����ӳ�䵱ǰĩ����
        void setTargetLegPositon(Vector3d _lPosition);  // �����Ȳ�ĩ��λ��Ŀ��ֵ
        void setTargetLegVelocity(Vector3d _lVelocity); // �����Ȳ�ĩ���ٶ�Ŀ��ֵ
        void setTargetLegForce(Vector3d _lForce);       // �����Ȳ�ĩ����Ŀ��ֵ
        void legIK_Cal();                               // ������ĩ��Ŀ��λ��ӳ��ؽ�Ŀ��Ƕȣ���ĩ��Ŀ���ٶ�ӳ�䵽�ؽ�Ŀ���ٶȣ���ĩ��Ŀ����ӳ�䵽��ǰ�ؽ�����

        const LegS& getLegCurrent(); // ��ȡ����ģ�Ͳ����۲�ֵ
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
        // �ſɱȾ������
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
        // ���˶�ѧ���µ�ǰĩ��λ��
        currentLeg.Position(0) = -L2 * sin(currentJoint.Angle(1)) - L3 * sin(currentJoint.Angle(1) + currentJoint.Angle(2));
        currentLeg.Position(1) = L1 * cos(currentJoint.Angle(0)) + L2 * sin(currentJoint.Angle(0)) * cos(currentJoint.Angle(1)) + L3 * sin(currentJoint.Angle(0)) * cos(currentJoint.Angle(1) + currentJoint.Angle(2));
        currentLeg.Position(2) = L1 * sin(currentJoint.Angle(0)) - L2 * cos(currentJoint.Angle(0)) * cos(currentJoint.Angle(1)) - L3 * cos(currentJoint.Angle(0)) * cos(currentJoint.Angle(1) + currentJoint.Angle(2));
        // �ſɱȾ�����ɴӹؽ��ٶȵ�ĩ���ٶȵ�ӳ��
        currentLeg.Velocity = jacobi * currentJoint.Velocity;
        // �ſɱȾ�����ɴӵ�ǰ�ؽ����ص���ǰĩ����������ӳ��
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
        // �������η����˶�ѧ
        double x = targetLeg.Position(0);
        double y = targetLeg.Position(1);
        double z = targetLeg.Position(2);

        double L = sqrt(y * y + z * z - L1 * L1);// �����Ƕ���������ͶӰ��yzƽ���ӳ�䳤��
        double theta1 = atan2((L1 * z + L * y), -L * z + L1 * y);
        L = sqrt(x * x + y * y + z * z - L1 * L1);// �����ͶӰ����ά�ռ�ĵ�Ч����
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
