/**
 * @brief ʹ��α�������Ȳ�����������
*/
#pragma once

#include "Leg.hpp"

namespace Quadruped
{
	class virtualLegCtrl {
	private:
		Leg* legObj;
		int timeStep = 0;
		// ���Ʋ�����λ�û�pd���ƣ����ػ�p����
		Vector3d kp_p = Vector3d::Zero();
		Vector3d kd_p = Vector3d::Zero();
		Vector3d kp_t = Vector3d::Zero();
		Vector3d lastP = Vector3d::Zero();
	public:
		virtualLegCtrl(Leg* _leg, int _timeStep)
		{
			this->legObj = _leg;
			this->timeStep = _timeStep;
			kp_p << 10, 10, 10;
			kd_p << 0, 0, 0;
			kp_t << 2, 2, 2;
		}
		// ��������������û�з����Ķ�������Ψһ��״̬����ΪĿ��λ��
		void setEndPositionTar(Vector3d _p);
		// �Ȳ�����ִ��
		void virtualLegCtrlPosition(); // ����ĩ��λ��

	};

	void virtualLegCtrl::setEndPositionTar(Vector3d _p)
	{
		legObj->setTargetLegPositon(_p);
		legObj->legIK_Cal();
		legObj->currentJoint.Angle = legObj->targetJoint.Angle;
	}

	void virtualLegCtrl::virtualLegCtrlPosition()
	{
		// ʹ��Ŀ��ֵ���������ٶ�
		legObj->setTargetLegVelocity((legObj->targetLeg.Position - legObj->currentLeg.Position) / ((double)timeStep * 0.001));
		//std::cout << "Pos:" << legObj->targetLeg.Position << std::endl;
		//std::cout << "Vel:" << legObj->targetLeg.Velocity << std::endl;
		// ʹ��α��������������
		for (int i(0); i < 3; i++)
		{
			legObj->targetLeg.Force(i) = kp_p(i) * (legObj->targetLeg.Position(i) - legObj->currentLeg.Position(i)) + kd_p(i) * (0 - legObj->targetLeg.Velocity(i));
		}
		//std::cout << "Force:" << legObj->targetLeg.Force << std::endl;
		// �ſɱȾ���ת��Ϊ�������
		//std::cout << "TarJoint:" << legObj->targetJoint.Angle << std::endl;
		legObj->jacobi = legObj->legJacobi_Cal(legObj->currentJoint);
		//std::cout << "Jacobi:" << legObj->jacobi << std::endl;
		legObj->targetJoint.Torque = legObj->jacobi.inverse() * legObj->targetLeg.Force;
		//std::cout << "Torque:" << legObj->targetJoint.Torque << "\n\n" << std::endl;
		// ����ת���ɶ�����ٶȣ������µ������ؽڽǶ�
		Vector3d W3d = Vector3d::Zero();
		for (int i(0); i < 3; i++)
		{
			W3d(i) = kp_t(i) * legObj->targetJoint.Torque(i);
			legObj->targetJoint.Angle(i) += W3d(i) * ((double)timeStep * 0.001);
		}
		// α��������
		legObj->currentJoint.Angle = legObj->targetJoint.Angle;
		legObj->legFK_Cal();
	}
}
