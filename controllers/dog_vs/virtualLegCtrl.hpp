/**
 * @brief 使用伪反馈的腿部虚拟力控制
*/
#pragma once

#include "Leg.hpp"

namespace Quadruped
{
	class virtualLegCtrl {
	private:
		Leg* legObj;
		int timeStep = 0;
		// 控制参数，位置环pd控制，力矩环p控制
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
		// 该虚拟控制器针对没有反馈的舵机，因此唯一的状态输入为目标位置
		void setEndPositionTar(Vector3d _p);
		// 腿部控制执行
		void virtualLegCtrlPosition(); // 控制末端位置

	};

	void virtualLegCtrl::setEndPositionTar(Vector3d _p)
	{
		legObj->setTargetLegPositon(_p);
		legObj->legIK_Cal();
		legObj->currentJoint.Angle = legObj->targetJoint.Angle;
	}

	void virtualLegCtrl::virtualLegCtrlPosition()
	{
		// 使用目标值计算期望速度
		legObj->setTargetLegVelocity((legObj->targetLeg.Position - legObj->currentLeg.Position) / ((double)timeStep * 0.001));
		//std::cout << "Pos:" << legObj->targetLeg.Position << std::endl;
		//std::cout << "Vel:" << legObj->targetLeg.Velocity << std::endl;
		// 使用伪反馈计算虚拟力
		for (int i(0); i < 3; i++)
		{
			legObj->targetLeg.Force(i) = kp_p(i) * (legObj->targetLeg.Position(i) - legObj->currentLeg.Position(i)) + kd_p(i) * (0 - legObj->targetLeg.Velocity(i));
		}
		//std::cout << "Force:" << legObj->targetLeg.Force << std::endl;
		// 雅可比矩阵转换为输出力矩
		//std::cout << "TarJoint:" << legObj->targetJoint.Angle << std::endl;
		legObj->jacobi = legObj->legJacobi_Cal(legObj->currentJoint);
		//std::cout << "Jacobi:" << legObj->jacobi << std::endl;
		legObj->targetJoint.Torque = legObj->jacobi.inverse() * legObj->targetLeg.Force;
		//std::cout << "Torque:" << legObj->targetJoint.Torque << "\n\n" << std::endl;
		// 力矩转化成舵机角速度，并更新到期望关节角度
		Vector3d W3d = Vector3d::Zero();
		for (int i(0); i < 3; i++)
		{
			W3d(i) = kp_t(i) * legObj->targetJoint.Torque(i);
			legObj->targetJoint.Angle(i) += W3d(i) * ((double)timeStep * 0.001);
		}
		// 伪反馈构造
		legObj->currentJoint.Angle = legObj->targetJoint.Angle;
		legObj->legFK_Cal();
	}
}
