#pragma once
#include <math.h>
#include <iostream>
#include "BodyCtrl.hpp"
#include "LegCtrl.hpp"
using namespace std;

#define M_PI 3.14159265358979323846

namespace Quadruped
{
	enum class WaveStatus {
		STANCE_ALL = 0,
		SWING_ALL = 1,
		WAVE_ALL = 2
	};

	class GaitCtrl
	{
	public:
		GaitCtrl(BodyCtrl* _bc, LegCtrl* _lc[4], int timeStep, Eigen::Vector4d* _gaitPhase, Eigen::Vector4i* _gaitContact);
		Eigen::Vector3d calFootPos(int legID, Eigen::Vector2d vxyTargetGlobal, double dYawTarget, double phase);
		void initExpectK(Eigen::Vector3d _k);// ��ʼ�������˶����Ʋ���
		void initSwingParams(double _period, double _stancePhaseRatio, Eigen::Vector4d _bias, double _t);// ��ʼ���ڶ���ز���
		void calcWave(Eigen::Vector4d& phase, Eigen::Vector4i& contact, WaveStatus status, double _t);
		void calcContactPhase(WaveStatus status, double _t);
		
		void setGait(Eigen::Vector2d vxyTargetGlobal, double dYawTarget, double gaitHeight);
		void run(Eigen::Matrix<double, 3, 4>& _feetPos, Eigen::Matrix<double, 3, 4>& _feetVel);
		Eigen::Vector3d getFootPos(int i);
		Eigen::Vector3d getFootVel(int i);
		void restart();
	public:
		double dt;
		// ���ϵĿ�����,����������Լ��Ȳ�������
		BodyCtrl* bodyController;
		LegCtrl* legController[4];
		// ĩ����ŵ���㲿��
		Eigen::Vector3d nextStep = Eigen::Vector3d::Zero();
		Eigen::Vector3d footPos = Eigen::Vector3d::Zero();
		Eigen::Vector3d bodyVelGlobal = Eigen::Vector3d::Zero();// ����������ϵ�µĻ������ٶ�
		Eigen::Vector3d bodyAccGlobal = Eigen::Vector3d::Zero();// ����������ϵ�µĻ����߼��ٶ�
		Eigen::Vector3d bodyWGlobal = Eigen::Vector3d::Zero();// ����������ϵ�µĻ�����ٶ�
		Eigen::Vector4d feetRadius = Eigen::Vector4d::Zero();// ����˶��뾶
		Eigen::Vector4d feetInitAngle = Eigen::Vector4d::Zero();;// ��˳�ʼ�˶��Ƕ�
		double yaw, dYaw, nextYaw;
		double Tstance, Tswing;
		double kx, ky, kyaw;
		// �Ӵ�״̬��ڶ�״̬���
		double period;// ��̬����p
		double stRatio;// ����ϵ��r(��һ��)
		Eigen::Vector4d bias = Eigen::Vector4d::Zero();// ��̬ƫ��ϵ��������ƫ��ʱ���벽̬���ڵı�ֵ(��һ��)
		Eigen::Vector4d normalT = Eigen::Vector4d::Zero();// ��һ����ʱ��
		Eigen::Vector4d phase = Eigen::Vector4d::Zero();
		Eigen::Vector4d phasePast = Eigen::Vector4d::Zero();
		Eigen::Vector4i contact = Eigen::Vector4i::Zero();
		Eigen::Vector4i contactPast = Eigen::Vector4i::Zero();
		Eigen::Vector4i switchStatus = Eigen::Vector4i::Zero();// �Ƿ��ܹ��л�״̬��1Ϊ���ԣ�0����
		WaveStatus statusPast;
		double passT;
		double startT;
		// �ڶ���������
		double gaitHeight;
		Eigen::Vector2d VxyTarget;
		double dYawTarget;
		Eigen::Vector4d* gaitPhase;
		Eigen::Vector4d gaitPhasePast;
		Eigen::Vector4i* gaitContact;
		Eigen::Matrix<double, 3, 4> startP, endP, idealP, pastP;
		bool is_firstRun;
		// �Ȳ��������ɺ���
		double cycloidXYPosition(double startXY, double endXY, double phase);
		double cycloidXYVelocity(double startXY, double endXY, double phase);
		double cycloidZPosition(double startZ, double height, double phase);
		double cycloidZVelocity(double height, double phase);
	};

	GaitCtrl::GaitCtrl(BodyCtrl* _bc, LegCtrl* _lc[4], int timeStep, Eigen::Vector4d* _gaitPhase, Eigen::Vector4i* _gaitContact)
	{
		this->gaitPhase = _gaitPhase;
		this->gaitContact = _gaitContact;
		this->bodyController = _bc;
		for (int i = 0; i < 4; i++)
		{
			this->legController[i] = _lc[i];
		}
		Eigen::Vector2d initLeg[4];
		initLeg[RB](0) = bodyController->bodyObject->initRbLegXYPosition(0);
		initLeg[RB](1) = bodyController->bodyObject->initRbLegXYPosition(1);
		initLeg[LB](0) = bodyController->bodyObject->initRbLegXYPosition(0);
		initLeg[LB](1) = - bodyController->bodyObject->initRbLegXYPosition(1);
		initLeg[RF](0) = - bodyController->bodyObject->initRbLegXYPosition(0);
		initLeg[RF](1) = bodyController->bodyObject->initRbLegXYPosition(1);
		initLeg[LF](0) = - bodyController->bodyObject->initRbLegXYPosition(0);
		initLeg[LF](1) = - bodyController->bodyObject->initRbLegXYPosition(1);
		for (int i = 0; i < 4; i++)
		{
			feetRadius(i) = sqrt(pow(initLeg[i](0), 2) + pow(initLeg[i](1), 2));
			feetInitAngle(i) = atan2(initLeg[i](1), initLeg[i](0));
		}
		dt = 0.001 * timeStep;
		// TODO: Tstance��Tswing����
	}

	void GaitCtrl::initExpectK(Eigen::Vector3d _k)
	{
		kx = _k(0);
		ky = _k(1);
		kyaw = _k(2);
	}

	void GaitCtrl::initSwingParams(double _period, double _stancePhaseRatio, Eigen::Vector4d _bias, double _t)
	{
		period = _period;
		stRatio = _stancePhaseRatio;
		bias = _bias;
		startT = _t;
		contactPast.setZero();
		phasePast.setConstant(0.5);
		statusPast = WaveStatus::WAVE_ALL;
		Tstance = period * stRatio;
		Tswing = period * (1 - stRatio);
	}

	Eigen::Vector3d GaitCtrl::calFootPos(int legID, Eigen::Vector2d vxyTargetGlobal, double dYawTarget, double phase)
	{
		// ����xyƽ�����ŵ�滮
		bodyVelGlobal = bodyController->currentBalanceState.p_dot;
		bodyWGlobal = bodyController->currentBalanceState.r_dot;

		nextStep(0) = bodyVelGlobal(0) * (1 - phase) * Tswing + bodyVelGlobal(0) * Tstance / 2 + kx * (bodyVelGlobal(0) - vxyTargetGlobal(0));
		nextStep(1) = bodyVelGlobal(1) * (1 - phase) * Tswing + bodyVelGlobal(1) * Tstance / 2 + ky * (bodyVelGlobal(1) - vxyTargetGlobal(1));
		nextStep(2) = 0;

		// ������ת״̬����ŵ���ӹ滮
		yaw = bodyController->currentBalanceState.r(2);
		dYaw = bodyController->currentBalanceState.r_dot(2);
		nextYaw = dYaw * (1 - phase) * Tswing + dYaw * Tstance / 2 + kyaw * (dYawTarget - dYaw);

		nextStep(0) += feetRadius(legID) * cos(yaw + feetInitAngle(legID) + nextYaw);
		nextStep(1) += feetRadius(legID) * sin(yaw + feetInitAngle(legID) + nextYaw);

		footPos = bodyController->currentBalanceState.p + nextStep;
		footPos(2) = 0.0;

		return footPos;
	}

	void GaitCtrl::calcWave(Eigen::Vector4d& phase, Eigen::Vector4i& contact, WaveStatus status, double _t)
	{
		if (status == WaveStatus::WAVE_ALL)
		{
			passT = _t - startT;
			for (int i(0); i < 4; ++i)
			{
				normalT(i) = fmod(passT + period - period * bias(i), period) / period;// ȡ���������һ��
				// ���ݹ�һ����T�ж�Ӧ���ǽӴ����滹�ǰڶ�
				if (normalT(i) < stRatio)
				{
					contact(i) = 1;
					phase(i) = normalT(i) / stRatio;
				}
				else
				{
					contact(i) = 0;
					phase(i) = (normalT(i) - stRatio) / (1 - stRatio);
				}
			}
		}
		else if (status == WaveStatus::SWING_ALL)
		{
			contact.setZero();
			phase.setConstant(0.5);
		}
		else if (status == WaveStatus::STANCE_ALL)
		{
			contact.setOnes();
			phase.setConstant(0.5);
		}
	}

	void GaitCtrl::calcContactPhase(WaveStatus status, double _t)
	{

		calcWave(phase, contact, status, _t);

		if (status != statusPast)
		{
			if (switchStatus.sum() == 0)
			{
				switchStatus.setOnes();
			}
			calcWave(phasePast, contactPast, statusPast, _t);
			//// ����������ֱ��Ǵ���ȫվ����ȫ�ڶ����Լ�ȫ�ڶ�����ȫվ��
			//if ((status == WaveStatus::STANCE_ALL) && (statusPast == WaveStatus::SWING_ALL))
			//{
			//	contactPast.setOnes();
			//}
			//else if ((status == WaveStatus::SWING_ALL) && (statusPast == WaveStatus::STANCE_ALL))
			//{
			//	contactPast.setZero();
			//}
		}

		// ����л�״̬Ϊ����
		if (switchStatus.sum() != 0)
		{
			for (int i = 0; i < 4; ++i)
			{
				if (contact(i) == contactPast(i))
				{
					// �л����
					switchStatus(i) = 0;
				}
				else
				{
					contact(i) = contactPast(i);
					phase(i) = phasePast(i);
				}
			}
			if (switchStatus.sum() == 0)
			{
				statusPast = status;
			}
		}

		*gaitPhase = phase;
		*gaitContact = contact;
	}

	double GaitCtrl::cycloidXYPosition(double start, double end, double phase) {
		double phasePI = 2 * M_PI * phase;
		return (end - start) * (phasePI - sin(phasePI)) / (2 * M_PI) + start;
	}

	double GaitCtrl::cycloidXYVelocity(double start, double end, double phase) {
		double phasePI = 2 * M_PI * phase;
		return (end - start) * (1 - cos(phasePI)) / Tswing;
	}

	double GaitCtrl::cycloidZPosition(double start, double h, double phase) {
		double phasePI = 2 * M_PI * phase;
		return h * (1 - cos(phasePI)) / 2 + start;
	}

	double GaitCtrl::cycloidZVelocity(double h, double phase) {
		double phasePI = 2 * M_PI * phase;
		return h * M_PI * sin(phasePI) / Tswing;
	}

	void GaitCtrl::setGait(Eigen::Vector2d _VxyTargetGlobal, double _dYawTarget, double _gaitHeight)
	{
		this->VxyTarget = _VxyTargetGlobal;
		this->dYawTarget = _dYawTarget;
		this->gaitHeight = _gaitHeight;
	}

	void GaitCtrl::run(Eigen::Matrix<double, 3, 4>& _feetPos, Eigen::Matrix<double, 3, 4>& _feetVel)
	{
		if (is_firstRun) {
			//startP = bodyController->bodyObject->getEstFeetPos();
			startP = bodyController->bodyObject->getFKFeetPos();
			is_firstRun = false;
		}

		for (int i(0); i < 4; ++i) {
			if ((*gaitContact)(i) == 1) {
				if ((*gaitPhase)(i) < 0.5) {
					//startP.col(i) = bodyController->bodyObject->getEstFeetPos(i);
					startP.col(i) = bodyController->bodyObject->getFKFeetPos(i);
				}
				_feetPos.col(i) = startP.col(i);
				_feetVel.col(i).setZero();
			}
			else {
				endP.col(i) = calFootPos(i, VxyTarget, dYawTarget, (*gaitPhase)(i));

				_feetPos.col(i) = getFootPos(i);
				_feetVel.col(i) = getFootVel(i);
			}
		}
		pastP = _feetPos;
		gaitPhasePast = *gaitPhase;
	}

	Eigen::Vector3d GaitCtrl::getFootPos(int i)
	{
		Eigen::Vector3d footPos;

		footPos(0) = cycloidXYPosition(startP.col(i)(0), endP.col(i)(0), (*gaitPhase)(i));
		footPos(1) = cycloidXYPosition(startP.col(i)(1), endP.col(i)(1), (*gaitPhase)(i));
		footPos(2) = cycloidZPosition(startP.col(i)(2), gaitHeight, (*gaitPhase)(i));

		return footPos;
	}

	Eigen::Vector3d GaitCtrl::getFootVel(int i)
	{
		Eigen::Vector3d footVel;

		footVel(0) = cycloidXYVelocity(startP.col(i)(0), endP.col(i)(0), (*gaitPhase)(i));
		footVel(1) = cycloidXYVelocity(startP.col(i)(1), endP.col(i)(1), (*gaitPhase)(i));
		footVel(2) = cycloidZVelocity(gaitHeight, (*gaitPhase)(i));

		return footVel;
	}

	void GaitCtrl::restart()
	{
		this->is_firstRun = true;
		this->VxyTarget.setZero();
	}
}
