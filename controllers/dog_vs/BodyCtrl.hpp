#pragma once
#include "Body.hpp"
#include "mpcCal.hpp"
#include "../../libraries/myLibs/PIDmethod.h"
#include "LegCtrl.hpp"

using namespace Eigen;

namespace Quadruped
{
    // ����ƽ���������״̬
    typedef struct _balanceState
    {
        Vector3d p = Vector3d::Zero();
        Vector3d p_dot = Vector3d::Zero();
        Vector3d r = Vector3d::Zero();
        Vector3d r_dot = Vector3d::Zero();
    } balanceState;

    class BodyCtrl
    {
    public:
        // ����ƽ���������״̬����
        balanceState targetBalanceState;
        balanceState currentBalanceState;
        // �����ƵĻ����˱���
        Body* bodyObject;
        // �Ȳ�������
        LegCtrl* legsCtrl[4];
        // ������ƽ�������
        mpcCal<6, 12, 20, 1, 5> balanceController;
        // mpc���������
        //Vector<double, 12> mpcOut = Vector<double, 12>::Zero();
        Eigen::Matrix<double, 3, 4> mpcOut = Eigen::Matrix<double, 3, 4>::Zero();
        // ����mpc�������ľ���
        Eigen::Matrix<double, 6, 6> A;
        Eigen::Matrix<double, 6, 12> B;
        // ������Ȩ�ز���
        Eigen::Matrix<double, 6, 6> Q; // ״̬Ȩ�ؾ���
        Eigen::Matrix<double, 6, 6> F; // �ն˲�������
        Eigen::Matrix<double, 12, 12> R; // ����Ȩ�ؾ���
        Eigen::Matrix<double, 12, 12> W; // ����ƽ������
        // Լ������
        Eigen::Matrix<double, 12, 1> lb; // ����Լ��
        Eigen::Matrix<double, 12, 1> ub;
        Eigen::Matrix<double, 20, 12> cA; // boxԼ������
        Eigen::Matrix<double, 20, 1> Alb; // boxԼ����Ե
        Eigen::Matrix<double, 20, 1> Aub;
        // ״̬��
        Eigen::Vector<double, 6> y;
        Eigen::Vector<double, 6> x;
        // һЩ������
        double u = 2;
        double force_c = 200;
        double dt;

        // flags
        bool is_init = false;

        // ���ڻ���λ�ص�pid
        PIDmethod linPID[3];
        PIDmethod angPID[3];

    public:
        // ���캯��
        BodyCtrl(Body* _obj, LegCtrl* _legsCtrl[4], int timeStep);
        // ����Ȩ�ز���
        void importWeight(Vector<double, 6> _Q, Vector<double, 6> _F, Vector<double, 12> _R, Vector<double, 12> _W);
        void importPDparam(Vector<double, 9> lin, Vector<double, 9> ang);
        // ���µ�ǰ״̬
        void updateBalanceState();
        // �����ն�Ŀ��
        void setBalanceTarget(Vector3d _p, Vector3d _r);
        // ִ��mpc������
        void mpc_adjust();
        // �����Ȳ��Ӵ�Լ��
        void setContactConstrain(Vector4i _contact);
        // ֱ�������������
        void setLegsForce(Eigen::Matrix<double, 3, 4> _force);
    };

    BodyCtrl::BodyCtrl(Body* _obj, LegCtrl* _legsCtrl[4], int timeStep) : bodyObject(_obj)
    {
        for (int i = 0; i < 4; i++)
        {
            this->legsCtrl[i] = _legsCtrl[i];
        }
        dt = static_cast<double>(timeStep) * 0.001;
        A.setZero();
        /*A.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
        A.block<3, 3>(6, 9) = Eigen::Matrix3d::Identity();*/
        B.setZero();
        Q.setZero();
        R.setZero();
        lb.setConstant(-force_c);
        ub.setConstant(force_c);
        cA.setZero();

        // mit constrain
        //Eigen::Matrix<double, 5, 3> _cA;
        //_cA.setZero();
        //_cA << 1, 0, -u, 1, 0, u, 0, 1, -u, 0, 1, u, 0, 0, 1;
        ///*cA.block<5, 3>(0, 0) = _cA;
        //cA.block<5, 3>(5, 3) = _cA;
        //cA.block<5, 3>(10, 6) = _cA;
        //cA.block<5, 3>(15, 9) = _cA;*/
        //Eigen::Matrix<double, 5, 1> _Alb;
        //_Alb.setZero();
        //_Alb << std::numeric_limits<double>::min(), 0, std::numeric_limits<double>::min(), 0, -force_c;
        //Alb.block<5, 1>(0, 0) = _Alb;
        //Alb.block<5, 1>(5, 0) = _Alb;
        //Alb.block<5, 1>(10, 0) = _Alb;
        //Alb.block<5, 1>(15, 0) = _Alb;
        //Eigen::Matrix<double, 5, 1> _Aub;
        //_Aub.setZero();
        //_Aub << 0, std::numeric_limits<double>::max(), 0, std::numeric_limits<double>::max(), force_c;
        //Aub.block<5, 1>(0, 0) = _Aub;
        //Aub.block<5, 1>(5, 0) = _Aub;
        //Aub.block<5, 1>(10, 0) = _Aub;
        //Aub.block<5, 1>(15, 0) = _Aub;
        
        // unitree constrain
        Eigen::Matrix<double, 5, 3> _cA;
        _cA.setZero();
        _cA << 1, 0, u, -1, 0, u, 0, 1, u, 0, -1, u, 0, 0, 1;
        cA.block<5, 3>(0, 0) = _cA;
        cA.block<5, 3>(5, 3) = _cA;
        cA.block<5, 3>(10, 6) = _cA;
        cA.block<5, 3>(15, 9) = _cA;
        Alb.setZero();
        Aub.setConstant(100000.);
    }

    void BodyCtrl::importWeight(Vector<double, 6> _Q, Vector<double, 6> _F, Vector<double, 12> _R, Vector<double, 12> _W)
    {
        for (int i = 0; i < 6; i++)
        {
            Q(i, i) = _Q(i);
            F(i, i) = _F(i);
        }
        for (int j = 0; j < 12; j++)
        {
            R(j, j) = _R(j);
            W(j, j) = _W(j);
        }
    }

    void BodyCtrl::importPDparam(Vector<double, 9> lin, Vector<double, 9> ang)
    {
        for (int i = 0; i < 3; i++)
        {
            linPID[i].Params_Config(lin(0 + i * 3), 0, lin(1 + i * 3), 0, abs(lin(2 + i * 3)), -abs(lin(2 + i * 3)));
            angPID[i].Params_Config(ang(0 + i * 3), 0, ang(1 + i * 3), 0, abs(ang(2 + i * 3)), -abs(ang(2 + i * 3)));
        }
    }

    void BodyCtrl::updateBalanceState()
    {
        currentBalanceState.p = bodyObject->currentWorldState.dist;
        currentBalanceState.p_dot = bodyObject->currentWorldState.linVel_xyz;
        currentBalanceState.r = bodyObject->currentBodyState.Ang_xyz;
        currentBalanceState.r_dot = bodyObject->currentBodyState.angVel_xyz;
    }

    void BodyCtrl::setBalanceTarget(Vector3d _p, Vector3d _r)
    {
        targetBalanceState.p = _p;
        targetBalanceState.r = _r;
    }

    void BodyCtrl::setContactConstrain(Vector4i _contact)
    {
        Eigen::Matrix<double, 5, 3> _cA;
        _cA.setZero();
        Eigen::Matrix<double, 5, 1> _Aub;
        _Aub.setZero();
        // �����Ȳ����أ�������Լ������
        for (int i = 0; i < 4; i++)
        {
            if (_contact(i) == 1)
            {
                _cA << 1, 0, u, -1, 0, u, 0, 1, u, 0, -1, u, 0, 0, 1;
                _Aub.setConstant(100000.);
            }
            else
            {
                _cA << 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                _Aub.setZero();
            }
            this->cA.block<5, 3>(5 * i, 3 * i) = _cA;
            this->Aub.block<5, 1>(5 * i, 0) = _Aub;
        }
    }

    void BodyCtrl::mpc_adjust()
    {
        for (int i = 0; i < 3; i++)
        {
            linPID[i].target = targetBalanceState.p(i);
            linPID[i].current = currentBalanceState.p(i);
            linPID[i].Adjust(0, currentBalanceState.p_dot(i));
            targetBalanceState.p_dot(i) = linPID[i].out;

            angPID[i].target = targetBalanceState.r(i);
            angPID[i].current = currentBalanceState.r(i);
            angPID[i].Adjust(0, currentBalanceState.r_dot(i));
            targetBalanceState.r_dot(i) = angPID[i].out;
        }

        B = bodyObject->dynamicRight.inverse() * bodyObject->dynamicLeft;
        balanceController.setConstrain(lb, ub);
        balanceController.setBoxConstrain(cA, Alb, Aub);
        y.block<3, 1>(0, 0) = targetBalanceState.p_dot;
        y.block<3, 1>(3, 0) = targetBalanceState.r_dot;
        x.block<3, 1>(0, 0) = currentBalanceState.p_dot;
        x.block<3, 1>(3, 0) = currentBalanceState.r_dot;
        balanceController.mpc_update(y, x, 100, 0.002);
        balanceController.mpc_init(A, B, Q, F, R, W, dt);
        balanceController.mpc_solve();
        for (int i = 0; i < 4; i++)
        {
            this->mpcOut.col(i) = -bodyObject->Rsb_c.inverse() * balanceController.getOutput().block<3, 1>(3 * i, 0);
        }
    }

    void BodyCtrl::setLegsForce(Eigen::Matrix<double, 3, 4> _force)
    {
        for (int i = 0; i < 4; i++)
        {
            this->legsCtrl[i]->setEndForceTar(_force.col(i));
        }
    }
}