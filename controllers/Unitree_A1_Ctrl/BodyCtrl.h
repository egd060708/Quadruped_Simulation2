#pragma once
#include "Body.h"
#include "mpcCal.h"

using namespace Eigen;

namespace Quadruped
{
    // 用于平衡控制器的状态
    typedef struct _balanceState
    {
        Vector3f p = Vector3f::Zero();
        Vector3f p_dot = Vector3f::Zero();
        Vector3f r = Vector3f::Zero();
        Vector3f r_dot = Vector3f::Zero();
    } balanceState;

    class BodyCtrl
    {
    private:
        // 用于平衡控制器的状态变量
        balanceState targetBalanceState;
        balanceState currentBalanceState;
        // 被控制的机器人本体
        Body *bodyObject;
        // 机器人平衡控制器
        mpcCal<12, 12, 20, 1, 5> balanceController;
        // mpc控制器输出
        Vector<float, 12> mpcOut = Vector<float, 12>::Zero();
        // 用于mpc控制器的矩阵
        Eigen::Matrix<float, 12, 12> A;
        Eigen::Matrix<float, 12, 12> B;
        // 控制器权重参数
        Eigen::Matrix<float, 12, 12> Q; // 状态权重矩阵
        Eigen::Matrix<float, 12, 12> R; // 输入权重矩阵
        Eigen::Matrix<float, 12, 12> F; // 终端补偿矩阵
        // 约束矩阵
        Eigen::Matrix<float, 12, 1> lb; // 输入约束
        Eigen::Matrix<float, 12, 1> ub;
        Eigen::Matrix<float, 20, 12> cA; // box约束矩阵
        Eigen::Matrix<float, 20, 1> Alb; // box约束边缘
        Eigen::Matrix<float, 20, 1> Aub;
        // 一些物理常数
        float u = 0.3;
        float force_c = 200;
        float dt;

        // flags
        bool is_init = false;

    public:
        // 构造函数
        BodyCtrl(Body *_obj,int timeStep);
        // 导入权重参数
        void importWeight(Vector<float, 12> _Q, Vector<float, 12> _R, Vector<float, 12> _F);
        // 更新当前状态
        void updateBalanceState();
        // 设置终端目标
        void setBalanceTarget(Vector3f _p, Vector3f _r);
        // 执行mpc控制器
        void mpc_adjust();
    };

    BodyCtrl::BodyCtrl(Body *_obj,int timeStep) : bodyObject(_obj)
    {
        dt = static_cast<float>(timeStep) * 0.001f;
        A.setZero();
        A.block<3, 3>(0, 3) = Eigen::Matrix3f::Identity();
        A.block<3, 3>(6, 9) = Eigen::Matrix3f::Identity();
        B.setZero();
        Q.setZero();
        R.setZero();
        lb.setConstant(-force_c);
        ub.setConstant(force_c);
        cA << 1, 0, u, -1, 0, u, 0, 1, u, 0, -1, u, 0, 0, 1;
        Alb.setConstant(0);
        Aub.setConstant(std::numeric_limits<float>::max());
    }

    void BodyCtrl::importWeight(Vector<float, 12> _Q, Vector<float, 12> _R, Vector<float, 12> _F)
    {
        for (int i = 0; i < 12; i++)
        {
            Q(i, i) = _Q(i);
            R(i, i) = _R(i);
            F(i, i) = _F(i);
        }
    }

    void BodyCtrl::updateBalanceState()
    {
        currentBalanceState.p = bodyObject->estimator.getState().segment(0, 3);
        currentBalanceState.p_dot = bodyObject->estimator.getState().segment(3, 3);
        currentBalanceState.r = bodyObject->currentBodyState.Ang_xyz;
        currentBalanceState.r_dot = bodyObject->currentBodyState.angVel_xyz;
    }

    void BodyCtrl::setBalanceTarget(Vector3f _p, Vector3f _r)
    {
        targetBalanceState.p = _p;
        targetBalanceState.r = _r;
    }

    void BodyCtrl::mpc_adjust()
    {
        // 得到输入矩阵的紧凑形式
        Eigen::Matrix<float, 6, 12> dB;
        dB = bodyObject->dynamicRight.inverse() * bodyObject->dynamicLeft;
        // 紧凑形式导入到状态空间方程中
        this->B.block<3, 12>(3, 0) = dB.block<3, 12>(0, 0);
        this->B.block<3, 12>(9, 0) = dB.block<3, 12>(3, 0);
        balanceController.setConstrain(lb,ub);
        balanceController.setBoxConstrain(cA,Alb,Aub);
        Eigen::Vector<float,12> y;
        y.block<3,1>(0,0) = targetBalanceState.p;
        y.block<3,1>(3,0) = targetBalanceState.p_dot;
        y.block<3,1>(6,0) = targetBalanceState.r;
        y.block<3,1>(9,0) = targetBalanceState.r_dot;
        Eigen::Vector<float,12> x;
        x.block<3,1>(0,0) = currentBalanceState.p;
        x.block<3,1>(3,0) = currentBalanceState.p_dot;
        x.block<3,1>(6,0) = currentBalanceState.r;
        x.block<3,1>(9,0) = currentBalanceState.r_dot;
        balanceController.mpc_update(y,x,20,0.002);
        balanceController.mpc_init(A,B,Q,R,F,dt);
        balanceController.mpc_solve();
    }
}