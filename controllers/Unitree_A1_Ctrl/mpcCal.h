#pragma once

#include </usr/local/include/qpOASES.hpp>
#include <Eigen/Dense>
#include <limits>
#include <eigen3/unsupported/Eigen/KroneckerProduct>

USING_NAMESPACE_QPOASES
using namespace Eigen;

// 定义常规矩阵类型
#define Matrixr(r, c) Eigen::Matrix<real_t, r, c>
// 定义方阵类型（Square）
#define MatrixSr(d) Eigen::Matrix<real_t, d, d>
// 预编译选项，是否使用简单约束
#define SIMPLE_CONSTRAIN 0

#if SIMPLE_CONSTRAIN
template <int_t xNum, int_t uNum, int_t preStep = 10, int_t ctrlStep = 5> // 问题规模(状态变量和输入变量个数，预测及控制步长）
#else
template <int_t xNum, int_t uNum, int_t cNum, int_t preStep = 10, int_t ctrlStep = 5>
#endif
class mpcCal
{
public:
    // 离散状态空间方程
    MatrixSr(xNum) A = MatrixSr(xNum)::Zero();
    Matrixr(xNum, uNum) B = Matrixr(xNum, uNum)::Zero();
    // 权重矩阵，补偿矩阵
    MatrixSr(xNum) Q = MatrixSr(xNum)::Identity();
    MatrixSr(uNum) R = MatrixSr(uNum)::Zero();
    MatrixSr(xNum) F = MatrixSr(xNum)::Zero();
    // 状态向量
    Matrixr(xNum, 1) Y = Matrixr(xNum, 1)::Zero(); // 目标向量
    Matrixr(xNum, 1) X = Matrixr(xNum, 1)::Zero(); // 当前状态
    Matrixr(uNum, 1) U = Matrixr(uNum, 1)::Zero(); // 输出向量
    // 中间变量
    MatrixSr(xNum) G = MatrixSr(xNum)::Zero();
    Matrixr(ctrlStep *uNum, xNum) E = Matrixr(ctrlStep * uNum, xNum)::Zero();
    Matrixr(uNum *ctrlStep, (ctrlStep + 1) * xNum) L = Matrixr(uNum * ctrlStep, (ctrlStep + 1) * xNum)::Zero();
    MatrixSr(ctrlStep *uNum) H = MatrixSr(ctrlStep * uNum)::Zero();
    MatrixSr(ctrlStep *uNum) extraH = MatrixSr(ctrlStep * uNum)::Zero();
    Matrixr(ctrlStep *uNum, 1) extra_g = Matrixr(ctrlStep * uNum, 1)::Zero();
    // 预测结果
    Matrixr(xNum *(ctrlStep + 1), 1) Y_K = Matrixr(xNum * (ctrlStep + 1), 1)::Zero(); // qp求解给定
    Matrixr(xNum, preStep + 1) X_K = Matrixr(xNum, preStep + 1)::Zero();              // qp求解状态
    Matrixr(uNum, preStep) U_K = Matrixr(uNum, preStep)::Zero();                      // qp求解输出
    Matrixr(2 * xNum, 1) X_COMPARE = Matrixr(2 * xNum, 1)::Zero();                    // 对齐时间戳后的状态，预测在低位，实际在高位
    // 约束矩阵
    // 输入约束
    Matrixr(uNum *ctrlStep, 1) lb = Matrixr(1, uNum *ctrlStep)::Constant(std::numeric_limits<real_t>::min());
    Matrixr(uNum *ctrlStep, 1) ub = Matrixr(1, uNum *ctrlStep)::Constant(std::numeric_limits<real_t>::max());
#if not SIMPLE_CONSTRAIN
    // 更过box约束
    Matrixr(cNum *ctrlStep, uNum *ctrlStep) cA = Matrixr(cNum * ctrlStep, uNum *ctrlStep)::Zero();
    Matrixr(cNum *ctrlStep, 1) Alb = Matrixr(cNum * ctrlStep, 1)::Constant(std::numeric_limits<real_t>::min());
    Matrixr(cNum *ctrlStep, 1) Aub = Matrixr(cNum * ctrlStep, 1)::Constant(std::numeric_limits<real_t>::max());
#endif
    // qp求解器
#if SIMPLE_CONSTRAIN
    QProblemB qp_solver;
#else
    SQProblem qp_solver;
#endif
    // qp求解执行次数
    int_t nWSR_static = 20; // 最大单轮qp迭代次数
    int_t nWSR = 20;
    real_t CPU_t_static = 0.002; // 最长CPU使用时间
    real_t CPU_t = 0.002;
    uint8_t isModelUpdate = 1; // 系统模型是否更新

public:
#if SIMPLE_CONSTRAIN
    mpcCal() : qp_solver(ctrlStep * uNum, HST_POSDEF)
    {
        Options option;
        option.printLevel = PL_NONE; // 禁用qpOASES库的打印输出
        qp_solver.setOptions(option);
    }
#else
    mpcCal() : qp_solver(ctrlStep * uNum, ctrlStep * cNum, HST_POSDEF)
    {
        Options option;
        option.printLevel = PL_NONE; // 禁用qpOASES库的打印输出
        qp_solver.setOptions(option);
    }
#endif

    // qp求解得预测输出
    Matrixr(uNum, 1) prediction(Matrixr(xNum *(ctrlStep + 1), 1) & y_k, Matrixr(xNum, 1) & x_k)
    {
        real_t qp_out[ctrlStep * uNum];
        Matrixr(ctrlStep * uNum, 1) g_new;
        g_new = E * x_k - L * y_k + extra_g;
        MatrixSr(ctrlStep * uNum) H_new;
        H_new = H + extraH;

#if SIMPLE_CONSTRAIN
        if (isModelUpdate == 1)
        {
            qp_solver.init(H_new.data(), g_new.data(), lb.data(), ub.data(), nWSR, &CPU_t);
            isModelUpdate = 0;
        }
        else
        {
            qp_solver.hotstart(g_new.data(), lb.data(), ub.data(), nWSR, &CPU_t);
        }
#else
        if (isModelUpdate == 1)
        {
            qp_solver.init(const_cast<const real_t*>(H_new.data()), const_cast<const real_t*>(g_new.data()), const_cast<const real_t*>(cA.data()), const_cast<const real_t*>(lb.data()), const_cast<const real_t*>(ub.data()), const_cast<const real_t*>(Alb.data()), const_cast<const real_t*>(Aub.data()), nWSR, &CPU_t);
            isModelUpdate = 0;
        }
        else
        {
            qp_solver.hotstart(const_cast<const real_t*>(H_new.data()), const_cast<const real_t*>(g_new.data()), const_cast<const real_t*>(cA.data()), const_cast<const real_t*>(lb.data()), const_cast<const real_t*>(ub.data()), const_cast<const real_t*>(Alb.data()), const_cast<const real_t*>(Aub.data()), nWSR, &CPU_t);
        }
#endif

        nWSR = nWSR_static;
        CPU_t = CPU_t_static;
        qp_solver.getPrimalSolution(qp_out);
        // std::cout << qp_out[0] << " " << qp_out[1] << " " << qp_out[2] << std::endl;

        Matrixr(uNum, 1) result = Eigen::Map<Matrixr(uNum, 1)>(qp_out);
        return result;
    }
    // mpc控制器参数矩阵生成
    void mpc_matrices()
    {
        Matrixr((ctrlStep + 1) * xNum, xNum) M = Matrixr((ctrlStep + 1) * xNum, xNum)::Identity();
        M.block(0, 0, xNum, xNum) = Matrixr(xNum, xNum)::Identity();
        Matrixr((ctrlStep + 1) * xNum, ctrlStep * uNum) C = Matrixr((ctrlStep + 1) * xNum, ctrlStep * uNum)::Zero();
        MatrixSr(xNum) tmp = MatrixSr(xNum)::Identity();
        // 填充C矩阵和M矩阵
        for (int i = 1; i <= ctrlStep; i++)
        {
            int rowStart = i * xNum;
            C.block(rowStart, 0, xNum, uNum) = tmp * B;
            if (rowStart > xNum)
            {
                C.block(rowStart, uNum, xNum, C.cols() - uNum) = C.block(rowStart - xNum, 0, xNum, C.cols() - uNum);
            }
            tmp = A * tmp;
            M.block(rowStart, 0, xNum, xNum) = tmp;
        }
        // // 构建kron积
        // MatrixSr(ctrlStep) tmp1 = MatrixSr(ctrlStep)::Identity();
        // Eigen::MatrixX<real_t> _Q_bar = Eigen::kroneckerProduct(tmp1, Q);
        // Eigen::MatrixX<real_t> R_bar = Eigen::kroneckerProduct(tmp1, R);
        // // Matrixr(tmp1.rows() * Q.rows(), tmp1.cols() * Q.cols()) _Q_bar = Eigen::kroneckerProduct(tmp1, Q);
        // // Matrixr(tmp1.rows() * R.rows(), tmp1.cols() * R.cols()) R_bar = Eigen::kroneckerProduct(tmp1, R);
        // // 构建分块对角矩阵
        // // Matrixr(_Q_bar.rows() + F.rows(), _Q_bar.cols() + F.cols()) Q_bar = Matrixr(_Q_bar.rows() + F.rows(), _Q_bar.cols() + F.cols())::Zero();
        // Eigen::MatrixX<real_t> Q_bar(_Q_bar.rows() + F.rows(), _Q_bar.cols() + F.cols());
        // Q_bar.setZero();
        // Q_bar.block(0, 0, _Q_bar.rows(), _Q_bar.cols()) = _Q_bar;
        // // Q_bar.block<_Q_bar.rows(), _Q_bar.cols()>(0, 0) = _Q_bar;
        // Q_bar.block(_Q_bar.rows(), _Q_bar.cols(), F.rows(), F.cols()) = F;
        // // Q_bar.block<F.rows(), F.cols()>(_Q_bar.rows(), _Q_bar.cols()) = F;

        // 构建kron积
        Matrixr((ctrlStep + 1) * xNum, (ctrlStep + 1) * xNum) Q_bar;
        Matrixr(ctrlStep * xNum, ctrlStep * xNum) R_bar;
        Q_bar.setZero();
        R_bar.setZero();
        for (int i = 0; i < ctrlStep; i++)
        {
            Q_bar.block(i * xNum, i * xNum, xNum, xNum) = Q;
            R_bar.block(i * xNum, i * xNum, xNum, xNum) = R;
        }
        Q_bar.block(ctrlStep * xNum, ctrlStep * xNum, xNum, xNum) = F;

        G = M.transpose() * Q_bar * M;         // G: n x n
        L = C.transpose() * Q_bar;             // F: NP x n
        E = L * M;                             // E: NP x n
        H = C.transpose() * Q_bar * C + R_bar; // NP x NP
    }
    // mpc初始化
    void mpc_init(MatrixSr(xNum) _A, Matrixr(xNum, uNum) _B, MatrixSr(xNum) _Q, MatrixSr(uNum) _R, MatrixSr(xNum) _F, real_t _Ts = 0)
    {
        if (_Ts <= 0)
        {
            // 输入的是离散
            this->A = _A;
            this->B = _B;
        }
        else
        {
            // 输入的是连续，需要做离散化
            MatrixSf(xNum) AI = MatrixSf(xNum)::Identity();
            this->A = AI + _Ts * _A;
            this->B = _Ts * _B;
        }
        this->Q = _Q;
        this->R = _R;
        this->F = _F;
        mpc_matrices();
    }
    // 控制器状态更新
    void mpc_update(Matrixr(xNum, 1) _Y, Matrixr(xNum, 1) _X, int_t _nWSR = 10, real_t _cpu_t = 1)
    {
        this->Y = _Y;
        this->X = _X;
        this->nWSR_static = _nWSR;
        this->CPU_t_static = _cpu_t;
        this->X_K.block(0, 0, xNum, 1) = this->X;
        for (int i = 0; i <= ctrlStep; i++)
        {
            this->Y_K.block(i * xNum, 0, xNum, 1) = this->Y;
        }
    }
    // 设置额外的代价
    void setExtraCost(MatrixSr(uNum) _extraH, Matrixr(uNum, 1) _extra_g)
    {
        for (int i = 0; i < ctrlStep; i++)
        {
            this->extraH.block(i * uNum, i * uNum, uNum, uNum) = _extraH;
            this->extra_g.block(i * uNum, 0, uNum, 1) = _extra_g;
        }
    }
    // 设置输入约束（上下限）
    void setConstrain(Matrixr(uNum, 1) _lb, Matrixr(uNum, 1) _ub)
    {
        for (int i = 0; i < ctrlStep; i++)
        {
            this->lb.block(i * uNum, 0, uNum, 1) = _lb;
            this->ub.block(i * uNum, 0, uNum, 1) = _ub;
        }
    }
#if not SIMPLE_CONSTRAIN
    // 设置box约束（约束矩阵以及上下限）
    void setBoxConstrain(Matrixr(cNum, uNum) _cA, Matrixr(cNum, 1) _Alb, Matrixr(cNum, 1) _Aub)
    {
        this->cA.setZero();
        for (int i = 0; i < ctrlStep; i++)
        {
            this->cA.block(i * cNum, i * uNum, cNum, uNum) = _cA;
            this->Alb.block(i * cNum, 0, cNum, 1) = _Alb;
            this->Aub.block(i * cNum, 0, cNum, 1) = _Aub;
        }
    }
#endif
    // 控制器求解
    void mpc_solve()
    {
        // 执行预测
        Matrixr(xNum, 1) tmp_xk = X;
        Matrixr(uNum, 1) tmp_uk = Matrixr(uNum, 1)::Zero();
        for (int i = 0; i < preStep; i++)
        {
            tmp_uk = prediction(Y_K, tmp_xk);      // qp求解出当前的输出
            tmp_xk = A * tmp_xk + B * tmp_uk;      // 预测下一周期的状态
            X_K.block(0, i + 1, xNum, 1) = tmp_xk; // 把新状态记录下来
            U_K.block(0, i, uNum, 1) = tmp_uk;     // 把预测输出记录下来
        }
        U = U_K.block(0, 0, uNum, 1); // 选择第一个周期的输出作为最后输出
    }
    // 进行预测状态与实际状态的对齐存放(用于循环更新的过程中)
    void compare_storage()
    {
        static Matrixr(xNum, preStep) preStorage = Matrixr(xNum, preStep)::Zero();
        static uint_t count = 0;
        Matrixr(xNum, 1) get = Matrixr(xNum, 1)::Zero();
        Matrixr(2 * xNum, 1) put = Matrixr(2 * xNum, 1)::Zero();
        if (count > (preStep - 1))
        {
            count = 0;
        }
        put.block(0, 0, xNum, 1) = preStorage.block(0, count, xNum, 1);
        put.block(xNum, 0, xNum, 1) = X;
        X_COMPARE = put;
        get = X_K.block(0, preStep, xNum, 1);
        preStorage.block(0, count, xNum, 1) = get;
        count++;
    }
    // 获取输出值
    const Matrixr(uNum, 1) & getOutput()
    {
        return U;
    }
    // 获取预测向量
    const Matrixr(xNum, preStep + 1) & getPreState()
    {
        return X_K;
    }
    // 获取输出向量
    const Matrixr(uNum, preStep) & getPreCtrl()
    {
        return U_K;
    }
    // 获取对齐的预测与实际状态向量
    const Matrixr(2 * uNum, 1) & getCompareState()
    {
        return X_COMPARE;
    }
};