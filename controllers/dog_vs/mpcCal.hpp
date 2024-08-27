#pragma once

#include <qpOASES.hpp>
#include <Eigen/Dense>
#include <limits>

USING_NAMESPACE_QPOASES
using namespace Eigen;

// ���峣���������
#define Matrixr(r, c) Eigen::Matrix<real_t, r, c>
// ���巽�����ͣ�Square��
#define MatrixSr(d) Eigen::Matrix<real_t, d, d>
// Ԥ����ѡ��Ƿ�ʹ�ü�Լ��
#define SIMPLE_CONSTRAIN 0

#if SIMPLE_CONSTRAIN
template <int_t xNum, int_t uNum, int_t preStep = 10, int_t ctrlStep = 5> // �����ģ(״̬�������������������Ԥ�⼰���Ʋ�����
#else
template <int_t xNum, int_t uNum, int_t cNum, int_t preStep = 10, int_t ctrlStep = 5>
#endif
class mpcCal
{
public:
    // ����qpOASES������
    real_t H_qpOASES[ctrlStep * uNum * ctrlStep * uNum];
    real_t g_qpOASES[ctrlStep * uNum];
    real_t lb_qpOASES[uNum * ctrlStep];
    real_t ub_qpOASES[uNum * ctrlStep];
    real_t xOpt_qpOASES[ctrlStep * uNum];
    real_t yOpt_qpOASES[ctrlStep * uNum + ctrlStep * cNum];

    real_t xOpt_initialGuess[ctrlStep * uNum];
    // ��ɢ״̬�ռ䷽��
    MatrixSr(xNum) A = MatrixSr(xNum)::Zero();
    Matrixr(xNum, uNum) B = Matrixr(xNum, uNum)::Zero();
    // ״̬Ȩ�ؾ���Q��״̬ĩ�˲�������R������Ȩ�ؾ���R��ƽ������W
    MatrixSr(xNum) Q = MatrixSr(xNum)::Identity();
    MatrixSr(xNum) F = MatrixSr(xNum)::Identity();
    MatrixSr(uNum) R = MatrixSr(uNum)::Zero();
    MatrixSr(uNum) W = MatrixSr(uNum)::Zero();
    // ״̬����
    Matrixr(xNum, 1) Y = Matrixr(xNum, 1)::Zero(); // Ŀ������
    Matrixr(xNum, 1) X = Matrixr(xNum, 1)::Zero(); // ��ǰ״̬
    Matrixr(uNum, 1) U = Matrixr(uNum, 1)::Zero(); // �������
    // �м����
    MatrixSr(xNum) G = MatrixSr(xNum)::Zero();
    Matrixr(ctrlStep* uNum, xNum) E = Matrixr(ctrlStep * uNum, xNum)::Zero();
    Matrixr(uNum* ctrlStep, (ctrlStep + 1)* xNum) L = Matrixr(uNum * ctrlStep, (ctrlStep + 1) * xNum)::Zero();
    MatrixSr(ctrlStep* uNum) H = MatrixSr(ctrlStep * uNum)::Zero();
    MatrixSr(ctrlStep* uNum) extraH = MatrixSr(ctrlStep * uNum)::Zero();
    Matrixr(ctrlStep* uNum, 1) extra_g = Matrixr(ctrlStep * uNum, 1)::Zero();
    // ���̱���
    Matrixr((ctrlStep + 1)* xNum, xNum) M = Matrixr((ctrlStep + 1) * xNum, xNum)::Identity();
    Matrixr((ctrlStep + 1)* xNum, ctrlStep* uNum) C = Matrixr((ctrlStep + 1) * xNum, ctrlStep * uNum)::Zero();
    Matrixr((ctrlStep + 1)* xNum, (ctrlStep + 1)* xNum) Q_bar;
    Matrixr(ctrlStep* uNum, ctrlStep* uNum) R_bar;
    Matrixr(ctrlStep* uNum, ctrlStep* uNum) W_bar;
    Matrixr(ctrlStep* uNum, 1) g_new;
    MatrixSr(ctrlStep* uNum) H_new;
    /*MatrixXd H_new;
    MatrixXd g_new;*/
    // Ԥ����
    Matrixr(xNum* (ctrlStep + 1), 1) Y_K = Matrixr(xNum * (ctrlStep + 1), 1)::Zero(); // qp������
    Matrixr(xNum, preStep + 1) X_K = Matrixr(xNum, preStep + 1)::Zero();              // qp���״̬
    Matrixr(uNum * ctrlStep, preStep) U_K = Matrixr(uNum * ctrlStep, preStep)::Zero();                      // qp������
    Matrixr(uNum * ctrlStep, preStep) U_pre = Matrixr(uNum * ctrlStep, preStep)::Zero();                    // ��һ��qp�������
    Matrixr(2 * xNum, 1) X_COMPARE = Matrixr(2 * xNum, 1)::Zero();                    // ����ʱ������״̬��Ԥ���ڵ�λ��ʵ���ڸ�λ
    // Լ������
    // ����Լ��
    Matrixr(uNum* ctrlStep, 1) lb = Matrixr(uNum * ctrlStep, 1)::Constant(std::numeric_limits<real_t>::min());
    Matrixr(uNum* ctrlStep, 1) ub = Matrixr(uNum * ctrlStep, 1)::Constant(std::numeric_limits<real_t>::max());
    /*MatrixXd lb;
    MatrixXd ub;*/
//#if not SIMPLE_CONSTRAIN
    real_t cA_qpOASES[cNum * ctrlStep * uNum * ctrlStep];
    real_t Alb_qpOASES[cNum * ctrlStep];
    real_t Aub_qpOASES[cNum * ctrlStep];
    // boxԼ��
    Matrixr(cNum* ctrlStep, uNum* ctrlStep) cA = Matrixr(cNum * ctrlStep, uNum * ctrlStep)::Zero();
    Matrixr(cNum* ctrlStep, 1) Alb = Matrixr(cNum * ctrlStep, 1)::Constant(std::numeric_limits<real_t>::min());
    Matrixr(cNum* ctrlStep, 1) Aub = Matrixr(cNum * ctrlStep, 1)::Constant(std::numeric_limits<real_t>::max());
    /*MatrixXd cA;
    MatrixXd Alb;
    MatrixXd Aub;*/
    
//#endif
    // qp�����
#if SIMPLE_CONSTRAIN
    QProblemB qp_solver;
#else
    /*SQProblem qp_solver;*/
    SQProblem qp_solver;
    Bounds guessedBounds;
    Constraints guessedConstraints;
#endif
    // qp���ִ�д���
    int_t nWSR_static = 20; // �����qp��������
    int_t nWSR = 20;
    real_t CPU_t_static = 0.002; // �CPUʹ��ʱ��
    real_t CPU_t = 0.002;
    uint8_t isModelUpdate = 1; // ϵͳģ���Ƿ����

public:
#if SIMPLE_CONSTRAIN
    mpcCal() : qp_solver(ctrlStep* uNum, HST_POSDEF)
    {
        Options option;
        option.printLevel = PL_NONE; // ����qpOASES��Ĵ�ӡ���
        qp_solver.setOptions(option);
    }
#else
    mpcCal() : qp_solver(ctrlStep* uNum, ctrlStep* cNum, HST_UNKNOWN)
    {
        Options option;
        option.printLevel = PL_NONE; // ����qpOASES��Ĵ�ӡ���
        qp_solver.setOptions(option);
        qp_solver.setPrintLevel(PL_NONE);
        /*H_new.resize(ctrlStep * uNum, ctrlStep * uNum);
        H_new.setZero();
        g_new.resize(ctrlStep * uNum, 1);
        g_new.setZero();
        lb.resize(uNum * ctrlStep, 1);
        lb.Constant(std::numeric_limits<real_t>::min());
        ub.resize(uNum * ctrlStep, 1);
        ub.Constant(std::numeric_limits<real_t>::max());
        cA.resize(cNum * ctrlStep, uNum * ctrlStep);
        cA.setZero();
        Alb.resize(cNum * ctrlStep, 1);
        Alb.Constant(std::numeric_limits<real_t>::min());
        Aub.resize(cNum * ctrlStep, 1);
        Aub.Constant(std::numeric_limits<real_t>::max());*/
        for (int i = 0; i < ctrlStep * uNum; i++)
        {
            xOpt_qpOASES[i] = 0;
            xOpt_initialGuess[i] = 0;
        }
        xOpt_initialGuess[2] = 100;
        xOpt_initialGuess[5] = 100;
        xOpt_initialGuess[8] = 100;
        xOpt_initialGuess[11] = 100;
        for (int i = 0; i < ctrlStep * uNum + ctrlStep * cNum; i++) {
            yOpt_qpOASES[i] = 0.0;
        }
    }
#endif

    // qp����Ԥ�����
    Matrixr(uNum * ctrlStep, 1) prediction(Matrixr(xNum* (ctrlStep + 1), 1)& y_k, Matrixr(xNum, 1)& x_k)
    {
        real_t qp_out[ctrlStep * uNum];
        
        g_new = E * x_k - L * y_k - W_bar * U_pre.block(0, 0, uNum * ctrlStep, 1) + extra_g;
        
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
        // ����eigen��ľ����ǰ��д洢�������Ҫ�ֶ�ת��Ϊ����
        for (int i = 0; i < H_new.rows(); i++)
            for (int j = 0; j < H_new.cols(); j++)
                H_qpOASES[i * H_new.cols() + j] = H_new(i, j);
        for (int i = 0; i < g_new.rows(); i++)
            for (int j = 0; j < g_new.cols(); j++)
                g_qpOASES[i * g_new.cols() + j] = g_new(i, j);
        for (int i = 0; i < cA.rows(); i++)
            for (int j = 0; j < cA.cols(); j++)
                cA_qpOASES[i * cA.cols() + j] = cA(i, j);
        for (int i = 0; i < lb.rows(); i++)
            for (int j = 0; j < lb.cols(); j++)
                lb_qpOASES[i * lb.cols() + j] = lb(i, j);
        for (int i = 0; i < ub.rows(); i++)
            for (int j = 0; j < ub.cols(); j++)
                ub_qpOASES[i * ub.cols() + j] = ub(i, j);
        for (int i = 0; i < Alb.rows(); i++)
            for (int j = 0; j < Alb.cols(); j++)
                Alb_qpOASES[i * Alb.cols() + j] = Alb(i, j);
        for (int i = 0; i < Aub.rows(); i++)
            for (int j = 0; j < Aub.cols(); j++)
                Aub_qpOASES[i * Aub.cols() + j] = Aub(i, j);
        if (isModelUpdate == 1)
        {
            qp_solver.init(H_qpOASES, g_qpOASES, cA_qpOASES, lb_qpOASES, ub_qpOASES, Alb_qpOASES, Aub_qpOASES, nWSR, &CPU_t, xOpt_initialGuess);
            isModelUpdate = 0;
        }
        else
        {
            //qp_solver.init(H_qpOASES, g_qpOASES, cA_qpOASES, lb_qpOASES, ub_qpOASES, Alb_qpOASES, Aub_qpOASES, nWSR, &CPU_t, xOpt_qpOASES, yOpt_qpOASES, &guessedBounds, &guessedConstraints);
            qp_solver.hotstart(H_qpOASES, g_qpOASES, cA_qpOASES, lb_qpOASES, ub_qpOASES, Alb_qpOASES, Aub_qpOASES, nWSR, &CPU_t, &guessedBounds, &guessedConstraints);
        }
#endif

        nWSR = nWSR_static;
        CPU_t = CPU_t_static;
        qp_solver.getPrimalSolution(xOpt_qpOASES);
        qp_solver.getDualSolution(yOpt_qpOASES);
        qp_solver.getBounds(guessedBounds);
        qp_solver.getConstraints(guessedConstraints);
        //qp_solver.reset();
        
        Matrixr(uNum * ctrlStep, 1) result;
        for (int i = 0; i < uNum * ctrlStep; i++)
        {
            result(i, 0) = xOpt_qpOASES[i];
        }
        return result;
    }
    // mpc������������������
    void mpc_matrices()
    {
        
        M.block(0, 0, xNum, xNum) = Matrixr(xNum, xNum)::Identity();
        
        MatrixSr(xNum) tmp = MatrixSr(xNum)::Identity();
        // ���C�����M����
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

        // ����kron��
        Q_bar.setZero();
        R_bar.setZero();
        W_bar.setZero();
        for (int i = 0; i < ctrlStep; i++)
        {
            Q_bar.block(i * xNum, i * xNum, xNum, xNum) = Q;
            R_bar.block(i * uNum, i * uNum, uNum, uNum) = R;
            W_bar.block(i * uNum, i * uNum, uNum, uNum) = W;
        }
        Q_bar.block(ctrlStep * xNum, ctrlStep * xNum, xNum, xNum) = F;

        G = M.transpose() * Q_bar * M;         // G: n x n
        L = C.transpose() * Q_bar;             // F: NP x n
        E = L * M;                             // E: NP x n
        H = C.transpose() * Q_bar * C + R_bar + W_bar; // NP x NP
    }
    // mpc��ʼ��
    void mpc_init(MatrixSr(xNum) _A, Matrixr(xNum, uNum) _B, MatrixSr(xNum) _Q, MatrixSr(xNum) _F, MatrixSr(uNum) _R, MatrixSr(uNum) _W, real_t _Ts = 0)
    {
        if (_Ts <= 0)
        {
            // ���������ɢ
            this->A = _A;
            this->B = _B;
        }
        else
        {
            // ���������������Ҫ����ɢ��
            MatrixSr(xNum) AI = MatrixSr(xNum)::Identity();
            this->A = AI + _Ts * _A;
            this->B = _Ts * _B;
        }
        this->Q = _Q;
        this->F = _F;
        this->R = _R;
        this->W = _W;
        mpc_matrices();
    }
    // ������״̬����
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
    // ���ö���Ĵ���
    void setExtraCost(MatrixSr(uNum) _extraH, Matrixr(uNum, 1) _extra_g)
    {
        for (int i = 0; i < ctrlStep; i++)
        {
            this->extraH.block(i * uNum, i * uNum, uNum, uNum) = _extraH;
            this->extra_g.block(i * uNum, 0, uNum, 1) = _extra_g;
        }
    }
    // ��������Լ���������ޣ�
    void setConstrain(Matrixr(uNum, 1) _lb, Matrixr(uNum, 1) _ub)
    {
        for (int i = 0; i < ctrlStep; i++)
        {
            this->lb.block(i * uNum, 0, uNum, 1) = _lb;
            this->ub.block(i * uNum, 0, uNum, 1) = _ub;
        }
    }
#if not SIMPLE_CONSTRAIN
    // ����boxԼ����Լ�������Լ������ޣ�
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
    // ���������
    void mpc_solve()
    {
        // ִ��Ԥ��
        Matrixr(xNum, 1) tmp_xk = X;
        Matrixr(uNum * ctrlStep, 1) tmp_uk = Matrixr(uNum * ctrlStep, 1)::Zero();
        for (int i = 0; i < preStep; i++)
        {
            tmp_uk = prediction(Y_K, tmp_xk);      // qp������ǰ�����
            tmp_xk = A * tmp_xk + B * tmp_uk.block(0,0,uNum,1);      // Ԥ����һ���ڵ�״̬
            X_K.block(0, i + 1, xNum, 1) = tmp_xk; // ����״̬��¼����
            U_K.block(0, i, uNum * ctrlStep, 1) = tmp_uk;     // ��Ԥ�������¼����
        }
        U = U_K.block(0, 0, uNum, 1); // ѡ���һ�����ڵ������Ϊ������
        U_pre = U_K; // ������һ���������
    }
    // ����Ԥ��״̬��ʵ��״̬�Ķ�����(����ѭ�����µĹ�����)
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
    // ��ȡ���ֵ
    Matrixr(uNum, 1) getOutput()
    {
        return U;
    }
    // ��ȡԤ������
    Matrixr(xNum, preStep + 1) getPreState()
    {
        return X_K;
    }
    // ��ȡ�������
    Matrixr(uNum * ctrlStep, preStep) getPreCtrl()
    {
        return U_K;
    }
    // ��ȡ�����Ԥ����ʵ��״̬����
    Matrixr(2 * uNum, 1) getCompareState()
    {
        return X_COMPARE;
    }
};