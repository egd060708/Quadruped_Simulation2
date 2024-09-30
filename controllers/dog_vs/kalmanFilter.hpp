#pragma once

#include <Eigen/Dense>

//using namespace Eigen;

// ���峣���������
#define Matrixd(r, c) Eigen::Matrix<double, r, c>
// ���巽�����ͣ�Square��
#define MatrixSd(d) Eigen::Matrix<double, d, d>

// ״̬ά�ȣ�����ά�ȣ����ά��
template <uint8_t xNum, uint8_t uNum, uint8_t yNum>
class kalmanFilter
{
private:
    MatrixSd(xNum) A = MatrixSd(xNum)::Zero();           // ״̬ת�ƾ���
    Matrixd(xNum, uNum) B = Matrixd(xNum, uNum)::Zero(); // �������
    MatrixSd(xNum) Q = MatrixSd(xNum)::Zero();           // ��������Э�������
    MatrixSd(yNum) R = MatrixSd(yNum)::Zero();           // ��������Э�������
    Matrixd(yNum, xNum) H = Matrixd(yNum, xNum)::Zero(); // �������
    MatrixSd(xNum) P = MatrixSd(xNum)::Identity();       // �������Э�������
    Matrixd(xNum, 1) x = Matrixd(xNum, 1)::Zero();       // ����ÿ�ε�״̬
    Matrixd(yNum, 1) y = Matrixd(yNum, 1)::Zero();       // �������
public:
    // ���캯��
    kalmanFilter() {}
    // ������ɢ״̬�ռ䷽�̣�����������
    void setFunc(MatrixSd(xNum) _A, Matrixd(xNum, uNum) _B, Matrixd(yNum, xNum) _H, double _Ts = 0)
    {
        if (_Ts <= 0)
        {
            this->A = _A;
            this->B = _B;
        }
        else
        {
            // ���������������Ҫ����ɢ��
            MatrixSd(xNum) AI = MatrixSd(xNum)::Identity();
            this->A = AI + _Ts * _A;
            this->B = _Ts * _B;
        }
        this->H = _H;
    }
    // ����Э�������ע�⣬Э���������Ժ�С��������Ϊ�㣩
    void setConv(MatrixSd(xNum) _Q, MatrixSd(yNum) _R, MatrixSd(xNum) _P = MatrixSd(xNum)::Identity())
    {
        this->Q = _Q;
        this->R = _R;
        this->P = _P;
    }
    void updateConv(MatrixSd(xNum) _Q, MatrixSd(yNum) _R)
    {
        this->Q = _Q;
        this->R = _R;
    }
    // ��⿨�����˲�(�������Ϊ״̬�����������룬�Լ��۲�ֵ����)
    void f(Matrixd(uNum, 1) _u, Matrixd(yNum, 1) _y)
    {
        // ��������״̬����
        Matrixd(xNum, 1) x_minus = A * x + B * _u;
        // �����������Э����
        MatrixSd(xNum) P_minus = A * P * A.transpose() + Q;
        // ���㿨��������
        MatrixSd(yNum) temp = H * P_minus * H.transpose() + R;

        // // ʹ�þ�������
        // Matrixd(xNum, yNum) K = P_minus * H.transpose() * temp.inverse();
        // // ���º������
        // x = x_minus + K * (_y - H * x_minus);
        // // ���º������Э����
        // MatrixSd(xNum) E = MatrixSd(xNum)::Identity();
        // P = (E - K * H) * P_minus;

        // ʹ��lu�ֽⷨ
        Eigen::PartialPivLU<Eigen::Matrix<double, 28, 28>> _tempLu = temp.lu();
        Eigen::Matrix<double, yNum, 1> _tempY = _tempLu.solve(_y - y);
        Eigen::Matrix<double, yNum, xNum> _tempH = _tempLu.solve(H);
        Eigen::Matrix<double, yNum, yNum> _tempR = _tempLu.solve(R);
        Eigen::Matrix<double, yNum, xNum> _tempTH = (temp.transpose()).lu().solve(H);
        Eigen::Matrix<double, xNum, xNum> _IKH = Eigen::Matrix<double, xNum, xNum>::Identity();
        _IKH = _IKH - P_minus * H.transpose() * _tempH;
        x = x_minus + P_minus * H.transpose() * _tempY;
        P = _IKH * P_minus * _IKH.transpose() + P_minus * H.transpose() * _tempR * _tempTH * P_minus.transpose();

        // �����������
        y = H * x;
    }
    Matrixd(yNum, 1) getOut()
    {
        return y;
    }
    Matrixd(xNum, 1) getState()
    {
        return x;
    }
};