#pragma once
#include "Leg.hpp"
#include "kalmanFilter.hpp"

namespace Quadruped
{
    template<typename T>
    inline T windowFunc(const T x, const T windowRatio, const T xRange = 1.0, const T yRange = 1.0) {
        if ((x < 0) || (x > xRange)) {
            std::cout << "[ERROR][windowFunc] The x=" << x << ", which should between [0, xRange]" << std::endl;
        }
        if ((windowRatio <= 0) || (windowRatio >= 0.5)) {
            std::cout << "[ERROR][windowFunc] The windowRatio=" << windowRatio << ", which should between [0, 0.5]" << std::endl;
        }

        if (x / xRange < windowRatio) {
            return x * yRange / (xRange * windowRatio);
        }
        else if (x / xRange > 1 - windowRatio) {
            return yRange * (xRange - x) / (xRange * windowRatio);
        }
        else {
            return yRange;
        }
    }

    enum leg_id
    {
        LF = 0,
        RF = 1,
        LB = 2,
        RB = 3,
    };

    // ��������ϵ������״̬������ϵ��ʾΪ{s}
    typedef struct _worldFrame
    {
        LegS leg_s[4];
        Vector3d dist = Vector3d::Zero();       // ��������������ϵ�µ�λ��
        Vector3d angVel_xyz = Vector3d::Zero(); // ���ٶ�
        Vector3d linVel_xyz = Vector3d::Zero(); // ���ٶ�
        // Vector3d angAcc_xyz = Vector3d::Zero(); // �Ǽ��ٶ�
        Vector3d linAcc_xyz = Vector3d::Zero(); // ���Լ��ٶ�
    } worldFrame;
    // ����������ϵ�µ�״̬������ϵ��ʾΪ{b}
    typedef struct _bodyFrame
    {
        LegS leg_b[4];
        Vector3d Ang_xyz = Vector3d::Zero();    // �Ƕ�
        Vector4d Quat = Vector4d::Zero();       //��Ԫ��
        Vector3d angVel_xyz = Vector3d::Zero(); // ���ٶ�
        Vector3d linVel_xyz = Vector3d::Zero(); // ���ٶ�
        //Vector3d angAcc_xyz = Vector3d::Zero(); // �Ǽ��ٶ�
        Vector3d linAcc_xyz = Vector3d::Zero(); // ���Լ��ٶ�
    } bodyFrame;

    class Body
    {
    public:
        worldFrame targetWorldState;
        worldFrame currentWorldState;
        bodyFrame targetBodyState;
        bodyFrame currentBodyState;

        Leg* legs[4];
        Vector3d leg2body;                     // ��ʼֵ������ǰ������
        Matrix3d Rsb_c = Matrix3d::Identity(); // (��ǰ)��������ϵ����������ϵ����ת����ӳ��
        Matrix4d Tsb_c = Matrix4d::Identity(); // (��ǰ)��������ϵ����������ϵ����α任ӳ��
        Matrix3d Rsb_t = Matrix3d::Identity(); // (Ŀ��)��������ϵ����������ϵ����ת����ӳ��
        Matrix4d Tsb_t = Matrix4d::Identity(); // (Ŀ��)��������ϵ����������ϵ����α任ӳ��

        Matrix3d M = Matrix3d::Zero();      // �����˻�������
        Matrix3d Ib = Matrix3d::Zero();     // �����˵����嶯��ѧ����ת������
        Vector3d Pg = Vector3d::Zero();     // �����������ڻ�������ϵ�µ�λ�ã��൱��{PbPg}��
        Vector3d g = Vector3d(0, 0, -9.81); // ֱ�ӳ�ʼ���������ٶ�����
        double dt;                           // ��������
        double _largeVariance = 100;              // ���Э����
        double _trust;                      // �����Ȳ��Ƿ񴥵ص����Ŷ�

        Eigen::Matrix<double, 6, 12> dynamicLeft = Eigen::Matrix<double, 6, 12>::Zero();
        Eigen::Matrix<double, 6, 6> dynamicRight = Eigen::Matrix<double, 6, 6>::Zero();

        Vector3d initRbLegXYPosition; // ָ���Һ��ȳ�ʼ��ƽ�棩λ��

        // ������ת���ɽǶԳƾ���
        Matrix3d v3_to_m3(Vector3d _v);

        // ����״̬������
        kalmanFilter<18, 3, 28> estimator;
        Eigen::Matrix<double, 28, 18> _H = Eigen::Matrix<double, 28, 18>::Zero();
        Eigen::Matrix<double, 18, 18> _A = Eigen::Matrix<double, 18, 18>::Zero();
        Eigen::Matrix<double, 18, 3> _B = Eigen::Matrix<double, 18, 3>::Zero();
        Eigen::Matrix<double, 28, 28> _R = Eigen::Matrix<double, 28, 28>::Zero();
        Eigen::Matrix<double, 28, 28> _RInit = Eigen::Matrix<double, 28, 28>::Zero();
        Eigen::Matrix<double, 18, 18> _Q = Eigen::Matrix<double, 18, 18>::Zero();
        Eigen::Matrix<double, 18, 18> _QInit = Eigen::Matrix<double, 18, 18>::Zero();
        Eigen::Vector<double, 18> _Qdig = Eigen::Vector<double, 18>::Zero();
        Eigen::Matrix<double, 3, 3> _Cu = Eigen::Matrix<double, 3, 3>::Zero();
        Eigen::Matrix<double, 18, 18> _P = Eigen::Matrix<double, 18, 18>::Identity();

        Eigen::Vector<double, 28> estimatorOut = Eigen::Vector<double, 28>::Zero();
        Eigen::Vector<double, 18> estimatorState = Eigen::Vector<double, 18>::Zero();


    public:
        // ���캯�������ĸ��Ȳ�״̬��������
        Body(Leg* _legObj[4], double _dt);
        // ��ʼ�����������ڳ�ʼ���������
        void initParams(Vector3d _leg2body, Vector3d _initRbLegXYPosition, Vector<double,9> _M, Vector<double,9> _Ib, Vector3d _Pg);
        // ������α任����(directionΪ1�����ǵ�ǰ��Ϊ-1������Ŀ��)
        void calTbs(int8_t direction);
        // �����˶�ѧ���ı���������˵�λ�ôӶ��ı�����˻����λ�ú���̬
        // ���㵥�Ȼ�����ϵ���������ϵ�µ����λ��ת��(directionΪ1������(��ǰ)��->��Ϊ-1������(Ŀ��)��->��)
        void legAndBodyPosition(int8_t direction);
        // �����������ϵ����������ϵ�µ����λ��ת��(��������ϵ����Ϊ��ʼ״̬���Һ������λ��)��directionΪ1������(��ǰ)��->����Ϊ-1������(Ŀ��)��->��
        void bodyAndWorldFramePosition(int8_t direction);
        // �����������������ϵ�е�����ٶ�
        void legVelocityInWorldFrame();
        // ���»����˶���ѧ���̣�����Ϊ left*[f] = right��
        void updateDynamic();

        // ����Ŀ��λ��
        void updateBodyTargetPos(Vector3d _angle, Vector3d _position);
        // ���¹ߵ���̬
        void updateBodyImu(Vector3d _imuRPY);
        void updateBodyImu(Vector4d _imyQ);
        // ���������ǽ��ٶ�
        void updateBodyGyro(Vector3d _gyro);
        // ���¼��ٶȼƼ��ٶ�
        void updateBodyAcc(Vector3d _acc);
        // ����Ŀ�������(����)
        void updateTargetFootPoint(Eigen::Matrix<double,3,4> _point);
        // ����Ŀ�������ٶ�
        void updateTargetFootVel(Eigen::Matrix<double, 3, 4> _vel);

        // ״̬��������
        void estimatorRun(Vector4i _contact, Vector4d _phase);
        // ��ȡ���������λ
        Eigen::Matrix<double, 3, 4> getEstFeetPos();
        Eigen::Vector3d getEstFeetPos(int id);
        Eigen::Matrix<double, 3, 4> getEstFeetVel();
        Eigen::Vector3d getEstFeetVel(int id);
        Eigen::Matrix<double, 3, 4> getFKFeetPos();
        Eigen::Vector3d getFKFeetPos(int id);
        Eigen::Matrix<double, 3, 4> getFKFeetVel();
        Eigen::Vector3d getFKFeetVel(int i);

        // һЩ��ѧ���㺯��
        Eigen::Matrix3d quatToRot(const Vector4d& _quat);
        Eigen::Vector3d rotMatToRPY(const Matrix3d& R);
    };

    Body::Body(Leg* _legObj[4], double _dt)
    {
        for (int i = 0; i < 4; i++)
        {
            this->legs[i] = _legObj[i];
        }
        this->dt = _dt;

        Matrix3d Ident = Matrix3d::Identity(); // ��ʼ��һ��3ά�ԽǾ���
        dynamicLeft.block<3, 3>(0, 0) = Ident;
        dynamicLeft.block<3, 3>(0, 3) = Ident;
        dynamicLeft.block<3, 3>(0, 6) = Ident;
        dynamicLeft.block<3, 3>(0, 9) = Ident;

        // ��ʼ��״̬������
        // ״̬ת�ƾ���
        _A.block(0, 3, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
        // �������
        _B.block(3, 0, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
        // �������
        _H.block(0, 0, 3, 3) = -Eigen::Matrix<double, 3, 3>::Identity();
        _H.block(3, 0, 3, 3) = -Eigen::Matrix<double, 3, 3>::Identity();
        _H.block(6, 0, 3, 3) = -Eigen::Matrix<double, 3, 3>::Identity();
        _H.block(9, 0, 3, 3) = -Eigen::Matrix<double, 3, 3>::Identity();
        _H.block(12, 3, 3, 3) = -Eigen::Matrix<double, 3, 3>::Identity();
        _H.block(15, 3, 3, 3) = -Eigen::Matrix<double, 3, 3>::Identity();
        _H.block(18, 3, 3, 3) = -Eigen::Matrix<double, 3, 3>::Identity();
        _H.block(21, 3, 3, 3) = -Eigen::Matrix<double, 3, 3>::Identity();
        _H.block(0, 6, 12, 12) = Eigen::Matrix<double, 12, 12>::Identity();
        _H(24, 8) = 1;
        _H(25, 11) = 1;
        _H(26, 14) = 1;
        _H(27, 17) = 1;
        // ��ʼ��״̬�ռ䷽��
        estimator.setFunc(_A, _B, _H, dt);
        // ��������Э����
        _RInit << 0.008, 0.012, -0.000, -0.009, 0.012, 0.000, 0.009, -0.009, -0.000, -0.009, -0.009, 0.000, -0.000, 0.000, -0.000, 0.000, -0.000, -0.001, -0.002, 0.000, -0.000, -0.003, -0.000, -0.001, 0.000, 0.000, 0.000, 0.000,
            0.012, 0.019, -0.001, -0.014, 0.018, -0.000, 0.014, -0.013, -0.000, -0.014, -0.014, 0.001, -0.001, 0.001, -0.001, 0.000, 0.000, -0.001, -0.003, 0.000, -0.001, -0.004, -0.000, -0.001, 0.000, 0.000, 0.000, 0.000,
            -0.000, -0.001, 0.001, 0.001, -0.001, 0.000, -0.000, 0.000, -0.000, 0.001, 0.000, -0.000, 0.000, -0.000, 0.000, 0.000, -0.000, -0.000, 0.000, -0.000, -0.000, -0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,
            -0.009, -0.014, 0.001, 0.010, -0.013, 0.000, -0.010, 0.010, 0.000, 0.010, 0.010, -0.000, 0.001, 0.000, 0.000, 0.001, -0.000, 0.001, 0.002, -0.000, 0.000, 0.003, 0.000, 0.001, 0.000, 0.000, 0.000, 0.000,
            0.012, 0.018, -0.001, -0.013, 0.018, -0.000, 0.013, -0.013, -0.000, -0.013, -0.013, 0.001, -0.001, 0.000, -0.001, 0.000, 0.001, -0.001, -0.003, 0.000, -0.001, -0.004, -0.000, -0.001, 0.000, 0.000, 0.000, 0.000,
            0.000, -0.000, 0.000, 0.000, -0.000, 0.001, 0.000, 0.000, -0.000, 0.000, 0.000, -0.000, -0.000, 0.000, -0.000, 0.000, 0.000, 0.000, -0.000, -0.000, -0.000, -0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,
            0.009, 0.014, -0.000, -0.010, 0.013, 0.000, 0.010, -0.010, -0.000, -0.010, -0.010, 0.000, -0.001, 0.000, -0.001, 0.000, -0.000, -0.001, -0.001, 0.000, -0.000, -0.003, -0.000, -0.001, 0.000, 0.000, 0.000, 0.000,
            -0.009, -0.013, 0.000, 0.010, -0.013, 0.000, -0.010, 0.009, 0.000, 0.010, 0.010, -0.000, 0.001, -0.000, 0.000, -0.000, 0.000, 0.001, 0.002, 0.000, 0.000, 0.003, 0.000, 0.001, 0.000, 0.000, 0.000, 0.000,
            -0.000, -0.000, -0.000, 0.000, -0.000, -0.000, -0.000, 0.000, 0.001, 0.000, 0.000, 0.000, 0.000, -0.000, 0.000, -0.000, 0.000, -0.000, 0.000, -0.000, 0.000, 0.000, -0.000, -0.000, 0.000, 0.000, 0.000, 0.000,
            -0.009, -0.014, 0.001, 0.010, -0.013, 0.000, -0.010, 0.010, 0.000, 0.010, 0.010, -0.000, 0.001, 0.000, 0.000, -0.000, -0.000, 0.001, 0.002, -0.000, 0.000, 0.003, 0.000, 0.001, 0.000, 0.000, 0.000, 0.000,
            -0.009, -0.014, 0.000, 0.010, -0.013, 0.000, -0.010, 0.010, 0.000, 0.010, 0.010, -0.000, 0.001, -0.000, 0.000, -0.000, 0.000, 0.001, 0.002, -0.000, 0.000, 0.003, 0.001, 0.001, 0.000, 0.000, 0.000, 0.000,
            0.000, 0.001, -0.000, -0.000, 0.001, -0.000, 0.000, -0.000, 0.000, -0.000, -0.000, 0.001, 0.000, -0.000, -0.000, -0.000, 0.000, 0.000, -0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,
            -0.000, -0.001, 0.000, 0.001, -0.001, -0.000, -0.001, 0.001, 0.000, 0.001, 0.001, 0.000, 1.708, 0.048, 0.784, 0.062, 0.042, 0.053, 0.077, 0.001, -0.061, 0.046, -0.019, -0.029, 0.000, 0.000, 0.000, 0.000,
            0.000, 0.001, -0.000, 0.000, 0.000, 0.000, 0.000, -0.000, -0.000, 0.000, -0.000, -0.000, 0.048, 5.001, -1.631, -0.036, 0.144, 0.040, 0.036, 0.016, -0.051, -0.067, -0.024, -0.005, 0.000, 0.000, 0.000, 0.000,
            -0.000, -0.001, 0.000, 0.000, -0.001, -0.000, -0.001, 0.000, 0.000, 0.000, 0.000, -0.000, 0.784, -1.631, 1.242, 0.057, -0.037, 0.018, 0.034, -0.017, -0.015, 0.058, -0.021, -0.029, 0.000, 0.000, 0.000, 0.000,
            0.000, 0.000, 0.000, 0.001, 0.000, 0.000, 0.000, -0.000, -0.000, -0.000, -0.000, -0.000, 0.062, -0.036, 0.057, 6.228, -0.014, 0.932, 0.059, 0.053, -0.069, 0.148, 0.015, -0.031, 0.000, 0.000, 0.000, 0.000,
            -0.000, 0.000, -0.000, -0.000, 0.001, 0.000, -0.000, 0.000, 0.000, -0.000, 0.000, 0.000, 0.042, 0.144, -0.037, -0.014, 3.011, 0.986, 0.076, 0.030, -0.052, -0.027, 0.057, 0.051, 0.000, 0.000, 0.000, 0.000,
            -0.001, -0.001, -0.000, 0.001, -0.001, 0.000, -0.001, 0.001, -0.000, 0.001, 0.001, 0.000, 0.053, 0.040, 0.018, 0.932, 0.986, 0.885, 0.090, 0.044, -0.055, 0.057, 0.051, -0.003, 0.000, 0.000, 0.000, 0.000,
            -0.002, -0.003, 0.000, 0.002, -0.003, -0.000, -0.001, 0.002, 0.000, 0.002, 0.002, -0.000, 0.077, 0.036, 0.034, 0.059, 0.076, 0.090, 6.230, 0.139, 0.763, 0.013, -0.019, -0.024, 0.000, 0.000, 0.000, 0.000,
            0.000, 0.000, -0.000, -0.000, 0.000, -0.000, 0.000, 0.000, -0.000, -0.000, -0.000, 0.000, 0.001, 0.016, -0.017, 0.053, 0.030, 0.044, 0.139, 3.130, -1.128, -0.010, 0.131, 0.018, 0.000, 0.000, 0.000, 0.000,
            -0.000, -0.001, -0.000, 0.000, -0.001, -0.000, -0.000, 0.000, 0.000, 0.000, 0.000, 0.000, -0.061, -0.051, -0.015, -0.069, -0.052, -0.055, 0.763, -1.128, 0.866, -0.022, -0.053, 0.007, 0.000, 0.000, 0.000, 0.000,
            -0.003, -0.004, -0.000, 0.003, -0.004, -0.000, -0.003, 0.003, 0.000, 0.003, 0.003, 0.000, 0.046, -0.067, 0.058, 0.148, -0.027, 0.057, 0.013, -0.010, -0.022, 2.437, -0.102, 0.938, 0.000, 0.000, 0.000, 0.000,
            -0.000, -0.000, 0.000, 0.000, -0.000, 0.000, -0.000, 0.000, -0.000, 0.000, 0.001, 0.000, -0.019, -0.024, -0.021, 0.015, 0.057, 0.051, -0.019, 0.131, -0.053, -0.102, 4.944, 1.724, 0.000, 0.000, 0.000, 0.000,
            -0.001, -0.001, 0.000, 0.001, -0.001, 0.000, -0.001, 0.001, -0.000, 0.001, 0.001, 0.000, -0.029, -0.005, -0.029, -0.031, 0.051, -0.003, -0.024, 0.018, 0.007, 0.938, 1.724, 1.569, 0.000, 0.000, 0.000, 0.000,
            0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 1.0, 0.000, 0.000, 0.000,
            0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 1.0, 0.000, 0.000,
            0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 1.0, 0.000,
            0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 1.0;
        // ��������Э����
        for (int i(0); i < _Qdig.rows(); ++i)
        {
            if (i < 3)
            {
                _Qdig(i) = 0.0003;
            }
            else if (i < 6)
            {
                _Qdig(i) = 0.0003;
            }
            else
            {
                _Qdig(i) = 0.01;
            }
        }

        _Cu << 268.573, -43.819, -147.211,
            -43.819, 92.949, 58.082,
            -147.211, 58.082, 302.120;
        // ����Э����
        _QInit = _Qdig.asDiagonal();
        _QInit += _B * _Cu * _B.transpose();

        estimator.setConv(_QInit, _RInit, _largeVariance * _P);// ��ʼ��һ���Ƚϴ��Ԥ��Э�������
    }

    void Body::initParams(Vector3d _leg2body, Vector3d _initRbLegXYPosition, Vector<double, 9> _M, Vector<double, 9> _Ib, Vector3d _Pg)
    {
        this->leg2body = _leg2body;
        this->initRbLegXYPosition = _initRbLegXYPosition;
      /*  this->M = Eigen::Map<Matrix3d>(_M.data(),3,3);
        this->Ib = Eigen::Map<Matrix3d>(_Ib.data(),3,3);*/
        this->M(0, 0) = _M(0, 0);
        this->M(1, 1) = _M(4, 0);
        this->M(2, 2) = _M(8, 0);
        this->Ib(0, 0) = _Ib(0, 0);
        this->Ib(1, 1) = _Ib(4, 0);
        this->Ib(2, 2) = _Ib(8, 0);
        this->Pg = _Pg;
    }

    Matrix3d Body::v3_to_m3(Vector3d _v)
    {
        Matrix3d m = Matrix3d::Zero();
        m(0, 1) = -_v(2);
        m(0, 2) = _v(1);
        m(1, 0) = _v(2);
        m(1, 2) = -_v(0);
        m(2, 0) = -_v(1);
        m(2, 1) = _v(0);
        return m;
    }

    void Body::calTbs(int8_t direction)
    {
        if (direction == 1)
        {
            // ��ǰ
            // ����ŷ������ת����
            //Eigen::AngleAxisd rotationx_c(currentBodyState.Ang_xyz(0), Eigen::Vector3d::UnitX());
            //Eigen::AngleAxisd rotationy_c(currentBodyState.Ang_xyz(1), Eigen::Vector3d::UnitY());
            //Eigen::AngleAxisd rotationz_c(currentBodyState.Ang_xyz(2), Eigen::Vector3d::UnitZ());
            //Eigen::Matrix3d rotation_c = (rotationx_c * rotationy_c * rotationz_c).toRotationMatrix();
            //// ������α任����
            //Rsb_c = rotation_c;
            //Tsb_c.block<3, 3>(0, 0) = rotation_c;
            //Tsb_c.block<3, 1>(0, 3) = currentWorldState.dist;
            // ʹ����Ԫ���õ��任����
            Eigen::Matrix3d rotation_c = quatToRot(currentBodyState.Quat);
            Rsb_c = rotation_c;
            currentBodyState.Ang_xyz = rotMatToRPY(rotation_c);
            Tsb_c.block<3, 3>(0, 0) = rotation_c;
            Tsb_c.block<3, 1>(0, 3) = currentWorldState.dist;
            // ����������������ϵ�µı���
            currentWorldState.angVel_xyz = rotation_c * currentBodyState.angVel_xyz;
            //currentWorldState.linVel_xyz = rotation_c * currentBodyState.linVel_xyz;
            // currentWorldState.angAcc_xyz = rotation_c * currentBodyState.angAcc_xyz;
            currentWorldState.linAcc_xyz = rotation_c * currentBodyState.linAcc_xyz;
        }
        else if (direction == -1)
        {
            // Ŀ��
            // ����ŷ������ת����
            Eigen::AngleAxisd rotationx_t(targetBodyState.Ang_xyz(0), Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd rotationy_t(targetBodyState.Ang_xyz(1), Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd rotationz_t(targetBodyState.Ang_xyz(2), Eigen::Vector3d::UnitZ());
            Eigen::Matrix3d rotation_t = (rotationx_t * rotationy_t * rotationz_t).toRotationMatrix();
            // ������α任����
            Rsb_t = rotation_t;
            Tsb_t.block<3, 3>(0, 0) = rotation_t;
            Tsb_t.block<3, 1>(0, 3) = targetWorldState.dist;
            // ����������������ϵ�µı���
            // targetBodyState.angVel_xyz = rotation_t.inverse() * targetWorldState.angVel_xyz;
            /*targetBodyState.linVel_xyz = rotation_t.inverse() * targetWorldState.linVel_xyz;*/
            // targetBodyState.angAcc_xyz = rotation_t.inverse() * targetWorldState.angAcc_xyz;
            /*targetBodyState.linAcc_xyz = rotation_t.inverse() * targetWorldState.linAcc_xyz;*/
        }
    }

    void Body::legAndBodyPosition(int8_t direction)
    {
        if (direction == 1)
        {
            currentBodyState.leg_b[LF].Position = legs[LF]->currentLeg.Position + this->leg2body;
            Vector3d tmp = this->leg2body;
            tmp(1) = tmp(1) * -1;
            currentBodyState.leg_b[RF].Position = legs[RF]->currentLeg.Position + tmp;
            tmp(0) = tmp(0) * -1;
            currentBodyState.leg_b[RB].Position = legs[RB]->currentLeg.Position + tmp;
            tmp(1) = tmp(1) * -1;
            currentBodyState.leg_b[LB].Position = legs[LB]->currentLeg.Position + tmp;
        }
        else if (direction == -1)
        {
            legs[LF]->targetLeg.Position = targetBodyState.leg_b[LF].Position - this->leg2body;
            Vector3d tmp = this->leg2body;
            tmp(1) = tmp(1) * -1;
            legs[RF]->targetLeg.Position = targetBodyState.leg_b[RF].Position - tmp;
            tmp(0) = tmp(0) * -1;
            legs[RB]->targetLeg.Position = targetBodyState.leg_b[RB].Position - tmp;
            tmp(1) = tmp(1) * -1;
            legs[LB]->targetLeg.Position = targetBodyState.leg_b[LB].Position - tmp;
        }
    }

    void Body::bodyAndWorldFramePosition(int8_t direction)
    {
        for (int i = 0; i < 4; i++)
        {
            if (direction == 1)
            {
                Vector4d Pbi(0, 0, 0, 1);
                Pbi.block<3, 1>(0, 0) = currentBodyState.leg_b[i].Position;
                Vector4d Psi = this->Tsb_c * Pbi;
                currentWorldState.leg_s[i].Position = Psi.block<3, 1>(0, 0);
            }
            else if (direction == -1)
            {
                Vector4d Psi(0, 0, 0, 1);
                Psi.block<3, 1>(0, 0) = targetWorldState.leg_s[i].Position;
                Vector4d Pbi = this->Tsb_t.inverse() * Psi;
                targetBodyState.leg_b[i].Position = Pbi.block<3, 1>(0, 0);
            }
        }
    }

    void Body::legVelocityInWorldFrame()
    {
        for (int i = 0; i < 4; i++)
        {
            // ��������ת���ٶ�����ת���ɷ��Գƾ���
            Matrix3d w = v3_to_m3(currentBodyState.angVel_xyz);
            // �����������ڻ�������ϵ���ٶ�
            currentBodyState.leg_b[i].Velocity = legs[i]->currentLeg.Velocity;
            // ��������������������ϵ���ٶ�
            currentWorldState.leg_s[i].Velocity = Rsb_c * (w * currentBodyState.leg_b[i].Position + legs[i]->currentLeg.Velocity);
        }
    }

    void Body::updateDynamic()
    {
        for (int i = 0; i < 4; i++)
        {
            //dynamicLeft.block<3, 3>(0, i * 3) = Rsb_c;
            Vector3d Pgi = Vector3d::Zero();
            Pgi = Rsb_c * (currentBodyState.leg_b[i].Position - Pg);
            //Pgi = (currentBodyState.leg_b[i].Position - Pg);
            dynamicLeft.block<3, 3>(3, i * 3) = v3_to_m3(Pgi);
        }
        dynamicRight.block<3, 3>(0, 0) = M;
        dynamicRight.block<3, 3>(3, 3) = Rsb_c * Ib * Rsb_c.transpose();
        //dynamicRight.block<3, 3>(3, 3) = Ib;
    }

    void Body::updateBodyTargetPos(Vector3d _angle, Vector3d _position)
    {
        targetBodyState.Ang_xyz = _angle;
        targetWorldState.dist = _position;
    }

    void Body::updateBodyImu(Vector3d _imuRPY)
    {
        currentBodyState.Ang_xyz = _imuRPY;
    }

    void Body::updateBodyImu(Vector4d _imuQ)
    {
        currentBodyState.Quat = _imuQ;
    }

    void Body::updateBodyGyro(Vector3d _gyro)
    {
        currentBodyState.angVel_xyz = _gyro;
    }

    void Body::updateBodyAcc(Vector3d _acc)
    {
        currentBodyState.linAcc_xyz = _acc;
    }

    void Body::updateTargetFootPoint(Eigen::Matrix<double, 3, 4> _point)
    {
        for (int i = 0; i < 4; i++)
        {
            targetWorldState.leg_s[i].Position = _point.col(i);
            Vector4d Psi(0, 0, 0, 1);
            Psi.block<3, 1>(0, 0) = targetWorldState.leg_s[i].Position;
            Vector4d Pbi = this->Tsb_c.inverse() * Psi;
            targetBodyState.leg_b[i].Position = Pbi.block<3, 1>(0, 0);
        }
    }

    void Body::updateTargetFootVel(Eigen::Matrix<double, 3, 4> _vel)
    {
        for (int i = 0; i < 4; i++)
        {
            //�˴���ָ�������ڻ�����ٶ�����������ϵ�еı������Ҫ��ȥ�����ٶ�
            targetWorldState.leg_s[i].Velocity = _vel.col(i) - estimatorState.block<3,1>(3,0);
            targetBodyState.leg_b[i].Velocity = Rsb_c.transpose() * _vel.col(i);
            legs[i]->setTargetLegVelocity(targetBodyState.leg_b[i].Velocity);
        }
    }

    void Body::estimatorRun(Vector4i _contact, Vector4d _phase)
    {
        IOFormat CommaInitFmt(StreamPrecision, DontAlignCols, ", ", ", ", "", "", " << ", ";");

        // �����������
        Eigen::Matrix<double, 3, 1> estimatorInput = this->currentWorldState.linAcc_xyz + this->g;
        // ���²ο�״ֵ̬,����Э���������д���
        Eigen::Matrix<double, 28, 1> y;
        _R = _RInit;
        _Q = _QInit;
        for (int i = 0; i < 4; i++)
        {
            if (_contact(i) == 0)
            {
                _Q.block(6 + 3 * i, 6 + 3 * i, 3, 3) = _largeVariance * Eigen::Matrix<double, 3, 3>::Identity();
                _R.block(12 + 3 * i, 12 + 3 * i, 3, 3) = _largeVariance * Eigen::Matrix<double, 3, 3>::Identity();
                _R(24 + i, 24 + i) = _largeVariance;
            }
            else
            {
                _trust = windowFunc(_phase(i), 0.2);
                _Q.block(6 + 3 * i, 6 + 3 * i, 3, 3) = (1 + (1 - _trust) * _largeVariance) * _QInit.block(6 + 3 * i, 6 + 3 * i, 3, 3);
                _R.block(12 + 3 * i, 12 + 3 * i, 3, 3) = (1 + (1 - _trust) * _largeVariance) * _RInit.block(12 + 3 * i, 12 + 3 * i, 3, 3);
                _R(24 + i, 24 + i) = (1 + (1 - _trust) * _largeVariance) * _RInit(24 + i, 24 + i);
            }
            y.block<3, 1>(i * 3, 0) = this->Rsb_c * currentBodyState.leg_b[i].Position;
            y.block<3, 1>(12 + i * 3, 0) = currentWorldState.leg_s[i].Velocity;
            y(24 + i, 0) = 0;
        }
        // ����Э�������
        estimator.updateConv(_Q, _R);
        // �������˲�ִ��
        estimator.f(estimatorInput, y);
        // �������
        estimatorOut = estimator.getOut();
        // ����״̬
        estimatorState = estimator.getState();
        // �õ���ǰ�����λ�ú��ٶ�
        currentWorldState.dist = estimatorState.block<3, 1>(0, 0);
        currentWorldState.linVel_xyz = estimatorState.block<3, 1>(3, 0);
    }

    Eigen::Matrix<double, 3, 4> Body::getEstFeetPos()
    {
        Eigen::Matrix<double, 3, 4> out;
        for (int i = 0; i < 4; i++)
        {
            out.block<3, 1>(0, i) = estimatorState.block<3, 1>(6 + 3 * i, 0);
        }
        return out;
    }

    Eigen::Vector3d Body::getEstFeetPos(int id)
    {
        Eigen::Vector3d out;
        out = estimatorState.block<3, 1>(6 + 3 * id, 0);
        return out;
    }

    Eigen::Matrix<double, 3, 4> Body::getEstFeetVel()
    {
        Eigen::Matrix<double, 3, 4> out;
        for (int i = 0; i < 4; i++)
        {
            out.block<3, 1>(0, i) = estimatorOut.block<3, 1>(12 + 3 * i, 0);
        }
        return out;
    }

    Eigen::Vector3d Body::getEstFeetVel(int id)
    {
        Eigen::Vector3d out;
        out = estimatorOut.block<3, 1>(12 + 3 * id, 0);
        return out;
    }

    Eigen::Matrix<double, 3, 4> Body::getFKFeetPos()
    {
        Eigen::Matrix<double, 3, 4> out;
        for (int i = 0; i < 4; i++)
        {
            out.block<3, 1>(0, i) = currentWorldState.leg_s[i].Position;
        }
        return out;
    }

    Eigen::Vector3d Body::getFKFeetPos(int id)
    {
        Eigen::Vector3d out;
        out = currentWorldState.leg_s[id].Position;
        return out;
    }

    Eigen::Matrix<double, 3, 4> Body::getFKFeetVel()
    {
        Eigen::Matrix<double, 3, 4> out;
        for (int i = 0; i < 4; i++)
        {
            out.block<3, 1>(0, i) = currentWorldState.leg_s[i].Velocity;
        }
        return out;
    }

    Eigen::Vector3d Body::getFKFeetVel(int id)
    {
        Eigen::Vector3d out;
        out = currentWorldState.leg_s[id].Velocity;
        return out;
    }

    Eigen::Matrix3d Body::quatToRot(const Vector4d& _quat)
    {
        double e0 = _quat(0);
        double e1 = _quat(1);
        double e2 = _quat(2);
        double e3 = _quat(3);

        Matrix3d R;
        R << 1 - 2 * (e2 * e2 + e3 * e3), 2 * (e1 * e2 - e0 * e3),
            2 * (e1 * e3 + e0 * e2), 2 * (e1 * e2 + e0 * e3),
            1 - 2 * (e1 * e1 + e3 * e3), 2 * (e2 * e3 - e0 * e1),
            2 * (e1 * e3 - e0 * e2), 2 * (e2 * e3 + e0 * e1),
            1 - 2 * (e1 * e1 + e2 * e2);
        return R;
    }

    Eigen::Vector3d Body::rotMatToRPY(const Matrix3d& R) {
        Vector3d rpy;
        rpy(0) = atan2(R(2, 1), R(2, 2));
        rpy(1) = asin(-R(2, 0));
        rpy(2) = atan2(R(1, 0), R(0, 0));
        return rpy;
    }
}
