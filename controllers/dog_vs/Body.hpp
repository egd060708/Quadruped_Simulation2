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

    // 世界坐标系下整机状态，坐标系表示为{s}
    typedef struct _worldFrame
    {
        LegS leg_s[4];
        Vector3d dist = Vector3d::Zero();       // 机身在世界坐标系下的位移
        Vector3d angVel_xyz = Vector3d::Zero(); // 角速度
        Vector3d linVel_xyz = Vector3d::Zero(); // 线速度
        // Vector3d angAcc_xyz = Vector3d::Zero(); // 角加速度
        Vector3d linAcc_xyz = Vector3d::Zero(); // 线性加速度
    } worldFrame;
    // 机器人坐标系下的状态，坐标系表示为{b}
    typedef struct _bodyFrame
    {
        LegS leg_b[4];
        Vector3d Ang_xyz = Vector3d::Zero();    // 角度
        Vector4d Quat = Vector4d::Zero();       //四元数
        Vector3d angVel_xyz = Vector3d::Zero(); // 角速度
        Vector3d linVel_xyz = Vector3d::Zero(); // 线速度
        //Vector3d angAcc_xyz = Vector3d::Zero(); // 角加速度
        Vector3d linAcc_xyz = Vector3d::Zero(); // 线性加速度
    } bodyFrame;

    class Body
    {
    public:
        worldFrame targetWorldState;
        worldFrame currentWorldState;
        bodyFrame targetBodyState;
        bodyFrame currentBodyState;

        Leg* legs[4];
        Vector3d leg2body;                     // 初始值代表左前腿向量
        Matrix3d Rsb_c = Matrix3d::Identity(); // (当前)世界坐标系到机身坐标系的旋转矩阵映射
        Matrix4d Tsb_c = Matrix4d::Identity(); // (当前)世界坐标系到机身坐标系的齐次变换映射
        Matrix3d Rsb_t = Matrix3d::Identity(); // (目标)世界坐标系到机身坐标系的旋转矩阵映射
        Matrix4d Tsb_t = Matrix4d::Identity(); // (目标)世界坐标系到机身坐标系的齐次变换映射

        Matrix3d M = Matrix3d::Zero();      // 机器人机体质量
        Matrix3d Ib = Matrix3d::Zero();     // 机器人单刚体动力学机身转动惯量
        Vector3d Pg = Vector3d::Zero();     // 机器人重心在机身坐标系下的位置（相当于{PbPg}）
        Vector3d g = Vector3d(0, 0, -9.81); // 直接初始化重力加速度向量
        double dt;                           // 控制周期
        double _largeVariance = 100;              // 大的协方差
        double _trust;                      // 对于腿部是否触地的置信度

        Eigen::Matrix<double, 6, 12> dynamicLeft = Eigen::Matrix<double, 6, 12>::Zero();
        Eigen::Matrix<double, 6, 6> dynamicRight = Eigen::Matrix<double, 6, 6>::Zero();

        Vector3d initRbLegXYPosition; // 指定右后腿初始（平面）位置

        // 将向量转换成角对称矩阵
        Matrix3d v3_to_m3(Vector3d _v);

        // 引入状态估计器
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
        // 构造函数，绑定四个腿部状态描述类型
        Body(Leg* _legObj[4], double _dt);
        // 初始化函数，用于初始化物理参数
        void initParams(Vector3d _leg2body, Vector3d _initRbLegXYPosition, Vector<double,9> _M, Vector<double,9> _Ib, Vector3d _Pg);
        // 计算齐次变换矩阵(direction为1，则是当前；为-1，则是目标)
        void calTbs(int8_t direction);
        // 四足运动学：改变四条腿足端的位置从而改变机器人机身的位置和姿态
        // 计算单腿基坐标系与机身坐标系下的足端位置转换(direction为1，则是(当前)腿->身；为-1，则是(目标)身->腿)
        void legAndBodyPosition(int8_t direction);
        // 计算机身坐标系与世界坐标系下的足端位置转换(世界坐标系定义为初始状态下右后腿足端位置)（direction为1，则是(当前)身->世；为-1，则是(目标)世->身）
        void bodyAndWorldFramePosition(int8_t direction);
        // 计算足端在世界坐标系中的足端速度
        void legVelocityInWorldFrame();
        // 更新机器人动力学方程（描述为 left*[f] = right）
        void updateDynamic();

        // 更新目标位姿
        void updateBodyTargetPos(Vector3d _angle, Vector3d _position);
        // 更新惯导姿态
        void updateBodyImu(Vector3d _imuRPY);
        void updateBodyImu(Vector4d _imyQ);
        // 更新陀螺仪角速度
        void updateBodyGyro(Vector3d _gyro);
        // 更新加速度计加速度
        void updateBodyAcc(Vector3d _acc);
        // 更新目标四足点(地面)
        void updateTargetFootPoint(Eigen::Matrix<double,3,4> _point);
        // 更新目标四足速度
        void updateTargetFootVel(Eigen::Matrix<double, 3, 4> _vel);

        // 状态估计运行
        void estimatorRun(Vector4i _contact, Vector4d _phase);
        // 获取估计四足点位
        Eigen::Matrix<double, 3, 4> getEstFeetPos();
        Eigen::Vector3d getEstFeetPos(int id);
        Eigen::Matrix<double, 3, 4> getEstFeetVel();
        Eigen::Vector3d getEstFeetVel(int id);
        Eigen::Matrix<double, 3, 4> getFKFeetPos();
        Eigen::Vector3d getFKFeetPos(int id);
        Eigen::Matrix<double, 3, 4> getFKFeetVel();
        Eigen::Vector3d getFKFeetVel(int i);

        // 一些数学计算函数
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

        Matrix3d Ident = Matrix3d::Identity(); // 初始化一个3维对角矩阵
        dynamicLeft.block<3, 3>(0, 0) = Ident;
        dynamicLeft.block<3, 3>(0, 3) = Ident;
        dynamicLeft.block<3, 3>(0, 6) = Ident;
        dynamicLeft.block<3, 3>(0, 9) = Ident;

        // 初始化状态估计器
        // 状态转移矩阵
        _A.block(0, 3, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
        // 输入矩阵
        _B.block(3, 0, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
        // 输出矩阵
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
        // 初始化状态空间方程
        estimator.setFunc(_A, _B, _H, dt);
        // 测量噪声协方差
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
        // 过程噪声协方差
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
        // 过程协方差
        _QInit = _Qdig.asDiagonal();
        _QInit += _B * _Cu * _B.transpose();

        estimator.setConv(_QInit, _RInit, _largeVariance * _P);// 初始化一个比较大的预测协方差矩阵
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
            // 当前
            // 三轴欧拉角旋转矩阵
            //Eigen::AngleAxisd rotationx_c(currentBodyState.Ang_xyz(0), Eigen::Vector3d::UnitX());
            //Eigen::AngleAxisd rotationy_c(currentBodyState.Ang_xyz(1), Eigen::Vector3d::UnitY());
            //Eigen::AngleAxisd rotationz_c(currentBodyState.Ang_xyz(2), Eigen::Vector3d::UnitZ());
            //Eigen::Matrix3d rotation_c = (rotationx_c * rotationy_c * rotationz_c).toRotationMatrix();
            //// 构建齐次变换矩阵
            //Rsb_c = rotation_c;
            //Tsb_c.block<3, 3>(0, 0) = rotation_c;
            //Tsb_c.block<3, 1>(0, 3) = currentWorldState.dist;
            // 使用四元数得到变换矩阵
            Eigen::Matrix3d rotation_c = quatToRot(currentBodyState.Quat);
            Rsb_c = rotation_c;
            currentBodyState.Ang_xyz = rotMatToRPY(rotation_c);
            Tsb_c.block<3, 3>(0, 0) = rotation_c;
            Tsb_c.block<3, 1>(0, 3) = currentWorldState.dist;
            // 更新其他世界坐标系下的变量
            currentWorldState.angVel_xyz = rotation_c * currentBodyState.angVel_xyz;
            //currentWorldState.linVel_xyz = rotation_c * currentBodyState.linVel_xyz;
            // currentWorldState.angAcc_xyz = rotation_c * currentBodyState.angAcc_xyz;
            currentWorldState.linAcc_xyz = rotation_c * currentBodyState.linAcc_xyz;
        }
        else if (direction == -1)
        {
            // 目标
            // 三轴欧拉角旋转矩阵
            Eigen::AngleAxisd rotationx_t(targetBodyState.Ang_xyz(0), Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd rotationy_t(targetBodyState.Ang_xyz(1), Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd rotationz_t(targetBodyState.Ang_xyz(2), Eigen::Vector3d::UnitZ());
            Eigen::Matrix3d rotation_t = (rotationx_t * rotationy_t * rotationz_t).toRotationMatrix();
            // 构建齐次变换矩阵
            Rsb_t = rotation_t;
            Tsb_t.block<3, 3>(0, 0) = rotation_t;
            Tsb_t.block<3, 1>(0, 3) = targetWorldState.dist;
            // 更新其他世界坐标系下的变量
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
            // 将机身旋转角速度向量转换成反对称矩阵
            Matrix3d w = v3_to_m3(currentBodyState.angVel_xyz);
            // 更新足端相对于机身坐标系的速度
            currentBodyState.leg_b[i].Velocity = legs[i]->currentLeg.Velocity;
            // 计算足端相对于世界坐标系的速度
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
            //此处是指足端相对于机身的速度在世界坐标系中的表达，因此需要减去整机速度
            targetWorldState.leg_s[i].Velocity = _vel.col(i) - estimatorState.block<3,1>(3,0);
            targetBodyState.leg_b[i].Velocity = Rsb_c.transpose() * _vel.col(i);
            legs[i]->setTargetLegVelocity(targetBodyState.leg_b[i].Velocity);
        }
    }

    void Body::estimatorRun(Vector4i _contact, Vector4d _phase)
    {
        IOFormat CommaInitFmt(StreamPrecision, DontAlignCols, ", ", ", ", "", "", " << ", ";");

        // 更新输入变量
        Eigen::Matrix<double, 3, 1> estimatorInput = this->currentWorldState.linAcc_xyz + this->g;
        // 更新参考状态值,并对协方差矩阵进行处理
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
        // 更新协方差矩阵
        estimator.updateConv(_Q, _R);
        // 卡尔曼滤波执行
        estimator.f(estimatorInput, y);
        // 估计输出
        estimatorOut = estimator.getOut();
        // 估计状态
        estimatorState = estimator.getState();
        // 得到当前机身的位置和速度
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
