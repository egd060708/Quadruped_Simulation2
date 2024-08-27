#pragma once
#include "Leg.h"
#include "kalmanFilter.h"

namespace Quadruped
{
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
        Vector3f dist = Vector3f::Zero();       // 机身在世界坐标系下的位移
        // Vector3f angVel_xyz = Vector3f::Zero(); // 角速度
        Vector3f linVel_xyz = Vector3f::Zero(); // 线速度
        // Vector3f angAcc_xyz = Vector3f::Zero(); // 角加速度
        Vector3f linAcc_xyz = Vector3f::Zero(); // 线性加速度
    } worldFrame;
    // 机器人坐标系下的状态，坐标系表示为{b}
    typedef struct _bodyFrame
    {
        LegS leg_b[4];
        Vector3f Ang_xyz = Vector3f::Zero();    // 角度
        Vector3f angVel_xyz = Vector3f::Zero(); // 角速度
        Vector3f linVel_xyz = Vector3f::Zero(); // 线速度
        Vector3f angAcc_xyz = Vector3f::Zero(); // 角加速度
        Vector3f linAcc_xyz = Vector3f::Zero(); // 线性加速度
    } bodyFrame;

    class Body
    {
    public:
        worldFrame targetWorldState;
        worldFrame currentWorldState;
        bodyFrame targetBodyState;
        bodyFrame currentBodyState;

        Leg *legs[4];
        Vector3f leg2body;                     // 初始值代表左前腿向量
        Matrix3f Rsb_c = Matrix3f::Identity(); // (当前)世界坐标系到机身坐标系的旋转矩阵映射
        Matrix4f Tsb_c = Matrix4f::Identity(); // (当前)世界坐标系到机身坐标系的齐次变换映射
        Matrix3f Rsb_t = Matrix3f::Identity(); // (目标)世界坐标系到机身坐标系的旋转矩阵映射
        Matrix4f Tsb_t = Matrix4f::Identity(); // (目标)世界坐标系到机身坐标系的齐次变换映射

        Matrix3f M = Matrix3f::Zero();      // 机器人机体质量
        Matrix3f Ib = Matrix3f::Zero();     // 机器人单刚体动力学机身转动惯量
        Vector3f Pg = Vector3f::Zero();     // 机器人重心在机身坐标系下的位置（相当于{PbPg}）
        Vector3f g = Vector3f(0, 0, -9.81); // 直接初始化重力加速度向量
        float dt;                           // 控制周期

        Eigen::Matrix<float, 6, 12> dynamicLeft = Eigen::Matrix<float, 6, 12>::Zero();
        Eigen::Matrix<float, 6, 6> dynamicRight = Eigen::Matrix<float, 6, 6>::Zero();

        Vector3f initRbLegXYPosition; // 指定右后腿初始（平面）位置

        // 将向量转换成角对称矩阵
        Matrix3f v3_to_m3(Vector3f _v);

        // 引入状态估计器
        kalmanFilter<18, 3, 28> estimator;
        Eigen::Matrix<float, 28, 18> _H = Eigen::Matrix<float, 28, 18>::Zero();
        Eigen::Matrix<float, 18, 18> _A = Eigen::Matrix<float, 18, 18>::Zero();
        Eigen::Matrix<float, 18, 3> _B = Eigen::Matrix<float, 18, 3>::Zero();
        Eigen::Matrix<float, 28, 28> _R = Eigen::Matrix<float, 28, 28>::Zero();
        Eigen::Matrix<float, 18, 18> _Q = Eigen::Matrix<float, 18, 18>::Zero();
        Eigen::Vector<float, 18> _Qdig = Eigen::Vector<float, 18>::Zero();
        Eigen::Matrix<float, 3, 3> _Cu = Eigen::Matrix<float, 3, 3>::Zero();
        Eigen::Matrix<float, 18, 18> _P = Eigen::Matrix<float, 18, 18>::Identity();

        Eigen::Vector<float, 28> estimatorOut = Eigen::Vector<float, 28>::Zero();

    public:
        // 构造函数，绑定四个腿部状态描述类型
        Body(Leg *lf_leg, Leg *rf_leg, Leg *lb_leg, Leg *rb_leg, float _dt);
        // 初始化函数，用于初始化物理参数
        void initParams(Vector3f _leg2body, Vector3f _initRbLegXYPosition, Matrix3f _M, Matrix3f _Ib, Vector3f _Pg);
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
        void updateBodyTargetPos(Vector3f _angle, Vector3f _position);
        // 更新惯导姿态
        void updateBodyImu(Vector3f _imu);
        // 更新陀螺仪角速度
        void updateBodyGyro(Vector3f _gyro);
        // 更新加速度计加速度
        void updateBodyAcc(Vector3f _acc);
        // 更新目标四足点(地面)
        void updateTargetFootPoint(Vector3f _point[4]);

        // 状态估计运行
        void estimatorRun();
    };

    Body::Body(Leg *lf_leg, Leg *rf_leg, Leg *lb_leg, Leg *rb_leg, float _dt)
    {
        this->legs[LF] = lf_leg;
        this->legs[RF] = rf_leg;
        this->legs[LB] = lb_leg;
        this->legs[RB] = rb_leg;
        this->dt = _dt;

        Matrix3f Ident = Matrix3f::Identity(); // 初始化一个3维对角矩阵
        dynamicLeft.block<3, 3>(0, 0) = Ident;
        dynamicLeft.block<3, 3>(0, 3) = Ident;
        dynamicLeft.block<3, 3>(0, 6) = Ident;
        dynamicLeft.block<3, 3>(0, 9) = Ident;

        // 初始化状态估计器
        // 状态转移矩阵
        _A.block(0, 3, 3, 3) = Eigen::Matrix<float, 3, 3>::Identity();
        // 输入矩阵
        _B.block(3, 0, 3, 3) = Eigen::Matrix<float, 3, 3>::Identity();
        // 输出矩阵
        _H.block(0, 0, 3, 3) = -Eigen::Matrix<float, 3, 3>::Identity();
        _H.block(3, 0, 3, 3) = -Eigen::Matrix<float, 3, 3>::Identity();
        _H.block(6, 0, 3, 3) = -Eigen::Matrix<float, 3, 3>::Identity();
        _H.block(9, 0, 3, 3) = -Eigen::Matrix<float, 3, 3>::Identity();
        _H.block(12, 3, 3, 3) = -Eigen::Matrix<float, 3, 3>::Identity();
        _H.block(15, 3, 3, 3) = -Eigen::Matrix<float, 3, 3>::Identity();
        _H.block(18, 3, 3, 3) = -Eigen::Matrix<float, 3, 3>::Identity();
        _H.block(21, 3, 3, 3) = -Eigen::Matrix<float, 3, 3>::Identity();
        _H.block(0, 6, 12, 12) = Eigen::Matrix<float, 12, 12>::Identity();
        _H(24, 8) = 1;
        _H(25, 11) = 1;
        _H(26, 14) = 1;
        _H(27, 17) = 1;
        // 初始化状态空间方程
        estimator.setFunc(_A, _B, _H, dt);
        // 测量噪声协方差
        _R << 0.008, 0.012, -0.000, -0.009, 0.012, 0.000, 0.009, -0.009, -0.000, -0.009, -0.009, 0.000, -0.000, 0.000, -0.000, 0.000, -0.000, -0.001, -0.002, 0.000, -0.000, -0.003, -0.000, -0.001, 0.000, 0.000, 0.000, 0.000,
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
        _Q = _Qdig.asDiagonal();
        _Q += _B * _Cu * _B.transpose();

        estimator.setConv(_Q, _R, 100 * _P);// 初始化一个比较大的预测协方差矩阵
    }

    void Body::initParams(Vector3f _leg2body, Vector3f _initRbLegXYPosition, Matrix3f _M, Matrix3f _Ib, Vector3f _Pg)
    {
        this->leg2body = _leg2body;
        this->initRbLegXYPosition = _initRbLegXYPosition;
        this->M = _M;
        this->Ib = _Ib;
        this->Pg = _Pg;
    }

    Matrix3f Body::v3_to_m3(Vector3f _v)
    {
        Matrix3f m = Matrix3f::Zero();
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
            Eigen::AngleAxisf rotationx_c(currentBodyState.Ang_xyz(0), Eigen::Vector3f::UnitX());
            Eigen::AngleAxisf rotationy_c(currentBodyState.Ang_xyz(1), Eigen::Vector3f::UnitY());
            Eigen::AngleAxisf rotationz_c(currentBodyState.Ang_xyz(2), Eigen::Vector3f::UnitZ());
            Eigen::Matrix3f rotation_c = (rotationx_c * rotationy_c * rotationz_c).toRotationMatrix();
            // 构建齐次变换矩阵
            Rsb_c = rotation_c;
            Tsb_c.block<3, 3>(0, 0) = rotation_c;
            Tsb_c.block<3, 1>(0, 3) = currentWorldState.dist - initRbLegXYPosition;
            // 更新其他世界坐标系下的变量
            // currentWorldState.angVel_xyz = rotation_c * currentBodyState.angVel_xyz;
            currentWorldState.linVel_xyz = rotation_c * currentBodyState.linVel_xyz;
            // currentWorldState.angAcc_xyz = rotation_c * currentBodyState.angAcc_xyz;
            currentWorldState.linAcc_xyz = rotation_c * currentBodyState.linAcc_xyz;
        }
        else if (direction == -1)
        {
            // 目标
            // 三轴欧拉角旋转矩阵
            Eigen::AngleAxisf rotationx_t(targetBodyState.Ang_xyz(0), Eigen::Vector3f::UnitX());
            Eigen::AngleAxisf rotationy_t(targetBodyState.Ang_xyz(1), Eigen::Vector3f::UnitY());
            Eigen::AngleAxisf rotationz_t(targetBodyState.Ang_xyz(2), Eigen::Vector3f::UnitZ());
            Eigen::Matrix3f rotation_t = (rotationx_t * rotationy_t * rotationz_t).toRotationMatrix();
            // 构建齐次变换矩阵
            Rsb_t = rotation_t;
            Tsb_t.block<3, 3>(0, 0) = rotation_t;
            Tsb_t.block<3, 1>(0, 3) = targetWorldState.dist - initRbLegXYPosition;
            // 更新其他世界坐标系下的变量
            // targetBodyState.angVel_xyz = rotation_t.inverse() * targetWorldState.angVel_xyz;
            targetBodyState.linVel_xyz = rotation_t.inverse() * targetWorldState.linVel_xyz;
            // targetBodyState.angAcc_xyz = rotation_t.inverse() * targetWorldState.angAcc_xyz;
            targetBodyState.linAcc_xyz = rotation_t.inverse() * targetWorldState.linAcc_xyz;
        }
    }

    void Body::legAndBodyPosition(int8_t direction)
    {
        if (direction == 1)
        {
            currentBodyState.leg_b[LF].Position = legs[LF]->currentLeg.Position + this->leg2body;
            Vector3f tmp = this->leg2body;
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
            Vector3f tmp = this->leg2body;
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
                Vector4f Pbi(0, 0, 0, 1);
                Pbi.block<3, 1>(0, 0) = currentBodyState.leg_b[i].Position;
                Vector4f Psi = this->Tsb_c * Pbi;
                currentWorldState.leg_s[i].Position = Psi.block<3, 1>(0, 0);
            }
            else if (direction == -1)
            {
                Vector4f Psi(0, 0, 0, 1);
                Psi.block<3, 1>(0, 0) = targetWorldState.leg_s[i].Position;
                Vector4f Pbi = this->Tsb_t.inverse() * Psi;
                targetBodyState.leg_b[i].Position = Pbi.block<3, 1>(0, 0);
            }
        }
    }

    void Body::legVelocityInWorldFrame()
    {
        for (int i = 0; i < 4; i++)
        {
            // 将机身旋转角速度向量转换成反对成矩阵
            Matrix3f w = v3_to_m3(currentBodyState.angVel_xyz);
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
            Vector3f Pgi = Rsb_c * currentBodyState.leg_b[i].Position - Rsb_c * Pg;
            dynamicLeft.block<3, 3>(3, i * 3) = v3_to_m3(Pgi);
        }
        dynamicRight.block<3, 3>(0, 0) = M;
        dynamicRight.block<3, 3>(3, 3) = Rsb_c * Ib * Rsb_c.transpose();
    }

    void Body::updateBodyTargetPos(Vector3f _angle, Vector3f _position)
    {
        targetBodyState.Ang_xyz = _angle;
        targetWorldState.dist = _position;
    }

    void Body::updateBodyImu(Vector3f _imu)
    {
        currentBodyState.Ang_xyz = _imu;
    }

    void Body::updateBodyGyro(Vector3f _gyro)
    {
        currentBodyState.angVel_xyz = _gyro;
    }

    void Body::updateBodyAcc(Vector3f _acc)
    {
        currentBodyState.linAcc_xyz = _acc;
    }

    void Body::updateTargetFootPoint(Vector3f _point[4])
    {
        for (int i = 0; i < 4; i++)
        {
            targetWorldState.leg_s[i].Position = _point[i];
        }
    }

    void Body::estimatorRun()
    {
        IOFormat CommaInitFmt(StreamPrecision, DontAlignCols, ", ", ", ", "", "", " << ", ";");

        // 更新输入变量
        Eigen::Matrix<float, 3, 1> estimatorInput = this->currentWorldState.linAcc_xyz + this->g;
        // 更新参考状态值
        Eigen::Matrix<float, 28, 1> y;
        for (int i = 0; i < 4; i++)
        {
            y.block<3, 1>(i * 3, 0) = currentWorldState.leg_s[i].Position;
            y.block<3, 1>(12 + i * 3, 0) = currentWorldState.leg_s[i].Velocity;
            y(24 + i, 0) = 0;
        }
        // 卡尔曼滤波执行
        estimator.f(estimatorInput, y);
        // 估计状态
        estimatorOut = estimator.getOut();
    }
}
