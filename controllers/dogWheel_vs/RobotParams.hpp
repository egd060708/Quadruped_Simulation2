#pragma once
#include <iostream>
#include <stdint.h>
#include <Eigen/Dense>

namespace Quadruped {
	// 腿长参数
	const double L1 = 0.11973;
	const double L2 = 0.35;
	const double L3 = 0.35;
	const double L4 = 0.17;
	const double L5 = 0.055;
	// 腿部基坐标系到机身坐标系的转换向量，要转换到机身坐标系，直接加上此向量(此处向量仅代表左前腿)
	const Eigen::Vector3d leg2bodyFrame(0.3285, 0.072, 0);
	// 初始位型右后腿位置
	const Eigen::Vector3d initRbLegXYPosition(-0.3154553, -0.19173, 0);
	// 机身惯量
	const Eigen::Vector<double,6> Ib(2.746600e-01,1.061800e+00,1.182500e+00,-6.220000e-04,3.150000e-03,-1.390000e-03);
	// 机身质量
	const double Mb = 35.606;
	// 机器人重心在机身坐标系下的位置(仿真中认为重心就是机器人机体形心)
	const Eigen::Vector3d Pb(0.000458, 0.005261, 0.000665);
	// 髋参数(左前腿为例)
	const Eigen::Vector<double, 6> Ihip(3.318800e-03,4.874300e-03,3.708700e-03,7.160000e-05,3.770000e-07,-4.000000e-09);
	const double Mhip = 2.673;
	const Eigen::Vector3d Phip(-0.003841, -0.009068, 0.000000);
	// 大腿参数(左前腿为例)
	const Eigen::Vector<double, 6> Ithigh(6.229900e-02,6.139900e-02,8.199700e-03,8.778100e-04,-3.647500e-03,8.353700e-03);
	const double Mthigh = 4.536;
	const Eigen::Vector3d Pthigh(-0.006279, -0.032049, -0.057835);
	// 小腿参数(左前腿为例)
	const Eigen::Vector<double, 6> Icalf(2.928300e-02,3.086800e-02,3.260000e-03,2.210000e-04,-1.064000e-03,2.093000e-03);
	const double Mcalf = 2.290600;
	const Eigen::Vector3d Pcalf(0.005278, 0.017301, -0.297780);
	// 足参数(左前腿为例)
	const Eigen::Vector<double, 6> Ifoot(4.121000e-03,7.776000e-03,4.108000e-03,6.000000e-06,-4.800000e-05,7.000000e-06);
	const double Mfoot = 1.083000;
	const Eigen::Vector3d Pfoot(-0.000991,0.051053,-0.001130);
	// mpc平衡控制器权重参数
	const Eigen::Vector<double, 6> Q(3000, 3000, 5000, 100, 200, 100);
	const Eigen::Vector<double, 6> F = Q;
	const Eigen::Vector<double, 12> R = Eigen::Vector<double, 12>::Constant(1);
	const Eigen::Vector<double, 12> W = Eigen::Vector<double, 12>::Constant(0.8);
	const Eigen::Vector<double, 9> linPD(30, -0.1, 10, 30, -0.1, 10, 50, -0.1, 10);
	const Eigen::Vector<double, 9> angPD(10, -0.1, 5, 20, -0.1, 5, 10, -0.1, 5);

	//// mpc约束
	//const Eigen::Vector<double, 12> lb = Eigen::Vector<double, 12>::Constant(-100);
	//const Eigen::Vector<double, 12> ub = Eigen::Vector<double, 12>::Constant(100);

	// 步态运动期望增益
	const Eigen::Vector3d gaitK(0.001, 0.001, 0.004);
}