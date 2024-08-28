#pragma once
#include <iostream>
#include <stdint.h>
#include <Eigen/Dense>

namespace Quadruped {

	const double L1 = 0.0838f;
	const double L2 = 0.2f;
	const double L3 = 0.2f;
	// 腿部基坐标系到机身坐标系的转换向量，要转换到机身坐标系，直接加上此向量(此处向量仅代表左前腿)
	const Eigen::Vector3d leg2bodyFrame(0.1805, 0.047, 0);
	// 初始右后腿位置
	const Eigen::Vector3d initRbLegXYPosition(-0.1805, -0.1308, 0);
	// 机身惯量
	const Eigen::Vector<double,9> Ib(0.132, 0, 0, 0, 0.3475, 0, 0, 0, 0.3775);
	// 机身质量
	const Eigen::Vector<double, 9> M(13.4, 0, 0, 0, 13.4, 0, 0, 0, 13.4);
	// 机器人重心在机身坐标系下的位置(仿真中认为重心就是机器人机体形心)
	const Eigen::Vector3d Pg(0, 0, 0);
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
	const Eigen::Vector3d gaitK(0.01, 0.01, 0.05);
}