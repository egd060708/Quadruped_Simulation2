#pragma once
#include <iostream>
#include <stdint.h>
#include <Eigen/Dense>

#define YSPAN 0.0
#define XMOVE 0.0
#define XSPANF 0.06
#define XSPANB 0.08

namespace Quadruped {
	// 腿长参数
	const double L1 = 0.11973;// 髋
	const double L2 = 0.35;// 大腿
	const double L3 = -0.000088;
	const double L4 = 0.35;// 小腿
	const double L5 = 0.04319;// 末端执行器
	const double L6 = 0.0575;
	const double L7 = 0.055;
	// 腿部基坐标系到机身坐标系的转换向量，要转换到机身坐标系，直接加上此向量(此处向量仅代表左前腿)
	const Eigen::Vector3d leg2bodyFrame(0.3285, 0.072, 0);
	//// 初始位型右后腿位置
	//const Eigen::Vector3d initRbLegXYPosition(-0.3154553 + XMOVE - XSPAN, -0.234832 - YSPAN, 0);
	// 机身参数
	const Eigen::Vector<double, 6> Imid(2.746600e-01,1.061800e+00,1.182500e+00,-6.220000e-04,3.150000e-03,-1.390000e-03);
	const double Mmid = 35.606;
	const Eigen::Vector3d Pmid(0.000458, 0.005261, 0.000665);
	const Eigen::Vector<double, 6> Ihead(2.645500e-02,2.922100e-02,1.091800e-02,3.150000e-05,1.660000e-03,-3.270000e-05);
	const double Mhead = 3.688500;
	const Eigen::Vector3d Phead(0.339890,-0.000168,0.120290);
	const Eigen::Vector<double, 6> Itail(2.811800e-03,4.750600e-03,2.915400e-03,1.950000e-05,-7.870000e-06,-1.570000e-05);
	const double Mtail = 0.795000;
	const Eigen::Vector3d Ptail(-0.374480,0.000488,0.006495);
	// 髋参数(左前腿为例)(初始位型水平放置)
	const Eigen::Vector<double, 6> Ihip(3.318800e-03,4.874300e-03,3.708700e-03,7.160000e-05,3.770000e-07,-4.000000e-09);
	const double Mhip = 2.673;
	const Eigen::Vector3d Phip(-0.003841, -0.009068, 0.000000);
	// 大腿参数(左前腿为例)(初始位型竖直向下)
	const Eigen::Vector<double, 6> Ithigh(6.229900e-02,6.139900e-02,8.199700e-03,8.778100e-04,-3.647500e-03,8.353700e-03);
	const double Mthigh = 4.536;
	const Eigen::Vector3d Pthigh(-0.006279, -0.032049, -0.057835);
	// 小腿参数(左前腿为例)(初始位型竖直向下)
	const Eigen::Vector<double, 6> Icalf(2.928300e-02,3.086800e-02,3.260000e-03,2.210000e-04,-1.064000e-03,2.093000e-03);
	const double Mcalf = 2.290600;
	const Eigen::Vector3d Pcalf(0.005278, 0.017301, -0.297780);
	// 足参数(左前腿为例)(初始位型为旋转方向沿世界坐标系y轴)
	const Eigen::Vector<double, 6> Ifoot(4.121000e-03,7.776000e-03,4.108000e-03,6.000000e-06,-4.800000e-05,7.000000e-06);
	const double Mfoot = 1.083000;
	const Eigen::Vector3d Pfoot(-0.000991,0.051053,-0.001130);
	// mpc平衡控制器权重参数
	/*const Eigen::Vector<double, 10> Q(400000, 300000, 600000, 30000, 50000, 30000,5000,5000,5000,5000);
	const Eigen::Vector<double, 10> F = Q;*/
	/*const Eigen::Vector<double, 23> Q(1e5, 6e4, 2e5,\
									  5e4, 1e5, 5e4,\
									  1e5, 1e5, 1e5, 1e5,\
									  2e2, 6e2, 1e3,\
									  5e1, 5e1, 5e1,\
									  5e1, 5e1, 5e1, 5e1,\
									  1e-6, 1e-6, 1e-6);*/
	/*const Eigen::Vector<double, 23> Q(1e5, 6e4, 3e5, \
									  5e4, 1e5, 5e4, \
									  1e5, 1e5, 1e5, 1e5, \
									  2e2, 5e2, 1e3, \
									  45, 64, 45, \
									  5e1, 5e1, 5e1, 5e1, \
									  1e-6, 1e-6, 1e-6);*/
	const Eigen::Vector<double, 23> Q(8e4, 6e4, 3e5, \
									  5e4, 1e5, 5e4, \
									  5e4, 5e4, 5e4, 5e4, \
									  5e2, 5e2, 2e3, \
									  45, 64, 45, \
									  2e2, 2e2, 2e2, 2e2, \
									  1e-6, 1e-6, 1e-6);
	/*const Eigen::Vector<double, 23> Q(8e4, 6e4, 3e5, \
										5e4, 1e5, 5e4, \
										5e4, 5e4, 5e4, 5e4, \
										5e2, 5e2, 2e3, \
										90, 128, 90, \
										2e2, 2e2, 2e2, 2e2, \
										1e-6, 1e-6, 1e-6);*/
	const Eigen::Vector<double, 23> F = Q;
	const Eigen::Vector<double, 16> R = Eigen::Vector<double, 16>::Constant(1e-4);
	const Eigen::Vector<double, 16> W = Eigen::Vector<double, 16>::Constant(1e-4);


	/*const Eigen::Vector<double, 16> R = Eigen::Vector<double, 16>::Constant(1);
	const Eigen::Vector<double, 16> W = Eigen::Vector<double, 16>::Constant(0.8);
	const Eigen::Vector<double, 9> linPD(20, -0.2, 2, 20, -0.2, 2, 35, -0.35, 2);
	const Eigen::Vector<double, 9> angPD(30, -0.1, 5, 30, -0.2, 10, 10, -0.5, 5);
	const Eigen::Vector<double, 3> wheelPID(20, -0.2, 2);*/

	//// mpc约束
	//const Eigen::Vector<double, 12> lb = Eigen::Vector<double, 12>::Constant(-100);
	//const Eigen::Vector<double, 12> ub = Eigen::Vector<double, 12>::Constant(100);

	// 步态运动期望增益
	const Eigen::Vector3d gaitK(-0.002, -0.002, 0.004);

	// 轨迹规划mpc参数测试
	const Eigen::Vector<double, 6> traBQ(100, 100, 300, 100, 100, 10);
	const Eigen::Vector<double, 6> traBF = traBQ;
	const Eigen::Vector<double, 3> traBR = Eigen::Vector<double, 3>::Constant(1e-4);
	const Eigen::Vector<double, 3> traBW = Eigen::Vector<double, 3>::Constant(1e-3);
	const Eigen::Vector3d bodyAccL(3., 3., 4);

	const Eigen::Vector<double, 16> traLQ(100, 100, 100, 100, 100, 100, 100, 100,\
											1, 1, 1, 1, 1, 1, 1, 1);
	const Eigen::Vector<double, 16> traLF = traLQ;
	const Eigen::Vector<double, 8> traLR = Eigen::Vector<double, 8>::Constant(1e-4);
	const Eigen::Vector<double, 8> traLW = Eigen::Vector<double, 8>::Constant(1e-3);
	const Eigen::Vector<double, 8> footAccL = Eigen::Vector<double, 8>::Constant(5.);

	// 联合轨迹规划方法
	//const Eigen::Vector<double, 22> traQ(250, 250, 300, 20, 20, 10, \
	//									50, 50, 50, 50, 50, 50, 50, 50, \
	//									1, 1, 1, 1, 1, 1, 1, 1);
	const Eigen::Vector<double, 22> traQ(25, 25, 300, 50, 50, 15, \
										 50, 50, 50, 50, 50, 50, 50, 50, \
										 1, 1, 1, 1, 1, 1, 1, 1);
	const Eigen::Vector<double, 22> traF = traQ;
	const Eigen::Vector<double, 11> traR = Eigen::Vector<double, 11>::Constant(1e-4);
	const Eigen::Vector<double, 11> traW = Eigen::Vector<double, 11>::Constant(1e-3);
	const Eigen::Vector<double, 11> accL(4., 2., 2., 2., 2., 2., 2., 2., 2., 2., 2.);
}