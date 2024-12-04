#pragma once
#include <iostream>
#include <stdint.h>
#include <Eigen/Dense>

#define XSPAN 0.02
#define XMOVE 0
#define YSPAN 0
#define YMOVE 0

namespace Quadruped {

	const double L1 = 0.0838f;
	const double L2 = 0.2f;
	const double L3 = 0.2f;
	// 腿部基坐标系到机身坐标系的转换向量，要转换到机身坐标系，直接加上此向量(此处向量仅代表左前腿)
	const Eigen::Vector3d leg2bodyFrame(0.1805, 0.047, 0);
	// 初始右后腿位置
	const Eigen::Vector3d initRbLegXYPosition(-0.1805 - XSPAN, -0.1308 - YSPAN, 0);
	// 机身参数
	const Eigen::Vector<double,6> Imid(1.585330e-02,3.779990e-02,4.565420e-02,-3.660000e-05,-6.110000e-05,-2.750000e-05);
	const double Mmid = 6.000000;
	const Eigen::Vector3d Pmid(0.000000,0.004100,-0.000500);
	const Eigen::Vector<double, 6> Ihead(0,0,0,0,0,0);
	const double Mhead = 0;
	const Eigen::Vector3d Phead(0, 0, 0);
	const Eigen::Vector<double, 6> Itail(0, 0, 0, 0, 0, 0);
	const double Mtail = 0;
	const Eigen::Vector3d Ptail(0, 0, 0);
	// 髋参数(左前腿为例)
	const Eigen::Vector<double, 6> Ihip(4.692460e-04,8.074900e-04,5.529290e-04,-9.409000e-06,-3.420000e-07,-4.660000e-07);
	const double Mhip = 0.696000;
	const Eigen::Vector3d Phip(-0.003311,0.000635,0.000031);
	// 大腿参数(左前腿为例)
	const Eigen::Vector<double, 6> Ithigh(5.529065e-03,5.139339e-03,1.367788e-03,4.825000e-06,3.438690e-04,2.244800e-05);
	const double Mthigh = 1.013000;
	const Eigen::Vector3d Pthigh(-0.003237,-0.022327,-0.027326);
	// 小腿参数(左前腿为例)
	const Eigen::Vector<double, 6> Icalf(2.997972e-03,3.014022e-03,3.242600e-05,0.000000e+00,-1.411630e-04,0.000000e+00);
	const double Mcalf = 0.166000;
	const Eigen::Vector3d Pcalf(0.006435,0.000000,-0.107388);
	// 足参数(左前腿为例)
	const Eigen::Vector<double, 6> Ifoot(9.600000e-06,9.600000e-06,9.600000e-06,0,0,0);
	const double Mfoot = 0.060000;
	const Eigen::Vector3d Pfoot(0,0,0);
	// mpc平衡控制器权重参数
	/*const Eigen::Vector<double, 6> Q(3000, 3000, 5000, 200, 200, 200);
	const Eigen::Vector<double, 6> F = Q;
	const Eigen::Vector<double, 12> R = Eigen::Vector<double, 12>::Constant(1);
	const Eigen::Vector<double, 12> W = Eigen::Vector<double, 12>::Constant(0.8);
	const Eigen::Vector<double, 9> linPD(15, -0., 5, 20, -0., 7, 30, -0., 6);
	const Eigen::Vector<double, 9> angPD(5, -0., 3, 10, -0., 5, 5, -0., 3);*/
	const Eigen::Vector<double, 15> Q(40000, 40000, 100000, 5000, 10000, 5000,100,100,120,20,50,20,1e-6,1e-6,1e-6);
	const Eigen::Vector<double, 15> F = Q;
	const Eigen::Vector<double, 12> R = Eigen::Vector<double, 12>::Constant(1e-4);
	const Eigen::Vector<double, 12> W = Eigen::Vector<double, 12>::Constant(1e-6);

	//// mpc约束
	//const Eigen::Vector<double, 12> lb = Eigen::Vector<double, 12>::Constant(-100);
	//const Eigen::Vector<double, 12> ub = Eigen::Vector<double, 12>::Constant(100);

	// 步态运动期望增益
	const Eigen::Vector3d gaitK(-0.005, -0.005, 0.005);
}