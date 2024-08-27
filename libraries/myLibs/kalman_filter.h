#pragma once

#include "myMatrices.h"
USING_NAMESPACE_MM

//卡尔曼滤波常用类型
template<class T,uint8_t xNum,uint8_t uNum>
class kalmanFilter {
private:
	myMatrices<T> A = myMatrices<T>(xNum);//状态转移矩阵
	myMatrices<T> B = myMatrices<T>(xNum, uNum);//输入矩阵
	myMatrices<T> Q = myMatrices<T>(xNum);//过程噪声协方差矩阵
	myMatrices<T> R = myMatrices<T>(xNum);//输入噪声协方差矩阵
	myMatrices<T> H = myMatrices<T>(xNum);//测量矩阵
	myMatrices<T> P = myMatrices<T>(xNum);//后验估计协方差矩阵
	myMatrices<T> x = myMatrices<T>(xNum, 1);//保留每次的状态
public:
	//构造函数
	kalmanFilter() { P.eye(); }
	//设置状态空间方程，及测量矩阵
	void setFunc(const T A_array[xNum * xNum], const T B_array[xNum * uNum],const T H_array[xNum * xNum])
	{
		A.setArray(A_array, xNum * xNum);
		B.setArray(B_array, xNum * uNum);
		H.setArray(H_array, xNum * xNum);
	}
	//设置协方差矩阵(注意，协方差矩阵可以很小，但不能为零)
	void setConv(const T Q_array[xNum * xNum], const T R_array[xNum * xNum])
	{
		Q.setArray(Q_array, xNum * xNum);
		R.setArray(R_array, xNum * xNum);
	}
	//求解卡尔曼滤波
	void f(const T u_array[uNum], const T z_array[xNum])
	{
		myMatrices<T> u(uNum, 1);
		myMatrices<T> z(xNum, 1);
		u.setArray(u_array, uNum);
		z.setArray(z_array, xNum);
		//计算先验状态估计
		myMatrices<T> x_minus = A * x + B * u;
		//计算先验估计协方差
		myMatrices<T> P_minus = A * P * A.transpose() + Q;
		//计算卡尔曼增益
		myMatrices<T> temp = H * P_minus * H.transpose() + R;
		myMatrices<T> K = P_minus * H.transpose() * temp.inverse();
		//更新后验估计
		x = x_minus + K * (z - H * x_minus);
		//更新后验估计协方差
		myMatrices<T> E(xNum);
		E.eye();
		P = (E - K * H) * P_minus;
	}
	const myMatrices<T> getOut()
	{
		return x;
	}
};
