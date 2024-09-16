#include "mathTool.h"
#include <math.h>
#include <iostream>

/* 三维向量点乘 */
double dot_v3d_v3d(double _row3d[3], double _col3d[3])
{
	double result = 0;
	for (int i = 0; i < 3; i++)
	{
		result += _row3d[i] * _col3d[i];
	}
	return result;
}

/* 三维矩阵与三维向量相乘 */
void mult_m3d_v3d(double _dst[3], double _m3d[3][3], double _col3d[3])
{
	for (int i = 0; i < 3; i++)
	{
		double res = 0;
		for (int j = 0; j < 3; j++)
		{
			res += _m3d[i][j] * _col3d[j];
		}
		_dst[i] = res;
	}
}

/* 四维矩阵与三维向量相乘 */
void mult_m4d_v4d(double _dst[4], double _m4d[4][4], double _col4d[4])
{
	for (int i = 0; i < 4; i++)
	{
		double res = 0;
		for (int j = 0; j < 4; j++)
		{
			res += _m4d[i][j] * _col4d[j];
		}
		_dst[i] = res;
	}
}

/* 三维矩阵相乘 */
void mult_m3d_m3d(double _dst[3][3], double _m3dL[3][3], double _m3dR[3][3])
{
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			double res = 0;
			for (int k = 0; k < 3; k++)
			{
				res += _m3dL[i][k] * _m3dR[k][j];
			}
			_dst[i][j] = res;
		}
	}
}

/* 构造绕x轴的旋转矩阵 */
void rotation_x(double _dst[3][3], double _rx)
{
	_dst[0][0] = 1;
	_dst[0][1] = 0;
	_dst[0][2] = 0;
	_dst[1][0] = 0;
	_dst[2][0] = 0;
	_dst[1][1] = cos(_rx);
	_dst[2][2] = cos(_rx);
	_dst[1][2] = -sin(_rx);
	_dst[2][1] = sin(_rx);
}

/* 构造绕y轴的旋转矩阵 */
void rotation_y(double _dst[3][3], double _ry)
{
	_dst[1][1] = 1;
	_dst[1][0] = 0;
	_dst[0][1] = 0;
	_dst[1][2] = 0;
	_dst[2][1] = 0;
	_dst[0][0] = cos(_ry);
	_dst[0][2] = sin(_ry);
	_dst[2][0] = -sin(_ry);
	_dst[2][2] = cos(_ry);
}

/* 构造绕z轴的旋转矩阵 */
void rotation_z(double _dst[3][3], double _rz)
{
	_dst[2][2] = 1;
	_dst[2][0] = 0;
	_dst[2][1] = 0;
	_dst[0][2] = 0;
	_dst[1][2] = 0;
	_dst[0][0] = cos(_rz);
	_dst[1][0] = sin(_rz);
	_dst[0][1] = -sin(_rz);
	_dst[1][1] = cos(_rz);
}

/* 3d矩阵对自身求转置 */
void m3d_transpose1(double _dst[3][3])
{
	double temp[3][3];
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			temp[i][j] = _dst[j][i];
		}
	}
	memcpy(_dst, temp, 9 * sizeof(double));
}

/* 3d矩阵求转置并保存到另一个矩阵 */
void m3d_transpose2(double _dst[3][3], double _src[3][3])
{
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			_dst[i][j] = _src[j][i];
		}
	}
}

/* 正齐次变换计算 */
void transform_calc(double _dst[3],double _src[3], double _R[3][3], double _P[3])
{
	double T[4][4] = { 0 };
	double P4d[4] = { 0 };
	T[3][3] = 1;
	P4d[3] = 1;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			T[i][j] = _R[i][j];
		}
		T[i][3] = _P[i];
		P4d[i] = _src[i];
	}
	double Res4d[4];
	mult_m4d_v4d(Res4d, T, P4d);
	memcpy(_dst, Res4d, 3 * sizeof(double));
}

/* 逆齐次变换计算 */
void invTransform_calc(double _dst[3], double _src[3], double _invR[3][3], double _P[3])
{
	double T[4][4] = { 0 };
	double P4d[4] = { 0 };
	double TP3d[3] = { 0 };
	T[3][3] = 1;
	P4d[3] = 1;
	mult_m3d_v3d(TP3d, _invR, _P);
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			T[i][j] = _invR[i][j];
		}
		T[i][3] = -TP3d[i];
		P4d[i] = _src[i];
	}
	double Res4d[4];
	mult_m4d_v4d(Res4d, T, P4d);
	memcpy(_dst, Res4d, 3 * sizeof(double));
}