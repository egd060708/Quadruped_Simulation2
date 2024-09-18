#ifndef __MATHTOOL_H
#define __MATHTOOL_H

double dot_v3d_v3d(double _row3d[3], double _col3d[3]);
void mult_m3d_v3d(double _dst[3], double _m3d[3][3], double _col3d[3]);
void mult_m4d_v4d(double _dst[4], double _m4d[4][4], double _col4d[4]);
void mult_m3d_m3d(double _dst[3][3], double _m3dL[3][3], double _m3dR[3][3]);
void rotation_x(double _dst[3][3], double _rx);
void rotation_y(double _dst[3][3], double _ry);
void rotation_z(double _dst[3][3], double _rz);
void m3d_transpose1(double _dst[3][3]);
void m3d_transpose2(double _dst[3][3], double _src[3][3]);
void transform_calc(double _dst[3], double _src[3], double _R[3][3], double _P[3]);
void invTransform_calc(double _dst[3], double _src[3], double _invR[3][3], double _P[3]);
double slopeConstrain(double _tar, double _cur, double _acc, double _dec);

#endif
