#pragma once
#include <iostream>
#include <stdint.h>
#include <Eigen/Dense>

namespace Quadruped{

const float L1 = 0.0838f;
const float L2 = 0.2f;
const float L3 = 0.2f;
//腿部基坐标系到机身坐标系的转换向量，要转换到机身坐标系，直接加上此向量(此处向量仅代表左前腿)
const Vector3f leg2bodyFrame(0.1805,0.047,0);

}