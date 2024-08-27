#pragma once

#include <math.h>

#define M_PI 3.14159265358979323846

//二阶巴特沃斯低通滤波器
class SecondOrderButterworthLPF {
public:
    SecondOrderButterworthLPF(){}
    // 构造函数初始化：截止频率，采样频率
    SecondOrderButterworthLPF(double cutoff_freq, double sampling_freq) {
        wc = 2.f * M_PI * cutoff_freq / sampling_freq;
        calculateCoefficients();
        reset();
    }

    void init(double cutoff_freq, double sampling_freq) {
        wc = 2.f * M_PI * cutoff_freq / sampling_freq;
        calculateCoefficients();
        reset();
    }

    // 对于输入信号进行滤波处理
    double f(double input) {
        double output = b0 * input + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
        x2 = x1;
        x1 = input;
        y2 = y1;
        y1 = output;
        return output;
    }

    // Reset the filter state
    void reset() {
        x1 = 0.f;
        x2 = 0.f;
        y1 = 0.f;
        y2 = 0.f;
    }

private:
    double wc; // 截止频率
    double b0, b1, b2, a1, a2;
    double x1, x2, y1, y2;

    // 计算滤波器参数
    void calculateCoefficients() {
        // double sqrt2 = sqrtf(2.f);
        double sqrt2wc = sqrtf(2.f) * wc;
        double wc2 = wc * wc;

        b0 = wc2 / (wc2 + sqrt2wc + 1.f);
        b1 = 2.f * b0;
        b2 = b0;
        a1 = 2.f * (wc2 - 1.f) / (wc2 + sqrt2wc + 1.f);
        a2 = (wc2 - sqrt2wc + 1.f) / (wc2 + sqrt2wc + 1.f);
    }
};

