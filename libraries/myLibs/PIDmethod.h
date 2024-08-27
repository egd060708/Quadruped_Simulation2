/*! @file PIDmethod.h
*  @brief PID Algorithm adjusts manipulator robot
*
*  This file contains the PIDmethod class and PIDtimer class, which provide ways to run PID.
*
*/
#ifndef _PIDMETHOD_H_
#define _PIDMETHOD_H_

#include <iostream>
#include <stdint.h>
#include <cstdint>
#include <stddef.h>
#include <math.h>
#include <limits>//定义各种变量的储存最大值
#include "Upper_Public.h"

using namespace std;


typedef uint32_t(*SystemTick_Fun)(void);//函数指针，用于接入不同单片机获取时间栈的函数

typedef enum
{
    Common,
    Fit
}Params_Mode;

typedef enum class _PID_Mode
{
    IS_PI,
    IS_PD
}PID_Mode;

class PIDtimer
{
public:
    static uint8_t getMicroTick_regist(uint32_t(*getTick_fun)(void));//获取当前时间函数接口
    static SystemTick_Fun Get_SystemTick;   //获取时间的函数
    double dt = 0;				                //时间微分
    uint32_t last_time = 0; 	                //记录上次时间
    uint8_t UpdataTimeStamp(void);          //时间栈更新

};

/*PID方法类*/
class PIDmethod : public PIDtimer
{
public:
    PIDmethod() {}
    PIDmethod(Params_Mode mode, double _timeStep = 0);//必须输入参数模式，时间栈选择输入
    void PID_Init(Params_Mode mode, double _timeStep = 0);
    //设置参数
    //拟合线性函数方法
    void Params_Config(Fit_Params _fun_p, Fit_Params _fun_i, Fit_Params _fun_d, double _I_Term_Max, double _Output_Max, double _Output_Min = numeric_limits<double>::max());
    void Params_Config(Fit_Params _fun_p, double _I_Term_Max, double _Output_Max, double _Output_Min = numeric_limits<double>::max());//纯p
    void Params_Config(PID_Mode mode, Fit_Params _fun_p, Fit_Params _fun_id, double _I_Term_Max, double _Output_Max, double _Output_Min = numeric_limits<double>::max());//pi或pd控制
    //普通参数方法
    void Params_Config(double _kp, double _ki, double _kd, double _I_Term_Max, double _Output_Max, double _Output_Min = numeric_limits<double>::max());
    void Params_Config(double _kp, double _I_Term_Max, double _Output_Max, double _Output_Min = numeric_limits<double>::max());//纯p
    void Params_Config(PID_Mode mode, double _kp, double _kid, double _I_Term_Max, double _Output_Max, double _Output_Min = numeric_limits<double>::max());//pi或pd控制
    //结算
    double Adjust(double _x);//线性自变量，使用自己算的d项
    double Adjust(double _x, double extern_d);//线性自变量，使用外部计算的d项

    Fit_Params fun_p = { 0 }, fun_i = { 0 }, fun_d = { 0 };//三次拟合参数
    double kp = 0, ki = 0, kd = 0;//普通参数
    double fact_kp = 0, fact_ki = 0, fact_kd = 0;

    double Error_Max = numeric_limits<double>::max();//P项限幅
    double I_Term_Max = 0;//I项限幅
    double Output_Max = 0;//输出上限
    double Output_Min = 0;//输出下限

    double P_Term = 0;//比例项输出
    double I_Term = 0;//积分项输出
    double D_Term = 0;//微分项输出

    double I_SeparThresh = 400;   /*!< 积分分离阈值，需为正数。std::abs(error)大于该阈值取消积分作用。*/

    double target = 0;
    double current = 0;
    double error = 0;
    double out = 0;

    double timeStep = 0;//如果被赋值，则以此为微分时间

    bool d_of_current = true;//是否使用微分先行
private:
    //线性拟合函数系数+自变量
    double fit_function(Fit_Params param, double x);
    Params_Mode params_mode = Common;
    double last_current = 0;
    double last_error = 0;
    double d_current = 0;
    double d_error = 0;
    double integral = 0;

};

SystemTick_Fun PIDtimer::Get_SystemTick = NULL;//静态变量必须实现

uint8_t PIDtimer::UpdataTimeStamp(void)
{
    uint32_t now_time;

    /*Check `Get_SystemTick` */
    if (PIDtimer::Get_SystemTick != NULL)
    {
        /*Convert to system time*/
        if (last_time == 0)
        {
            last_time = PIDtimer::Get_SystemTick();
            return 1;//第一步先不跑
        }
        now_time = PIDtimer::Get_SystemTick();

        /*Overflow*/
        if (now_time < last_time)
            dt = (double)(now_time + (0xFFFFFFFF - last_time));
        else
            dt = (double)(now_time - last_time);

        last_time = now_time;

        dt *= (double)0.000001;

        return 0;
    }
    else {
        dt = 0.001f;
        return 1;
    }
}

/**
 * @brief  Regist get time function(1Tick = 1us)
 * @param  realTime_fun: Pointer of function to get system real time
 * @retval 1: success
           0: error input param
 * @author
 */
uint8_t PIDtimer::getMicroTick_regist(uint32_t(*getTick_fun)(void))
{
    if (getTick_fun != NULL)
    {
        PIDtimer::Get_SystemTick = getTick_fun;
        return 1;
    }
    else
        return 0;
}

/*设置PID控制模式*/
PIDmethod::PIDmethod(Params_Mode mode, double _timeStep)
{
    params_mode = mode;
    timeStep = _timeStep;
}

void PIDmethod::PID_Init(Params_Mode mode, double _timeStep)
{
    params_mode = mode;
    timeStep = _timeStep;
}

/****************************************** PID参数录入 ******************************************/
void PIDmethod::Params_Config(Fit_Params _fun_p, Fit_Params _fun_i, Fit_Params _fun_d, double _I_Term_Max, double _Output_Max, double _Output_Min)
{
    fun_p = _fun_p;
    fun_i = _fun_i;
    fun_d = _fun_d;
    I_Term_Max = _I_Term_Max;
    Output_Max = _Output_Max;
    Output_Min = _Output_Min;
}

void PIDmethod::Params_Config(Fit_Params _fun_p, double _I_Term_Max, double _Output_Max, double _Output_Min)
{
    fun_p = _fun_p;
    I_Term_Max = _I_Term_Max;
    Output_Max = _Output_Max;
    Output_Min = _Output_Min;
}

void PIDmethod::Params_Config(PID_Mode mode, Fit_Params _fun_p, Fit_Params _fun_id, double _I_Term_Max, double _Output_Max, double _Output_Min)
{
    fun_p = _fun_p;
    if (mode == PID_Mode::IS_PI)
    {
        fun_i = _fun_id;
    }
    else if (mode == PID_Mode::IS_PD)
    {
        fun_d = _fun_id;
    }
    else {}
    I_Term_Max = _I_Term_Max;
    Output_Max = _Output_Max;
    Output_Min = _Output_Min;
}

void PIDmethod::Params_Config(double _kp, double _ki, double _kd, double _I_Term_Max, double _Output_Max, double _Output_Min)
{
    kp = _kp;
    ki = _ki;
    kd = _kd;
    I_Term_Max = _I_Term_Max;
    Output_Max = _Output_Max;
    Output_Min = _Output_Min;
}

void PIDmethod::Params_Config(double _kp, double _I_Term_Max, double _Output_Max, double _Output_Min)
{
    kp = _kp;
    I_Term_Max = _I_Term_Max;
    Output_Max = _Output_Max;
    Output_Min = _Output_Min;
}

void PIDmethod::Params_Config(PID_Mode mode, double _kp, double _kid, double _I_Term_Max, double _Output_Max, double _Output_Min)
{
    kp = _kp;
    if (mode == PID_Mode::IS_PI)
    {
        ki = _kid;
    }
    else if (mode == PID_Mode::IS_PD)
    {
        kd = _kid;
    }
    else {}
    I_Term_Max = _I_Term_Max;
    Output_Max = _Output_Max;
    Output_Min = _Output_Min;
}

/**************************************** 拟合三次函数 *********************************************/
double PIDmethod::fit_function(Fit_Params param, double x)
{
    return param.a * pow(x, 3) + param.b * pow(x, 2) + x * param.c + param.d;
}
/****************************************** PID运算 **********************************************/
double PIDmethod::Adjust(double _x)
{
    if (timeStep > 0)
    {
        this->dt = timeStep;
    }
    else
    {
        //if (this->UpdataTimeStamp())
        //    return 0;//如果时间栈出错则不执行pid
        this->dt = this->UpdataTimeStamp();
    }

    if (params_mode == Fit)
    {
        fact_kp = fit_function(fun_p, _x);
        fact_ki = fit_function(fun_i, _x);
        fact_kd = fit_function(fun_d, _x);
    }
    else if (params_mode == Common)
    {
        fact_kp = kp;
        fact_ki = ki;
        fact_kd = kd;
    }
    else
    {
        return 0;//参数出错不执行
    }

    error = upper::constrain(target - current, Error_Max);
    /*error = target - current;*/
    d_error = (error - last_error) / this->dt;
    d_current = (current - last_current) / this->dt;

    P_Term = error * fact_kp;

    integral += error * this->dt;
    integral = upper::constrain(integral, I_Term_Max / fact_ki);
    I_Term = integral * fact_ki;
    if (abs(I_Term) > I_SeparThresh)
    {
        I_Term = 0;
    }
    else {}

    if (d_of_current)
    {
        D_Term = d_current * fact_kd;
    }
    else
    {
        D_Term = d_error * fact_kd;
    }

    out = P_Term + I_Term + D_Term;

    if (Output_Min >= Output_Max)
    {
        out = upper::constrain(out, Output_Max);
    }
    else
    {
        out = upper::constrain(out, Output_Min, Output_Max);
    }


    last_current = current;
    last_error = error;
    return out;
}

double PIDmethod::Adjust(double _x, double extern_d)
{

    if (timeStep > 0)
    {
        this->dt = timeStep;
    }
    else
    {
        //if (this->UpdataTimeStamp())
        //    return 0;//如果时间栈出错则不执行pid
        this->dt = this->UpdataTimeStamp();
    }

    if (params_mode == Fit)
    {
        fact_kp = fit_function(fun_p, _x);
        fact_ki = fit_function(fun_i, _x);
        fact_kd = fit_function(fun_d, _x);
    }
    else if (params_mode == Common)
    {
        fact_kp = kp;
        fact_ki = ki;
        fact_kd = kd;
    }
    else
    {
        return 0;//参数出错不执行
    }

    error = upper::constrain(target - current, Error_Max);
    /*error = target - current;*/
    d_error = (error - last_error) / this->dt;
    d_current = (current - last_current) / this->dt;

    P_Term = error * fact_kp;

    integral += error * this->dt;
    integral = upper::constrain(integral, I_Term_Max / fact_ki);
    I_Term = integral * fact_ki;
    if (abs(I_Term) > I_SeparThresh)
    {
        I_Term = 0;
    }
    else {}

    D_Term = extern_d * fact_kd;

    out = P_Term + I_Term + D_Term;
    if (Output_Min >= Output_Max)
    {
        out = upper::constrain(out, Output_Max);
    }
    else
    {
        out = upper::constrain(out, Output_Min, Output_Max);
    }

    last_current = current;
    last_error = error;
    return out;
}

#endif