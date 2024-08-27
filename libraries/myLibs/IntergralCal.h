#pragma once

#include "PIDmethod.h"
#include "SecondButterworthLPF.h"

// namespace INT_CAL_INTERNAL
// {
//     template <class Int_Filter_Type>
//     class IntergralCal_Base : public PIDtimer
//     {
//     public:
//         /* 用模板构造函数，解决不同filter的构造函数参数不同的问题, 如果构造函数的参数不符合filter类型的构造函数传参，则会编译失败 */
//         template <typename... Args>
//         IntergralCal_Base(Args... args) : filter(args...) {}

//         inline double get_intergral() const { return intergral; }

//         double calc(double input)
//         {
//             if (UpdataTimeStamp()) // 计算dt
//                 return 0;

//             return calc(input, this->dt);
//         }

//         double calc(double input, double _dt)
//         {
//             origin_int += input * _dt; // 积分原始数据

//             intergral = get_filter_result(origin_int);
//             return intergral;
//         }

//         double clear()
//         {
//             origin_int = 0;
//         }

//     protected:
//         Int_Filter_Type filter;
//         inline virtual double get_filter_result(double data)
//         {
//             return filter.f(data);
//         }
//         double origin_int = 0;
//         double intergral;
//     };
// }

// template <class Int_Filter_Type>
// class IntergralCal : public INT_CAL_INTERNAL::IntergralCal_Base<Int_Filter_Type>
// {
// public:
//     /* 用模板构造函数，解决不同filter的构造函数参数不同的问题, 如果构造函数的参数不符合filter类型的构造函数传参，则会编译失败 */
//     template <typename... Args>
//     IntergralCal(Args... args) : INT_CAL_INTERNAL::IntergralCal_Base<Int_Filter_Type>(args...) {}
// };

class IntergralCal : public PIDtimer
{
    public:
        inline double get_intergral() const { return intergral; }

        double calc(double input)
        {
            if (UpdataTimeStamp()) // 计算dt
                return 0;

            return calc(input, this->dt);
        }

        double calc(double input, double _dt)
        {
            origin_int += input * _dt; // 积分原始数据

            intergral = origin_int;
            return intergral;
        }

        void clear()
        {
            origin_int = 0;
            intergral = 0;
        }
    protected:
        double origin_int = 0;
        double intergral = 0;  
};
