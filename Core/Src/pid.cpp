//
// Created by chenyincheng on 2025/10/18.
//

//pid.h中定义的PID类

//class PID {
//public:
//    PID(void) : PID(0, 0, 0, 0, 0) {}
//    PID(float kp, float ki, float kd, float i_max, float out_max,
//        float d_filter_k = 1);
//
//    void reset(void);
//    float calc(float ref, float fdb);
//
//    float kp_, ki_, kd_, d_filter_k_;
//    float i_max_, out_max_;
//    float output_;
//
//private:
//    float ref_, fdb_;
//    float err_, err_sum_, last_err_;
//    float pout_, iout_, dout_, last_dout_;
//};

//这里提供对PID类方法的实现。
#include "pid.h"

// 带参数的构造函数
PID::PID(float kp, float ki, float kd, float i_max, float out_max, float d_filter_k)
    : kp_(kp), ki_(ki), kd_(kd), i_max_(i_max), out_max_(out_max), d_filter_k_(d_filter_k) {
    reset();// 初始化时重置所有状态变量
}

// 默认构造函数，调用带参数构造函数并初始化为0
PID::PID(void) : PID(0, 0, 0, 0, 0) {}

// 重置PID控制器状态
void PID::reset(void) {
    ref_ = 0.0f;
    fdb_ = 0.0f;
    err_ = 0.0f;
    err_sum_ = 0.0f;
    last_err_ = 0.0f;
    pout_ = 0.0f;
    iout_ = 0.0f;
    dout_ = 0.0f;
    last_dout_ = 0.0f;
    output_ = 0.0f;
}

// PID计算函数
float PID::calc(float ref, float fdb) {
    // 更新参考值和反馈值
    ref_ = ref;
    fdb_ = fdb;

    // 计算当前误差
    err_ = ref_ - fdb_;

    // 比例项
    pout_ = kp_ * err_;

    // 积分项（带积分限幅）
    err_sum_ += err_;
    if (ki_ != 0) {
        // 积分限幅
        float temp_iout = ki_ * err_sum_;
        if (temp_iout > i_max_) {
            temp_iout = i_max_;
            err_sum_ = i_max_ / ki_;// 防止积分饱和
        } else if (temp_iout < -i_max_) {
            temp_iout = -i_max_;
            err_sum_ = -i_max_ / ki_;// 防止积分饱和
        }
        iout_ = temp_iout;
    } else {
        iout_ = 0.0f;
    }

    // 微分项（带滤波）
    if (kd_ != 0) {
        float raw_dout = kd_ * (err_ - last_err_);
        // 一阶低通滤波：dout = d_filter_k * raw_dout + (1 - d_filter_k) * last_dout
        dout_ = d_filter_k_ * raw_dout + (1.0f - d_filter_k_) * last_dout_;
        last_dout_ = dout_;
    } else {
        dout_ = 0.0f;
    }

    // 更新上一次误差
    last_err_ = err_;

    // 计算总输出并限幅
    output_ = pout_ + iout_ + dout_;

    if (output_ > out_max_) {
        output_ = out_max_;
    } else if (output_ < -out_max_) {
        output_ = -out_max_;
    }

    return output_;
}