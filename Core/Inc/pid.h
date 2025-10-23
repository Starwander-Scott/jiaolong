//
// Created by chenyincheng on 2025/10/18.
//

#ifndef TEST4_PID_H
#define TEST4_PID_H

class PID {
public:
    PID(void);
    PID(float kp, float ki, float kd, float i_max, float out_max,
        float d_filter_k = 1);

    void reset(void);
    float calc(float ref, float fdb);

    float kp_, ki_, kd_, d_filter_k_;
    float i_max_, out_max_;
    float output_;

private:
    float ref_, fdb_;
    float err_, err_sum_, last_err_;
    float pout_, iout_, dout_, last_dout_;
};

#endif//TEST4_PID_H


//#ifndef TEST4_PID_H
//#define TEST4_PID_H
//
//class PID {
//public:
//    PID(void);
//    // 构造函数，初始化PID控制器的参数
//    PID(float kp, float ki, float kd, float i_max, float out_max,
//        float d_filter_k = 1);
//
//    // 重置PID控制器的状态
//    void reset(void);
//    // 计算PID控制器的输出
//    float calc(float ref, float fdb);
//
//    // 比例系数
//    float kp_;
//    // 积分系数
//    float ki_;
//    // 微分系数
//    float kd_;
//    // 微分滤波系数
//    float d_filter_k_;
//    // 积分项的最大值
//    float i_max_;
//    // 输出的最大值
//    float out_max_;
//    // PID控制器的输出
//    float output_;
//
//private:
//    // 参考值
//    float ref_;
//    // 反馈值
//    float fdb_;
//    // 当前误差
//    float err_;
//    // 误差累计和
//    float err_sum_;
//    // 上一次的误差
//    float last_err_;
//    // 比例项的输出
//    float pout_;
//    // 积分项的输出
//    float iout_;
//    // 微分项的输出
//    float dout_;
//    // 上一次的微分项输出
//    float last_dout_;
//};
