//
// Created by chenyincheng on 2025/10/3.
//

//这里定义了电机的一些参数，包括电机减速比、电流、转速等
#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "pid.h"
#include "stdint.h"
extern uint8_t stop_flag;
extern float target_speed;
extern float target_angle;
float linearMapping(int in, int in_min, int in_max, float out_min,
                    float out_max);

class Motor {
private:
    float ratio_;                // 电机减速比
    float angle_ = 0.f;          // deg 输出端累计转动角度
    float normalized_angle = 0.f;// deg 归一化角度(0~360)
    float delta_angle_ = 0.f;    // deg 输出端新转动的角度
    float ecd_angle_ = 0.f;      // deg 当前电机编码器角度
    float last_ecd_angle_ = 0.f; // deg 上次电机编码器角度
    float delta_ecd_angle_ = 0.f;// deg 编码器端新转动的角度
    float rotate_speed_ = 0.f;   // dps 反馈转子转速
    float current_ = 0.f;        // A 反馈转矩电流
    float temp_ = 0.f;           // °C 反馈电机温度
    float flag1 = 0.f;
    float flag2 = 0.f;
    float flag3 = 0.f;
    float flag4 = 0.f;
    float res1 = 0.f;
    float res2 = 0.f;
    float res3 = 0.f;
    float res4 = 0.f;
    float res5 = 0.f;
    float res6 = 0.f;
    int result = 0;
    int result1 = 0;
    int result2 = 0;
    int16_t result3 = 0;

    int sum = 0;


    // PID控制器实例
    static PID spid_;// 速度环PID
    static PID ppid_;// 位置环PID

    // 控制相关变量
    //    float target_angle,
    float fdb_angle_;
    //    float target_speed_ = 0;
    float fdb_speed_, feedforward_speed_;
    float feedforward_intensity_, output_intensity_;
    float gravity_ff = 0.0f;
    uint16_t current_raw_ = 0;


    // 控制模式枚举
    enum {
        TORQUE,        // 扭矩控制
        SPEED,         // 速度控制
        POSITION_SPEED,// 位置-速度双环控制
    } control_method_;

public:
    //explicit M3508_Motor(const float ratio) : ratio_(ratio) {};
    float normalizeAngle(float angle);

    // 获取当前角度（基于编码器）
    float getCurrentAngle();

    // 获取当前速度（基于编码器变化）
    float getCurrentSpeed();

    // 设置电机电流
    void setCurrent(float current);

    // CAN发送函数（用于设置电流）
    //void sendCurrentToMotor(int16_t current);


    void canRxMsgCallback(const uint8_t rx_data[8]);

    Motor(float reduction_ratio);// 修改构造函数
    void handle();               // 处理函数，根据控制模式计算输出

    void SetPosition(float target_position, float feedforward_speed, float feedforward_intensity);
    void SetSpeed(float target_speed, float feedforward_intensity);
    void SetIntensity(float intensity);
    float FeedforwardIntensityCalc(float current_angle);
    void Motor_Stop();
};

#endif//__MOTOR_H__
