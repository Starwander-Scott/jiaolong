//
// Created by chenyincheng on 2025/10/18.
//

#include "motor.h"
#include "pid.h"

M3508_Motor::Motor(float reduction_ratio)
    :


      // 初始化原有成员

      //reduction_ratio_(reduction_ratio),
      ratio_(reduction_ratio),
      angle_(0), last_ecd_angle_(0), delta_ecd_angle_(0), delta_angle_(0),
      current_(0), temp_(0),
      ecd_angle_(0), rotate_speed_(0),


      spid_(5.0f, 0.1f, 0.05f, 1000.0f, 10000.0f, 0.8f),// 速度环PID参数
      ppid_(10.0f, 0.01f, 0.1f, 500.0f, 8000.0f, 0.9f), // 位置环PID参数
      target_angle_(0), fdb_angle_(0),
      target_speed_(0), fdb_speed_(0), feedforward_speed_(0),
      feedforward_intensity_(0), output_intensity_(0),
      control_method_(TORQUE)// 默认扭矩控制
{
    // 其他初始化代码...
}


void M3508_Motor::canRxMsgCallback(const uint8_t rx_data[8]) {
    // a. 解析16位数据 - 转子机械角度
    uint16_t ecd_angle_raw = (rx_data[0] << 8) | rx_data[1];

    // b. 使用linearMapping将原始角度映射到0-360度范围
    ecd_angle_ = linearMapping(ecd_angle_raw, 0, 8191, 0.0f, 360.0f);

    // c. 计算编码器端角度变化量，处理临界跳变
    delta_ecd_angle_ = ecd_angle_ - last_ecd_angle_;

    // 处理角度在0/360度临界点的跳变
    if (delta_ecd_angle_ > 180.0f) {
        delta_ecd_angle_ -= 360.0f;
    } else if (delta_ecd_angle_ < -180.0f) {
        delta_ecd_angle_ += 360.0f;
    }

    // 计算输出端角度变化（考虑减速比）
    delta_angle_ = delta_ecd_angle_ / ratio_;

    // 累加得到输出端累计角度
    angle_ += delta_angle_;

    // 更新上次编码器角度
    last_ecd_angle_ = ecd_angle_;

    // 解析转子转速（16位有符号，单位RPM）
    int16_t rotate_speed_raw = (rx_data[2] << 8) | rx_data[3];
    rotate_speed_ = static_cast<float>(rotate_speed_raw);// RPM

    // 解析转矩电流（16位有符号，范围-16384~16384对应-20~20A）
    int16_t current_raw = (rx_data[4] << 8) | rx_data[5];
    current_ = linearMapping(current_raw, -16384, 16384, -20.0f, 20.0f);

    // 解析电机温度
    temp_ = static_cast<float>(rx_data[6]);
}

void Motor::SetPosition(float target_position, float feedforward_speed, float feedforward_intensity) {
    control_method_ = POSITION_SPEED;
    target_angle_ = target_position;
    feedforward_speed_ = feedforward_speed;
    feedforward_intensity_ = feedforward_intensity;

    // 重置PID控制器状态
    ppid_.reset();
    spid_.reset();
}

void Motor::SetSpeed(float target_speed, float feedforward_intensity) {
    control_method_ = SPEED;
    target_speed_ = target_speed;
    feedforward_intensity_ = feedforward_intensity;

    // 重置速度环PID
    spid_.reset();
}

void Motor::SetIntensity(float intensity) {
    control_method_ = TORQUE;
    output_intensity_ = intensity;
}


void Motor::handle() {
    // 获取当前反馈值（需要根据实际电机接口实现）
    fdb_angle_ = getCurrentAngle();// 获取当前角度
    fdb_speed_ = getCurrentSpeed();// 获取当前速度

    switch (control_method_) {
        case TORQUE: {
            // 直接扭矩控制
            setCurrent(output_intensity_);
            break;
        }

        case SPEED: {
            // 速度单环控制
            float speed_output = spid_.calc(target_speed_, fdb_speed_);
            float total_current = speed_output + feedforward_intensity_;
            setCurrent(total_current);
            break;
        }

        case POSITION_SPEED: {
            // 位置-速度双环控制
            // 外环：位置环计算目标速度
            float target_speed_from_position = ppid_.calc(target_angle_, fdb_angle_);

            // 内环：速度环计算输出电流
            float total_target_speed = target_speed_from_position + feedforward_speed_;
            float speed_output = spid_.calc(total_target_speed, fdb_speed_);
            float total_current = speed_output + feedforward_intensity_;

            setCurrent(total_current);
            break;
        }

        default:
            // 安全处理
            setCurrent(0);
            break;
    }
}


M3508_Motor Motor(3591 / 187.0f);
