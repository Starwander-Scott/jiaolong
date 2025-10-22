//
// Created by chenyincheng on 2025/10/18.
//

#include "motor.h"
#include "pid.h"
#include "stdint.h"
#include <math.h>

#include "can.h"// 新增：确保 CAN 类型可用并包含 HAL 声明
extern uint8_t stop_flag;
// 声明在其它文件（例如 main.c / callback.cpp）定义的全局 CAN 相关变量
extern CAN_HandleTypeDef hcan1;
extern CAN_TxHeaderTypeDef tx_header;
extern uint8_t tx_data[8];
extern uint32_t can_tx_mail_box_;


float linearMapping(int in, int in_min, int in_max, float out_min,
                    float out_max) {
    return out_min + (out_max - out_min) * (in - in_min) / (in_max - in_min);
}

Motor::Motor(float reduction_ratio)
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

// 获取当前角度（输出轴角度）
float Motor::getCurrentAngle() {
    // 直接返回已经计算好的输出轴累计角度
    return angle_;
}

// 获取当前速度（输出轴速度，单位dps）
float Motor::getCurrentSpeed() {
    // 将转子转速(rpm)转换为输出轴速度(dps)
    // rpm转dps: rpm * 360 / 60 = rpm * 6
    // 考虑减速比: 转子转速 / 减速比 = 输出轴转速
    return (rotate_speed_ / ratio_) * 6.0f;
}

// 设置电机电流
void Motor::setCurrent(float current) {
    // 电流限幅保护
    control_method_ = TORQUE;
    const float MAX_CURRENT = 20.0f;// 最大电流20A

    if (current > MAX_CURRENT) {
        current = MAX_CURRENT;
    } else if (current < -MAX_CURRENT) {
        current = -MAX_CURRENT;
    }

    output_intensity_ = current;

    // 转换为电机驱动器能识别的格式（-16384~16384对应-20A~20A）
    int16_t current_raw = static_cast<int16_t>(linearMapping(current, -20.0f, 20.0f, -16384.0f, 16384.0f));

    // 发送电流指令
    //sendCurrentToMotor(current_raw);

    // 修改：打包 CAN 报文并发送（按照常见协议把电流放入 data[0..1]，大端）
    tx_data[0] = static_cast<uint8_t>((current_raw >> 8) & 0xFF);
    tx_data[1] = static_cast<uint8_t>((current_raw) & 0xFF);
    // 其余字节可以按协议填充或保留为0
    // 发送电流指令
    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &can_tx_mail_box_);
}

// CAN发送电流指令（需要根据实际硬件实现）
//void Motor::sendCurrentToMotor(int16_t current) {
// 这里需要根据您的CAN协议实现
// 示例：将电流值打包并发送给电机驱动器
// uint8_t data[8] = {...};
// CAN_Send(motor_id, data);
//}


void Motor::canRxMsgCallback(const uint8_t rx_data[8]) {
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
//这个setIntensity函数不用了，用handle里面直接用output_intensity_就行


void Motor::Motor_Stop() {
    // 停止电机，设置电流为0
    setCurrent(0.0f);        // 停止电机
    control_method_ = TORQUE;// 切换到扭矩控制
}


void Motor::handle() {
    // 获取当前反馈值（需要根据实际电机接口实现）
    fdb_angle_ = getCurrentAngle();// 获取当前角度
    fdb_speed_ = getCurrentSpeed();// 获取当前速度

    if (stop_flag == 1) {
        Motor_Stop();
    }

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

float Motor::FeedforwardIntensityCalc(float current_angle) {
    // 常量（依据图片规格）
    const float mass = 0.5f;            // kg
    const float lever = 0.05524f;       // m (55.24 mm)
    const float g = 9.80665f;           // m/s^2
    const float K_T = 0.3f;             // Nm/A，来自 3 N·m / 10 A
    const float MAX_CURRENT = 10.0f;    // A，额定持续电流
    const float MIN_HOLD_CURRENT = 0.1f;// A，静摩擦补偿（可调）


    // 角度转弧度
    const float PI = 3.14159265358979323846f;
    float rad = current_angle * PI / 180.0f;

    // 输出轴重力矩（臂长度 lever）
    float torque_out = mass * g * lever * std::sinf(rad);// Nm

    //    // 电机侧所需转矩（使用对象的减速比 ratio_）
    //    float torque_motor = torque_out / ratio_;// Nm
    //根据同学的要求，大疆电机的K_T已经包含这个减速比的影响了，所以不需要再除以ratio_


    // 转换为电流（可正负）
    float current = torque_out / K_T;// A

    //    // 最低保持电流阈值以克服静摩擦
    //    if (std::fabs(current) > 0.0f && std::fabs(current) < MIN_HOLD_CURRENT) {
    //        current = (current > 0.0f) ? MIN_HOLD_CURRENT : -MIN_HOLD_CURRENT;
    //    }

    // 限幅到额定电流
    if (current > MAX_CURRENT) current = MAX_CURRENT;
    if (current < -MAX_CURRENT) current = -MAX_CURRENT;

    return current;
}


Motor Motor(3591 / 187.0f);
