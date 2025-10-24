
#include "motor.h"
#include "pid.h"
#include "stdint.h"
#include <math.h>

#include "can.h"// 新增：确保 CAN 类型可用并包含 HAL 声明
extern uint8_t stop_flag;
extern CAN_HandleTypeDef hcan1;
extern CAN_TxHeaderTypeDef tx_header;
extern uint8_t tx_data[8];
extern uint32_t can_tx_mail_box_;
extern uint8_t tx_data[8];


float linearMapping(int in, int in_min, int in_max, float out_min,
                    float out_max) {
    return out_min + (out_max - out_min) * (in - in_min) / (in_max - in_min);
}

float linearMapping(float in, float in_min, float in_max, float out_min,
                    float out_max) {
    return out_min + (out_max - out_min) * (in - in_min) / (in_max - in_min);
}

Motor::Motor(float reduction_ratio) :

                                      ratio_(reduction_ratio),
                                      angle_(0), last_ecd_angle_(0), delta_ecd_angle_(0), delta_angle_(0),
                                      current_(0), temp_(0),
                                      ecd_angle_(0), rotate_speed_(0),


                                      //spid_(0.5f, 0.1f, 1.00f, 1000.0f, 10000.0f, 0.6f),
                                      //ppid_(0.0f, 0.00f, 0.00f, 500.0f, 8000.0f, 0.9f),

                                      target_angle_(0), fdb_angle_(0),
                                      target_speed_(0), fdb_speed_(0), feedforward_speed_(0),
                                      feedforward_intensity_(0), output_intensity_(0),
                                      control_method_(TORQUE) {
}

PID Motor::spid_ = PID(0.5f, 0.1f, 1.0f, 1000.0f, 10000.0f, 0.6f);

PID Motor::ppid_ = PID(0.0f, 0.00f, 0.00f, 500.0f, 8000.0f, 0.9f);
// 获取当前角度（输出轴角度）
float Motor::getCurrentAngle() {
    // 直接返回已经计算好的输出轴累计角度
    return angle_;
}

float Motor::getCurrentSpeed() {
    return rotate_speed_;
}


// 在 Motor 类中添加一个辅助函数用于角度归一化
float Motor::normalizeAngle(float angle) {
    angle = fmod(angle, 360.0f);
    if (angle < 0) {
        angle += 360.0f;
    }
    return angle;
}

// 设置电机电流
void Motor::setCurrent(float current) {
    res1 = current;
    flag1 = 1.0f;

    output_intensity_ = current;

    res2 = output_intensity_;

    // 转换为电机驱动器能识别的格式（-16384~16384对应-20A~20A）
    current_raw_ = static_cast<int16_t>(output_intensity_);

    res3 = current_raw_;

    // 修改：打包 CAN 报文并发送（按照常见协议把电流放入 data[0..1]，大端）
    tx_data[0] = static_cast<uint8_t>((current_raw_ >> 8) & 0xFF);
    tx_data[1] = static_cast<uint8_t>((current_raw_) & 0xFF);
}


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
}

void Motor::SetSpeed(float target_speed, float feedforward_intensity) {
    control_method_ = SPEED;
    target_speed_ = target_speed;
    feedforward_intensity_ = feedforward_intensity;
}

void Motor::SetIntensity(float intensity) {
    control_method_ = TORQUE;
    output_intensity_ = intensity;
}


void Motor::Motor_Stop() {
    // 停止电机，设置电流为0
    setCurrent(0.0f);        // 停止电机
    control_method_ = TORQUE;// 切换到扭矩控制
}


void Motor::handle() {
    // 获取当前反馈值（需要根据实际电机接口实现）
    flag3 = 3.0f;

    fdb_angle_ = getCurrentAngle();// 获取当前角度
    fdb_speed_ = getCurrentSpeed();// 获取当前速度

    angle_ = normalizeAngle(angle_);// 归一化累计角度

    gravity_ff = FeedforwardIntensityCalc(angle_);
    control_method_ = SPEED;// 测试时强制速度控制
    target_speed_ = 300.f;  // 测试时目标速度为0


    if (stop_flag == 0) {
        Motor_Stop();
    }

    switch (control_method_) {
        case TORQUE: {
            // 直接扭矩控制
            float total_intensity = gravity_ff;
            setCurrent(total_intensity);
            break;
        }

        case SPEED: {
            // 速度单环控制
            float speed_output = spid_.calc(target_speed_, fdb_speed_);
            feedforward_intensity_ = gravity_ff;
            //feedforward_intensity_ = 0.0f;
            float total_current = speed_output + feedforward_intensity_;
            setCurrent(total_current);
            break;
        }

        case POSITION_SPEED: {

            float target_speed_from_position = ppid_.calc(target_angle_, fdb_angle_);

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
    //feedforward_intensity_ = 0.5*9.8*sin(angle_/180*3.14159265358979323846)*0.05524/0.3*16384/20;
    flag2 = 2.0f;
    const float mass = 0.5f;            // kg
    const float lever = 0.05524f;       // m (55.24 mm)
    const float g = 9.80665f;           // m/s^2
    const float K_T = 0.3f;             // Nm/A，来自 3 N·m / 10 A
    const float MAX_CURRENT = 10.0f;    // A，额定持续电流
    const float MIN_HOLD_CURRENT = 0.1f;// A，静摩擦补偿（可调）

    const float PI = 3.14159265358979323846f;
    float rad = current_angle * PI / 180.0f;

    float torque_out = mass * g * lever * std::sinf(rad) * 16384 / 20;
    float current = torque_out / K_T;

    return current;
}


Motor Motor(3591 / 187.0f);
