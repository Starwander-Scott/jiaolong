//
// Created by chenyincheng on 2025/10/3.
//

//这里定义了电机的一些参数，包括电机减速比、电流、转速等
float linearMapping(int in, int in_min, int in_max, float out_min,
                    float out_max) {
    return out_min + (out_max - out_min) * (in - in_min) / (in_max - in_min);
}

class M3508_Motor {
private:
    const float ratio_;          // 电机减速比
    float angle_ = 0.f;          // deg 输出端累计转动角度
    float delta_angle_ = 0.f;    // deg 输出端新转动的角度
    float ecd_angle_ = 0.f;      // deg 当前电机编码器角度
    float last_ecd_angle_ = 0.f; // deg 上次电机编码器角度
    float delta_ecd_angle_ = 0.f;// deg 编码器端新转动的角度
    float rotate_speed_ = 0.f;   // dps 反馈转子转速
    float current_ = 0.f;        // A 反馈转矩电流
    float temp_ = 0.f;           // °C 反馈电机温度

public:
    explicit M3508_Motor(const float ratio) : ratio_(ratio) {};
    void canRxMsgCallback(const uint8_t rx_data[8]);
};

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

M3508_Motor Motor(3591 / 187.0f);