//
// Created by chenyincheng on 2025/10/3.
//

class M3508_Motor {
private:
  const float ratio_;           // 电机减速比
  float angle_ = 0.f;           // deg 输出端累计转动角度
  float delta_angle_ = 0.f;     // deg 输出端新转动的角度
  float ecd_angle_ = 0.f;       // deg 当前电机编码器角度
  float last_ecd_angle_ = 0.f;  // deg 上次电机编码器角度
  float delta_ecd_angle_ = 0.f; // deg 编码器端新转动的角度
  float rotate_speed_ = 0.f;    // dps 反馈转子转速
  float current_ = 0.f;         // A 反馈转矩电流
  float temp_ = 0.f;            // °C 反馈电机温度

public:
  explicit M3508_Motor(const float ratio) : ratio_(ratio) {};
  void canRxMsgCallback(const uint8_t rx_data[8]);
};