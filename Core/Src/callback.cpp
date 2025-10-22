//
// Created by chenyincheng on 2025/10/2.
//

#include "can.h"
#include "gpio.h"
#include "main.h"
#include "math.h"
#include "motor.h"
#include "tim.h"
#include "usart.h"
// void HAL_GPIO_EXTI_Callback(uint16_t,GPIO_Pin){
//     if(GPIO_Pin == BUTTON_Pin){
//         uint32_t arr_value = __HAL_TIM_GET_AUTORELOAD(&htim1) + 1;
//         uint32_t brightness =
//         (__HAL_TIM_GetCompare(&htim1,TIM_CHANNEL_2)+100) % arr_value;
//         __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,brightness);
//
//         }
//     }
// }

//
//volatile uint8_t stop_flag = 1;// 默认停止
//
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//    // 请将 BUTTON_Pin 替换为你实际的按键引脚名
//    if (GPIO_Pin == BUTTON_Pin) {
//        static uint32_t last_tick = 0;
//        uint32_t now = HAL_GetTick();
//        if (now - last_tick < 50) return;// 简单去抖 50ms
//        last_tick = now;
//        stop_flag ^= 1;// 切换 0/1
//    }
//}


// extern uint8_t rx_msg[4];
extern CAN_RxHeaderTypeDef rx_header;
extern CAN_TxHeaderTypeDef tx_header;
extern uint8_t tx_data[8];
extern uint8_t rx_data[8];
extern Motor Motor;
extern uint32_t can_tx_mail_box_;
extern uint8_t stop_flag;


// 新增：全局目标角度变量（可在其它模块通过 extern 访问/设置）
float target_angle = 45.0f;


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (hcan->Instance == CAN1) {
        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data);
    }
    if (rx_header.StdId == 0x201) {
        Motor.canRxMsgCallback(rx_data);
    }
}

// void HAL_TIM_PeriodElapsedCallback(CAN_HandleTypeDef *htim) {
//   if (htim->Instance == htim6.Instance) {
//     HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &can_tx_mail_box_);
//   }
// }// 修正后的代码
void HAL_TIM_PeriodElapsedCallback(
        TIM_HandleTypeDef *htim) {// 1. 修正函数名和参数类型
    if (htim->Instance ==
        htim6.Instance) {// 2. 现在比较的都是 TIM_TypeDef*，类型正确
        //HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &can_tx_mail_box_);
        if (stop_flag == 1) {
            Motor.handle();
        }
        // 调用 Motor 对象的 handle 方法
    }
}

// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//   if (huart == &huart7) {
//     //        if(rx_msg[0] == 'R'){
//     //            HAL_GPIO_WritePin(LEDR_GPIO_Port, LEDR_Pin,
//     GPIO_PIN_RESET);
//     //        }
//     //        else if(rx_msg[0] == 'M'){
//     //            HAL_GPIO_WritePin(LEDR_GPIO_Port, LEDR_Pin, GPIO_PIN_SET);
//     //        }
//     HAL_UART_Receive_IT(&huart7, rx_msg, 3);
//   }
// }


//Motor.handle(rx_data);
// 修正后的代码
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//    if (huart == &huart7) {
//        Motor.handle(rx_data);  // 调用 Motor 对象的 handle 方法
//        HAL_UART_Receive_IT(&huart7, rx_msg, 3);
//    }
//}
