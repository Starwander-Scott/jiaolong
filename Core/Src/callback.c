//
// Created by chenyincheng on 2025/10/2.
//
#include "math.h"
#include "tim.h"
#include "gpio.h"
#include "usart.h"
#include "main.h"


//void HAL_GPIO_EXTI_Callback(uint16_t,GPIO_Pin){
//    if(GPIO_Pin == BUTTON_Pin){
//        uint32_t arr_value = __HAL_TIM_GET_AUTORELOAD(&htim1) + 1;
//        uint32_t brightness = (__HAL_TIM_GetCompare(&htim1,TIM_CHANNEL_2)+100) % arr_value;
//        __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,brightness);
//
//        }
//    }
//}

extern uint8_t rx_msg[4];

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if(huart == &huart7){
        if(rx_msg[0] == 'R'){
            HAL_GPIO_WritePin(LEDR_GPIO_Port, LEDR_Pin, GPIO_PIN_RESET);
        }
        else if(rx_msg[0] == 'M'){
            HAL_GPIO_WritePin(LEDR_GPIO_Port, LEDR_Pin, GPIO_PIN_SET);
        }
        HAL_UART_Receive_IT(&huart7, rx_msg, 1);
    }
}
