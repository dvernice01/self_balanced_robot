#include <stdint.h>
#include <stdlib.h>
#include "stm32f4xx_hal.h"
#include "motors.h"

int16_t SaturatedSpeed(int16_t speed) {
    if (speed > MAX_SPEED) {
        speed = MAX_SPEED;
    } else if (speed < 0) {
        speed = 0;
    }
    return speed;
}

void MotorSpeed(int16_t speed, TIM_HandleTypeDef *hdl, uint32_t channel_pos, uint32_t channel_neg) {
    int16_t speed_saturated = SaturatedSpeed(abs(speed));
        uint32_t ccr = (speed_saturated * (ARR + 1)) / MAX_SPEED;
    if (speed > 0) {
        __HAL_TIM_SET_COMPARE(hdl, channel_pos, ccr);
        __HAL_TIM_SET_COMPARE(hdl, channel_neg, 0);
    } else if (speed < 0) {
        __HAL_TIM_SET_COMPARE(hdl, channel_pos, 0);
        __HAL_TIM_SET_COMPARE(hdl, channel_neg, ccr);
    } else {
        __HAL_TIM_SET_COMPARE(hdl, channel_pos, 0);
        __HAL_TIM_SET_COMPARE(hdl, channel_neg, 0);
    }
}
