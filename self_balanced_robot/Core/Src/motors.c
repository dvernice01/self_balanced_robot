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

void MotorSpeed(int16_t speed, TIM_HandleTypeDef *hdl, uint32_t ch_pos, uint32_t ch_neg) {
    if (speed == 0) {
        __HAL_TIM_SET_COMPARE(hdl, ch_pos, 0);
        __HAL_TIM_SET_COMPARE(hdl, ch_neg, 0);
        return;
    }

    int32_t mag = abs((int32_t)speed);
    if (mag > MAX_SPEED) mag = MAX_SPEED;

    uint32_t ccr = DEADBAND_PWM +
        ((uint32_t)mag * (MOTOR_PWM_ARR - DEADBAND_PWM)) / MAX_SPEED;

    if (speed > 0) {
        __HAL_TIM_SET_COMPARE(hdl, ch_pos, ccr);
        __HAL_TIM_SET_COMPARE(hdl, ch_neg, 0);
    } else {
        __HAL_TIM_SET_COMPARE(hdl, ch_pos, 0);
        __HAL_TIM_SET_COMPARE(hdl, ch_neg, ccr);
    }
}
