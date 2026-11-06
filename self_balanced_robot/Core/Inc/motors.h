#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

#define MAX_SPEED       7999
#define ARR             7999
#define DEADBAND_PWM    4000 // min value to make motors moving

int16_t SaturatedSpeed(int16_t speed);
void    MotorSpeed(int16_t speed, TIM_HandleTypeDef *hdl, uint32_t channel_pos, uint32_t channel_neg);

#endif /* INC_MOTORS_H_ */
