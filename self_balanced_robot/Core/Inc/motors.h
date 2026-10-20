#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

#define MAX_SPEED       300
#define ARR             2099

int16_t SaturatedSpeed(int16_t speed);
void    MotorSpeed(int16_t speed, TIM_HandleTypeDef *hdl, uint32_t channel_pos, uint32_t channel_neg);

#endif /* INC_MOTORS_H_ */
