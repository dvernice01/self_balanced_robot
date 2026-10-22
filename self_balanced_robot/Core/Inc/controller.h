#ifndef INC_CONTROLLER_H_
#define INC_CONTROLLER_H_

void PIDController(TIM_HandleTypeDef *htim_enc_right, TIM_HandleTypeDef *htim_enc_left, TIM_HandleTypeDef *htim_pwm_right, TIM_HandleTypeDef *htim_pwm_left);
float AngleController(float err);

#endif
