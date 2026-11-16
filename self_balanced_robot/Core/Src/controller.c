#include <math.h>
#include "main.h"
#include "controller.h"
#include "motors.h"
#include <stdint.h>

#define PWM_MAX        7500.0f
#define KFF_LEFT       450.0f
#define KFF_RIGHT      480.0f
#define VEL_MAX        15.0f
#define I_V_LIM        (PWM_MAX * 0.5f)
#define I_A_LIM        (VEL_MAX * 0.5f)
#define ANGLE_CUTOFF   30.0f
#define ANGLE_DIVIDER  5

extern volatile float   current_angle;
extern volatile float   gyro_rate;
extern volatile uint8_t stop_request;
volatile float   vel_target;

extern volatile uint32_t last_sensor_ms;   // aggiornata ogni volta che arriva un dato valido

float target_angle = 4.86f;
float Kp_v = 150.0f, Ki_v = 3.0, Kd_v = 0.0;
float Kp_a = 1.0, Ki_a = 0.0, Kd_a = 0.05;
float alpha = 0.3;

volatile int32_t trace_vel_left, trace_vel_right, trace_cmd_left, trace_filtered;

static uint8_t loop_divider = 0;
static float angle_integral = 0.0f;
static float vel_integral_left = 0.0f, vel_integral_right = 0.0f;
static float filtered_vel_left = 0.0f, filtered_vel_right = 0.0f;
static float prev_meas_left = 0.0f,   prev_meas_right = 0.0f;

static inline float clampf(float x, float lo, float hi) {
    return (x < lo) ? lo : (x > hi) ? hi : x;
}

static void ResetControllers(void) {
    angle_integral = 0.0f;
    vel_integral_left = vel_integral_right = 0.0f;
    filtered_vel_left = filtered_vel_right = 0.0f;
    prev_meas_left = prev_meas_right = 0.0f;
    vel_target = 0.0f;
    loop_divider = 0;
}

static float AngleController(float err, float gyro) {
    angle_integral = clampf(angle_integral + Ki_a * err, -I_A_LIM, I_A_LIM);
    float u = Kp_a * err + angle_integral + Kd_a * gyro;
    return clampf(u, -VEL_MAX, VEL_MAX);
}

static float VelocityPID(float target, float meas, float *integral, float *prev_meas, float KFF) {
    float err = target - meas;
    *integral = clampf(*integral + Ki_v * err, -I_V_LIM, I_V_LIM);
    float d = -(meas - *prev_meas);
    *prev_meas = meas;

    float u = target * KFF + Kp_v * err + *integral + Kd_v * d;
    return clampf(u, -PWM_MAX, PWM_MAX);
}

void PIDController(TIM_HandleTypeDef *htim_enc_right, TIM_HandleTypeDef *htim_enc_left,
                   TIM_HandleTypeDef *htim_pwm_right, TIM_HandleTypeDef *htim_pwm_left)
{

	if (HAL_GetTick() - last_sensor_ms > 30) {
	    stop_request = 1;     // sensore morto: spegni i motori
	}
	//vel_target = 10.0f;
    float angle_err = current_angle - target_angle;

    // --- fallen robot ---
    if (fabsf(angle_err) > ANGLE_CUTOFF) {
        stop_request = 1;
    }

    // --- stop ---
    if (stop_request == 1) {
        ResetControllers();
        __HAL_TIM_SET_COUNTER(htim_enc_right, 0);
        __HAL_TIM_SET_COUNTER(htim_enc_left, 0);
        MotorSpeed(0, htim_pwm_left,  TIM_CHANNEL_1, TIM_CHANNEL_2);
        MotorSpeed(0, htim_pwm_right, TIM_CHANNEL_1, TIM_CHANNEL_2);
        return;
    }

    // --- HIGH LEVEL: angle 100 Hz ---
    if (++loop_divider >= ANGLE_DIVIDER) {
        loop_divider = 0;
        vel_target = -AngleController(angle_err, gyro_rate);
    }

    // --- LOW LEVEL: velocity 500 Hz ---
    // Right
    int16_t right_ticks = (int16_t)__HAL_TIM_GET_COUNTER(htim_enc_right);
    __HAL_TIM_SET_COUNTER(htim_enc_right, 0);
    filtered_vel_right = alpha * (float)right_ticks + (1.0f - alpha) * filtered_vel_right;

    float cmd_right = VelocityPID(vel_target, filtered_vel_right,
                                  &vel_integral_right, &prev_meas_right, KFF_RIGHT);
    MotorSpeed((int16_t)cmd_right, htim_pwm_right, TIM_CHANNEL_1, TIM_CHANNEL_2);

    // Left
    int16_t left_ticks = (int16_t)__HAL_TIM_GET_COUNTER(htim_enc_left);
    __HAL_TIM_SET_COUNTER(htim_enc_left, 0);
    filtered_vel_left = alpha * (float)left_ticks + (1.0f - alpha) * filtered_vel_left;

    float cmd_left = VelocityPID(vel_target, filtered_vel_left,
                                 &vel_integral_left, &prev_meas_left, KFF_LEFT);
    MotorSpeed((int16_t)cmd_left, htim_pwm_left, TIM_CHANNEL_1, TIM_CHANNEL_2);

    trace_cmd_left   = (int32_t)cmd_left;
    trace_filtered   = (int32_t)filtered_vel_right;
    trace_vel_left   = left_ticks;
    trace_vel_right  = right_ticks;
    //trace_angle      = (int32_t)current_angle;
}
