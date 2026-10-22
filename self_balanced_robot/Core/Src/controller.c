#include "main.h"
#include "controller.h"
#include "motors.h"
#include <stdint.h>

// --- PID Parameters ---
float Kp_a = 0.0f, Ki_a = 0.0f, Kd_a = 0.0f; // PID Angle
float Kp_v = 0.0f, Ki_v = 0.0f, Kd_v = 0.0f; // PID Velocity

static uint8_t loop_divider = 0;
static float vel_target = 0.0f;
float current_angle = 0.0f;
float target_angle = 0.0f;

static float angle_integral   = 0.0f;
static float angle_derivative = 0.0f;
static float angle_prev_err   = 0.0f;

// Right motor
static float vel_integral_right   = 0.0f;
static float vel_derivative_right = 0.0f;
static float vel_prev_err_right   = 0.0f;
static float filtered_vel_right   = 0.0f;

// Left motor
static float vel_integral_left    = 0.0f;
static float vel_derivative_left  = 0.0f;
static float vel_prev_err_left    = 0.0f;
static float filtered_vel_left    = 0.0f;

static float alpha = 0.2f;

extern float current_angle;

void PIDController(TIM_HandleTypeDef *htim_enc_right, TIM_HandleTypeDef *htim_enc_left, TIM_HandleTypeDef *htim_pwm_right, TIM_HandleTypeDef *htim_pwm_left) {

    loop_divider++;

    // --- HIGH LEVEL CONTROL (Angle control 20Hz) ---
    if (loop_divider >= 10) {
        loop_divider = 0;
        float angle_err = target_angle - current_angle;
        vel_target = AngleController(angle_err);
    }

    // --- LOW LEVEL CONTROL (Velocity control 200Hz) ---

    // Right Motor
    int16_t right_ticks = (int16_t)__HAL_TIM_GET_COUNTER(htim_enc_right);
    __HAL_TIM_SET_COUNTER(htim_enc_right, 0);

    float current_vel_right = (float)right_ticks;
    filtered_vel_right = (alpha * current_vel_right) + ((1.0f - alpha) * filtered_vel_right);

    float vel_err_right = vel_target - filtered_vel_right;
    vel_integral_right += vel_err_right;
    vel_derivative_right = vel_err_right - vel_prev_err_right;
    vel_prev_err_right = vel_err_right;

    float command_right = (vel_err_right * Kp_v) + (vel_integral_right * Ki_v) + (vel_derivative_right * Kd_v); // Corretto segno "*"
    MotorSpeed((int16_t)command_right, htim_pwm_right, TIM_CHANNEL_1, TIM_CHANNEL_2);

    // Left Motor
    int16_t left_ticks = (int16_t)__HAL_TIM_GET_COUNTER(htim_enc_left);
    __HAL_TIM_SET_COUNTER(htim_enc_left, 0);

    float current_vel_left = (float)left_ticks;
    filtered_vel_left = (alpha * current_vel_left) + ((1.0f - alpha) * filtered_vel_left);

    float vel_err_left = vel_target - filtered_vel_left;
    vel_integral_left += vel_err_left;
    vel_derivative_left = vel_err_left - vel_prev_err_left;
    vel_prev_err_left = vel_err_left;

    float command_left = (vel_err_left * Kp_v) + (vel_integral_left * Ki_v) + (vel_derivative_left * Kd_v);
    MotorSpeed((int16_t)command_left, htim_pwm_left, TIM_CHANNEL_3, TIM_CHANNEL_4);
}

float AngleController(float err) {
    angle_integral += err;
    angle_derivative = err - angle_prev_err;
    angle_prev_err = err;

    float command = (err * Kp_a) + (angle_integral * Ki_a) + (angle_derivative * Kd_a);

    return command;
}
