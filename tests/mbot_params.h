#include "mbot.h"

// Hardware info
#define WHEEL_RADIUS 0.042
#define GEAR_RATIO 78.0
#define ENCODER_RES 20.0
#define WHEEL_BASE 0.165 // wheel separation distance in meters
#define MAX_FWD_VEL 0.8 // max forward speed (m/s)
#define MESSAGE_CONFIRMATION_CHANNEL "MSG_CONFIRM"
#define MAX_TURN_VEL 2.5 // max turning speed (rad/s)

#define LED_PIN 25
#define MAIN_LOOP_HZ 25.0 // 50 hz loop
#define MAIN_LOOP_PERIOD (1.0f / MAIN_LOOP_HZ)

#define LEFT_MOTOR_CHANNEL 1
#define RIGHT_MOTOR_CHANNEL 3

#define LEFT_ENC_POL 1
#define RIGHT_ENC_POL -1
#define LEFT_MOTOR_POL -1
#define RIGHT_MOTOR_POL 1

// TODO: Populate with calibration data (recommended to generate these for reverse direction as well)
#define SLOPE_L 1.39
#define SLOPE_R 1.45
#define SLOPE_L_REVERSE 1.46
#define SLOPE_R_REVERSE 1.49

#define INTERCEPT_L 0.117
#define INTERCEPT_R 0.145
#define INTERCEPT_L_REVERSE -0.117
#define INTERCEPT_R_REVERSE -0.124

// TODO: Decide which controller is used, open loop = 1, PID = 0
#define OPEN_LOOP 0

#define GYRODOMETRY 1

// TODO: define LPF time constant
#define LPF_TC MAIN_LOOP_PERIOD*3

// TODO: find out threshold through experimentation
#define THRESHOLD 0.005

#define PI 3.14159265

typedef struct pid_parameters pid_parameters_t;
struct pid_parameters
{
    float kp;
    float ki;
    float kd;
    float dFilterHz;
};

/**
 * Example filter and PID parameter initialization
 *
 * rc_filter_t my_filter;
 *
 * pid_parameters_t pid_params = {
 *    .kp = 1.0,
 *    .ki = 0.0,
 *    .kd = 0.0,
 *    .dFilterHz = 25.0
 * };
 */

rc_filter_t left_pid;
rc_filter_t right_pid;
rc_filter_t fwd_vel_pid;
rc_filter_t turn_vel_pid;

rc_filter_t left_i_pid;
rc_filter_t right_i_pid;
rc_filter_t fwd_vel_i_pid;
rc_filter_t turn_vel_i_pid;

pid_parameters_t left_pid_params = {
    .kp = 1.8,
    .ki = 0.4, // 50
    .kd = 0.001,
    .dFilterHz = 20.0,
};
pid_parameters_t right_pid_params = {
    .kp = 1.8,
    .ki = 0.4, // 50
    .kd = 0.001,
    .dFilterHz = 20.0,
};
pid_parameters_t fwd_vel_pid_params = {
    .kp = 1.1,
    .ki = 0.05,
    .kd = 0.0,
    .dFilterHz = 20.0,
};
pid_parameters_t turn_vel_pid_params = {
    .kp = 0.3,
    .ki = 0.0, // 1.5
    .kd = 0.0,
    .dFilterHz = 20.0,
};

pid_parameters_t left_i_pid_params = {
    .kp = 0,
    .ki = 0.4, // 50
    .kd = 0.00,
    .dFilterHz = 20.0,
};
pid_parameters_t right_i_pid_params = {
    .kp = 0,
    .ki = 0.4, // 50
    .kd = 0.00,
    .dFilterHz = 20.0,
};
pid_parameters_t fwd_vel_i_pid_params = {
    .kp = 0,
    .ki = 0.05,
    .kd = 0.0,
    .dFilterHz = 20.0,
};
pid_parameters_t turn_vel_i_pid_params = {
    .kp = 0,
    .ki = 0.0, // 1.5
    .kd = 0.0,
    .dFilterHz = 20.0,
};

rc_filter_t left_lpf;
rc_filter_t right_lpf;
rc_filter_t left_encoder_lpf;
rc_filter_t right_encoder_lpf;
rc_filter_t left_encoder_vel_lpf;
rc_filter_t right_encoder_vel_lpf;
rc_filter_t fwd_vel_lpf;
rc_filter_t turn_vel_lpf;