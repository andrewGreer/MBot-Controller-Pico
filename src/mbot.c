/**
 * This file is the main executable for the MBot firmware.
 */

#include <pico/stdlib.h>
#include <pico/mutex.h>
#include <pico/multicore.h>
#include <pico/time.h>
#include <rc/motor/motor.h>
#include <rc/encoder/encoder.h>
#include <rc/motor/motor.h>
#include <rc/mpu/mpu.h>
#include <rc/fram/fram.h>
#include <comms/common.h>
#include <comms/protocol.h>
#include <comms/listener.h>
#include <comms/topic_data.h>
#include <comms/messages_mb.h>

#include <math.h>
#include <inttypes.h>
#include "mbot_params.h"

// data to hold current mpu state
static mb_mpu_data_t mpu_data;

uint64_t timestamp_offset = 0;
uint64_t current_pico_time = 0;

static float theta_prime_gyro = 0.0;
static float theta_prime_odom = 0.0;

static float current_theta_odom = 0.0;
static float current_theta_gyro = 0.0;
static float prev_theta_odom = 0.0;
static float delta_g_o = 0.0;

static float prev_theta_gyro = 0.0;

float enc2meters = ((2.0 * PI * WHEEL_RADIUS) / (GEAR_RATIO * ENCODER_RES));

static i2c_inst_t *i2c = i2c0;

void timestamp_cb(timestamp_t *received_timestamp)
{
    // if we havent set the offset yet
    if (timestamp_offset == 0)
    {
        uint64_t cur_pico_time = to_us_since_boot(get_absolute_time());
        timestamp_offset = received_timestamp->utime - cur_pico_time;
    }
}

void reset_encoders_cb(mbot_encoder_t *received_encoder_vals)
{
    rc_encoder_write(LEFT_MOTOR_CHANNEL, received_encoder_vals->leftticks);
    rc_encoder_write(RIGHT_MOTOR_CHANNEL, received_encoder_vals->rightticks);
}

void reset_odometry_cb(odometry_t *received_odom)
{
    current_odom.utime = received_odom->utime;
    current_odom.x = received_odom->x;
    current_odom.y = received_odom->y;
    current_odom.theta = received_odom->theta;
}

int write_pid_coefficients()
{
    uint8_t pid_bytes[PID_VALUES_LEN];
    memcpy(pid_bytes, &mbot_pid_gains, PID_VALUES_LEN);
    return mb_write_fram(i2c, PID_VALUES_ADDR, PID_VALUES_LEN, &pid_bytes[0]);
}

void pid_values_cb(mbot_pid_gains_t *received_pid_gains)
{
    memcpy(&mbot_pid_gains, received_pid_gains, sizeof(mbot_pid_gains_t));
    write_pid_coefficients();
}

void register_topics()
{
    // timesync topic
    comms_register_topic(MBOT_TIMESYNC, sizeof(timestamp_t), (Deserialize)&timestamp_t_deserialize, (Serialize)&timestamp_t_serialize, (MsgCb)&timestamp_cb);
    // odometry topic
    comms_register_topic(ODOMETRY, sizeof(odometry_t), (Deserialize)&odometry_t_deserialize, (Serialize)&odometry_t_serialize, NULL);
    // reset odometry topic
    comms_register_topic(RESET_ODOMETRY, sizeof(odometry_t), (Deserialize)&odometry_t_deserialize, (Serialize)&odometry_t_serialize, (MsgCb)&reset_odometry_cb);
    // IMU topic
    comms_register_topic(MBOT_IMU, sizeof(mbot_imu_t), (Deserialize)&mbot_imu_t_deserialize, (Serialize)&mbot_imu_t_serialize, NULL);
    // encoders topic
    comms_register_topic(MBOT_ENCODERS, sizeof(mbot_encoder_t), (Deserialize)&mbot_encoder_t_deserialize, (Serialize)&mbot_encoder_t_serialize, NULL);
    // reset encoders topic
    comms_register_topic(RESET_ENCODERS, sizeof(mbot_encoder_t), (Deserialize)&mbot_encoder_t_deserialize, (Serialize)&mbot_encoder_t_serialize, (MsgCb)&reset_encoders_cb);
    // motor commands topic
    comms_register_topic(MBOT_MOTOR_COMMAND, sizeof(mbot_motor_command_t), (Deserialize)&mbot_motor_command_t_deserialize, (Serialize)&mbot_motor_command_t_serialize, NULL);
    // PID values topic
    comms_register_topic(MBOT_PIDS, sizeof(mbot_pid_gains_t), (Deserialize)&mbot_pid_gains_t_deserialize, (Serialize)&mbot_pid_gains_t_serialize, (MsgCb)&pid_values_cb);
}

void read_pid_coefficients(i2c_inst_t *i2c)
{
    uint8_t pid_bytes[PID_VALUES_LEN];

    if (mb_read_fram(i2c, PID_VALUES_ADDR, PID_VALUES_LEN, pid_bytes) > 0)
    {
        printf("reading fram success.\n");
        memcpy(&mbot_pid_gains, pid_bytes, PID_VALUES_LEN);
        printf("read gains from fram!\r\n");
    }
    else
    {
        printf("reading PID gains from fram failure.\n");
    }
}

bool timer_cb(repeating_timer_t *rt)
{
    // Read the PID values
    // if (comms_get_topic_data(MBOT_PIDS, &mbot_pid_gains))
    // {
    //     uint64_t msg_time = current_pico_time;
    //     // Print the PID values

    //     // update the PID gains of the left motor PID controller
    //     rc_filter_pid(&left_pid,
    //                   mbot_pid_gains.motor_a_kp,
    //                   mbot_pid_gains.motor_a_ki,
    //                   mbot_pid_gains.motor_a_kd,
    //                   1.0 / MAIN_LOOP_HZ,
    //                   1.0 / MAIN_LOOP_HZ);

    //     // update the PID gains of the right motor PID controller
    //     rc_filter_pid(&right_pid,
    //                   mbot_pid_gains.motor_c_kp,
    //                   mbot_pid_gains.motor_c_ki,
    //                   mbot_pid_gains.motor_c_kd,
    //                   1.0 / MAIN_LOOP_HZ,
    //                   1.0 / MAIN_LOOP_HZ);

    //     // update the PID gains of the body frame controller
    //     rc_filter_pid(&fwd_vel_pid,
    //                   mbot_pid_gains.bf_trans_kp,
    //                   mbot_pid_gains.bf_trans_ki,
    //                   mbot_pid_gains.bf_trans_kd,
    //                   1.0 / MAIN_LOOP_HZ,
    //                   1.0 / MAIN_LOOP_HZ);

    //     // update the PID gains of the body frame rotation controller
    //     rc_filter_pid(&turn_vel_pid,
    //                   mbot_pid_gains.bf_rot_kp,
    //                   mbot_pid_gains.bf_rot_ki,
    //                   mbot_pid_gains.bf_rot_kd,
    //                   1.0 / MAIN_LOOP_HZ, // 1.0 / mbot_pid_gains.bf_rot_Tf
    //                   1.0 / MAIN_LOOP_HZ);

    //     // printf("Left: %f, %f, %f, %f\r\n", mbot_pid_gains.motor_a_kp,
    //     //        mbot_pid_gains.motor_a_ki,
    //     //        mbot_pid_gains.motor_a_kd,
    //     //        1.0 / MAIN_LOOP_HZ);
    //     // printf("Right: %f, %f, %f, %f\r\n", mbot_pid_gains.motor_c_kp,
    //     //        mbot_pid_gains.motor_c_ki,
    //     //        mbot_pid_gains.motor_c_kd,
    //     //        1.0 / MAIN_LOOP_HZ);
    //     // printf("Fwd: %f, %f, %f, %f\r\n", mbot_pid_gains.bf_trans_kp,
    //     //        mbot_pid_gains.bf_trans_ki,
    //     //        mbot_pid_gains.bf_trans_kd,
    //     //        1.0 / MAIN_LOOP_HZ);
    //     // printf("Turn: %f, %f, %f, %f\r\n", mbot_pid_gains.bf_rot_kp,
    //     //        mbot_pid_gains.bf_rot_ki,
    //     //        mbot_pid_gains.bf_rot_kd,
    //     //        1.0 / MAIN_LOOP_HZ);
        
    // }

    // make up a LPF for our left setpoint controller
    rc_filter_moving_average(&left_lpf, 2, MAIN_LOOP_PERIOD);

    // make up a LPF for our right setpoint controller
    rc_filter_moving_average(&right_lpf, 2, MAIN_LOOP_PERIOD);

    // make up a LPF for our left encoder
    rc_filter_first_order_lowpass(&left_encoder_lpf, MAIN_LOOP_PERIOD, LPF_TC);

    // make up a LPF for our right setpoint controller
    rc_filter_first_order_lowpass(&right_encoder_lpf, MAIN_LOOP_PERIOD, LPF_TC);

    // make up a LPF for our left encoder
    rc_filter_first_order_lowpass(&left_encoder_vel_lpf, MAIN_LOOP_PERIOD, LPF_TC);

    // make up a LPF for our right setpoint controller
    rc_filter_first_order_lowpass(&right_encoder_vel_lpf, MAIN_LOOP_PERIOD, LPF_TC);

    // make up a LPF for our fwd velocity controller
    rc_filter_first_order_lowpass(&fwd_vel_lpf, MAIN_LOOP_PERIOD, LPF_TC);

    // make up a LPF for our rotational velocity controller
    rc_filter_first_order_lowpass(&turn_vel_lpf, MAIN_LOOP_PERIOD, LPF_TC);

    // only run if we've received a timesync message...
    if (comms_get_topic_data(MBOT_TIMESYNC, &received_time))
    {
        comms_get_topic_data(MBOT_IMU, &current_imu);

        uint64_t cur_pico_time = to_us_since_boot(get_absolute_time()) + timestamp_offset;
        uint64_t latency_time = cur_pico_time - current_pico_time;
        current_pico_time = cur_pico_time;
        // first, get the IMU data and send across the wire
        current_imu.utime = cur_pico_time; // received_time.utime;

        // read the encoders
        int enc_cnt_l = LEFT_ENC_POL * rc_encoder_read_count(LEFT_MOTOR_CHANNEL);
        int enc_delta_l = LEFT_ENC_POL * rc_encoder_read_delta(LEFT_MOTOR_CHANNEL);
        int enc_cnt_r = RIGHT_ENC_POL * rc_encoder_read_count(RIGHT_MOTOR_CHANNEL);
        int enc_delta_r = RIGHT_ENC_POL * rc_encoder_read_delta(RIGHT_MOTOR_CHANNEL);
        current_encoders.utime = cur_pico_time; // received_time.utime;
        // current_encoders.right_delta = rc_filter_march(&right_encoder_vel_lpf, enc_delta_r);
        // current_encoders.rightticks = rc_filter_march(&right_encoder_lpf, enc_cnt_r);
        // current_encoders.left_delta = rc_filter_march(&left_encoder_vel_lpf, enc_delta_l);
        // current_encoders.leftticks = rc_filter_march(&left_encoder_lpf, enc_cnt_l);
        current_encoders.right_delta = enc_delta_r;
        current_encoders.rightticks = enc_cnt_r;
        current_encoders.left_delta = enc_delta_l;
        current_encoders.leftticks = enc_cnt_l;

        // compute new odometry
        /*************************************************************
         * TODO:
         *       -SECTION 1.3 ODOMETRY
         *      - Populate the odometry messages.
         *          -The struct for odometry_t is defined in comms/include/comms/messages_mb.h (DO NOT EDIT THIS FILE)
         *      - Note that the way we compute the displacement of the motor is from encoder readings and
         *        that we convert encoder readings to meters using the enc2meters variable defined above.
         *      - Use the equations provided in the document to compute the odometry components
         *      - Remember to clamp the orientation between [0, 2pi]!
         *************************************************************/
        float delta_d, delta_theta_odom; // displacement in meters and rotation in radians
        float delta_theta = 0.0;

        float delta_x, delta_y;
        float s_r = current_encoders.right_delta * enc2meters;
        float s_l = current_encoders.left_delta * enc2meters;
        float theta_prime;

        prev_theta_gyro = current_theta_gyro;
        prev_theta_odom = current_theta_odom;

        delta_theta_odom = (s_r - s_l) / WHEEL_BASE;
        delta_theta = delta_theta_odom;
        theta_prime = current_theta + delta_theta_odom;

        theta_prime = clamp_orientation(theta_prime);
        
        current_theta = theta_prime;
        current_theta_gyro = mpu_data.dmp_TaitBryan[2];
        current_theta_odom = current_theta;

        // TODO: This //////////////////////////////////////////////////
        if (GYRODOMETRY) {
            float delta_g = mpu_data.dmp_TaitBryan[2] - prev_theta_gyro;
            float delta_o = current_theta_odom - prev_theta_odom;
            delta_g_o = delta_g - delta_o;

            // printf("Odometry option: %f\n", current_theta + delta_g * MAIN_LOOP_PERIOD);
            // printf("Gyro option: %f\n", current_theta + delta_o * MAIN_LOOP_PERIOD);
            theta_prime_gyro = current_theta + delta_g * MAIN_LOOP_PERIOD;
            theta_prime_odom = current_theta + delta_o * MAIN_LOOP_PERIOD;
            // theta_prime_gyro = delta_g;
            // theta_prime_odom = delta_o;

            if (fabs(delta_g_o) > THRESHOLD) {
                theta_prime = theta_prime_gyro;
                delta_theta = delta_g * MAIN_LOOP_PERIOD;
            }
            else {
                theta_prime = theta_prime_odom;
                delta_theta = delta_o * MAIN_LOOP_PERIOD;
            }

            theta_prime = clamp_orientation(theta_prime);
            
            current_theta = theta_prime;
        }
        
        delta_d = (s_r + s_l) / 2;

        delta_x = delta_d * cos(current_theta + (delta_theta / 2));
        delta_y = delta_d * sin(current_theta + (delta_theta / 2));

        x_pos += delta_x;
        y_pos += delta_y;

        current_odom.x = x_pos;
        current_odom.y = y_pos;
        current_odom.theta = current_theta;
        current_odom.utime = cur_pico_time;

        // TODO: This //////////////////////////////////////////////////

        /*************************************************************
         * End of TODO
         *************************************************************/

        // get the current motor command state (if we have one)
        if (comms_get_topic_data(MBOT_MOTOR_COMMAND, &current_cmd))
        {
            int16_t l_cmd, r_cmd;                 // left and right motor commands
            float left_sp, right_sp;              // speed in m/s
            float measured_vel_l, measured_vel_r; // measured velocity in m/s
            float l_duty, r_duty;                 // duty cycle in range [-1, 1]
            float dt = MAIN_LOOP_PERIOD;          // time since last update in seconds
            float linear_vel = current_cmd.trans_v;
            float angular_vel = current_cmd.angular_v;

            linear_vel = linear_vel;
            angular_vel = angular_vel;

            if (OPEN_LOOP)
            {
                /*************************************************************
                 * TODO:
                 *      -SECTION 1.2 OPEN LOOP CONTROL
                 *      - Implement the open loop motor controller to compute the left
                 *          and right wheel commands
                 *      - Determine the setpoint velocities for left and right motor using the wheel velocity model
                 ************************************************************/
                left_sp = linear_vel - ((angular_vel * WHEEL_BASE) / 2);
                right_sp = linear_vel + ((angular_vel * WHEEL_BASE) / 2);

                if (linear_vel == 0 && angular_vel == 0) {
                    l_duty = 0.0;
                    r_duty = 0.0;
                }
                else {
                    if (left_sp >= 0) {
                        l_duty = (SLOPE_L * left_sp) + INTERCEPT_L;
                    }
                    else {
                        l_duty = (SLOPE_L_REVERSE * left_sp) + INTERCEPT_L_REVERSE;
                    }
                    if (right_sp >= 0) {
                        r_duty = (SLOPE_R * right_sp) + INTERCEPT_R;
                    }
                    else {
                        r_duty = (SLOPE_R_REVERSE * right_sp) + INTERCEPT_R_REVERSE;
                    }
                }
                /*************************************************************
                 * End of TODO
                 *************************************************************/
            }
            else
            {
                /*************************************************************
                 * TODO:
                 *       -SECTION 1.2 PID CONTROL
                 *      - Implement the closed loop motor controller to compute the left
                 *          and right wheel commands
                 *      - To calculate the measured velocity, use MAIN_LOOP_PERIOD or latency_time
                 *          as the timestep
                 *      - Compute the error between the setpoint velocity and the measured velocity
                 *      - We recommend to use the open loop controller as part of the closed loop to improve
                 *          performance.
                 *          Example: open_loop_control(LEFT_MOTOR_CHANNEL, left_sp)
                 *      - Use the PID filters defined in mbot.h and main() function to calculate desired
                 *          duty, use rc_filter_march() function.
                 *          Example: rc_filter_march(&left_pid, left_error)
                 * TODO:
                        -SECTION 1.5 FRAME VELOCITY CONTROL
                 *      - Compute the error between the target and measured translation+rotation velocity
                 *      - Compute the new forward and rotation setpoints that will be used in
                 *          the wheel speed PID (these should be written in lines above
                 *          the previous controller)
                 *      - Update and the fwd_sp and turn_sp variables for this.
                 *
                 ************************************************************/
                float fwd_sp, turn_sp = 0.0;                     // forward and turn setpoints in m/s and rad/s
                float measured_vel_fwd, measured_vel_turn; // measured forward and turn velocities in m/s and rad/s


                // SECTION 1.5:
                /**
                 *  Example closed loop controller
                 *      (assuming the error between the target and measured is computed)
                 *
                 * float pid_delta_vel = rc_filter_march(&pid_filter, error);
                 * float desired_vel = commanded_val + pid_delta_vel;
                 */                

                if (linear_vel == 0 && angular_vel == 0) {
                    l_duty = 0.0;
                    r_duty = 0.0;
                }
                else {
                    // TODO: finish FF
                    // float ff_left = 0.0;
                    // float ff_right = 0.0;
                    float desired_left = linear_vel - ((angular_vel * WHEEL_BASE) / 2);
                    float desired_right = linear_vel + ((angular_vel * WHEEL_BASE) / 2);
                    // if (desired_left >= 0) {
                    //     ff_left = (SLOPE_L * desired_left) + INTERCEPT_L;
                    // }
                    // else {
                    //     ff_left = (SLOPE_L_REVERSE * desired_left) + INTERCEPT_L_REVERSE;
                    // }
                    // if (desired_right >= 0) {
                    //     ff_right = (SLOPE_R * desired_right) + INTERCEPT_R;
                    // }
                    // else {
                    //     ff_right = (SLOPE_R_REVERSE * desired_right) + INTERCEPT_R_REVERSE;
                    // }

                    measured_vel_l = enc2meters * current_encoders.left_delta / dt;
                    measured_vel_r = enc2meters * current_encoders.right_delta / dt;

                    measured_vel_fwd = (measured_vel_r + measured_vel_l) / 2;
                    measured_vel_turn = (measured_vel_r - measured_vel_l) / WHEEL_BASE;

                    printf("%lld, %f, %f\r\n", cur_pico_time, measured_vel_fwd, measured_vel_turn);

                    float error_fwd = linear_vel - measured_vel_fwd;
                    float error_turn = angular_vel - measured_vel_turn;

                    float pid_delta_vel_fwd = rc_filter_march(&fwd_vel_pid, error_fwd);
                    float pid_delta_vel_fwd_i = rc_filter_march(&fwd_vel_i_pid, error_fwd);
                    fwd_sp = linear_vel + pid_delta_vel_fwd + pid_delta_vel_fwd_i;
                    
                    float pid_delta_vel_turn = rc_filter_march(&turn_vel_pid, error_turn);
                    float pid_delta_vel_turn_i = rc_filter_march(&turn_vel_i_pid, error_turn);
                    turn_sp = angular_vel + pid_delta_vel_turn + pid_delta_vel_turn_i;

                    float left_sp_original = linear_vel - ((angular_vel * WHEEL_BASE) / 2);
                    float right_sp_original = linear_vel + ((angular_vel * WHEEL_BASE) / 2);
                    // left_sp = fwd_sp - ((turn_sp * WHEEL_BASE) / 2);
                    // right_sp = fwd_sp + ((turn_sp * WHEEL_BASE) / 2);
                    left_sp = left_sp_original;
                    right_sp = right_sp_original;

                    float error_l = left_sp - measured_vel_l;
                    float error_r = right_sp - measured_vel_r;

                    float pid_delta_vel_l = rc_filter_march(&left_pid, error_l);
                    float pid_delta_vel_l_i = rc_filter_march(&left_i_pid, error_l);
                    float desired_vel_l = left_sp + pid_delta_vel_l + pid_delta_vel_l_i;
                    
                    float pid_delta_vel_r = rc_filter_march(&right_pid, error_r);
                    float pid_delta_vel_r_i = rc_filter_march(&right_i_pid, error_r);
                    float desired_vel_r = right_sp + pid_delta_vel_r + pid_delta_vel_r_i;

                    if (desired_vel_l >= 0) {
                        l_duty = (SLOPE_L * desired_vel_l) + INTERCEPT_L;
                    }
                    else {
                        l_duty = (SLOPE_L_REVERSE * desired_vel_l) + INTERCEPT_L_REVERSE;
                    }
                    if (desired_vel_r >= 0) {
                        r_duty = (SLOPE_R * desired_vel_r) + INTERCEPT_R;
                    }
                    else {
                        r_duty = (SLOPE_R_REVERSE * desired_vel_r) + INTERCEPT_R_REVERSE;
                    }
                    // l_duty = rc_filter_march(&left_lpf, l_duty);
                    // r_duty = rc_filter_march(&right_lpf, r_duty);
                }

                /*************************************************************
                 * End of TODO
                 *************************************************************/
            }
            // Clamp duty cycle to [-1, 1]
            l_duty = clamp_duty(l_duty);
            r_duty = clamp_duty(r_duty);

            // duty to motor command
            l_cmd = LEFT_MOTOR_POL * (int)(l_duty * 0.95 * pow(2, 15));
            r_cmd = RIGHT_MOTOR_POL * (int)(r_duty * 0.95 * pow(2, 15));

            // set left and right motor command
            rc_motor_set(LEFT_MOTOR_CHANNEL, l_cmd);
            rc_motor_set(RIGHT_MOTOR_CHANNEL, r_cmd);
        }

        // write the encoders to serial
        comms_write_topic(MBOT_ENCODERS, &current_encoders);
        // send odom on wire
        comms_write_topic(ODOMETRY, &current_odom);
        // write the IMU to serial
        comms_write_topic(MBOT_IMU, &current_imu);
        uint64_t fn_run_len = to_us_since_boot(get_absolute_time()) + timestamp_offset - cur_pico_time;
    }

    return true;
}

int main()
{
    bi_decl(bi_program_description("The main binary for the MBot Pico Board."));
    bi_decl(bi_1pin_with_name(LED_PIN, "On-board LED"));
    set_sys_clock_khz(125000, true);

    stdio_init_all();
    sleep_ms(1500); // quick sleep so we can catch the bootup process in terminal
    printf("\nMBot Booting Up!\n");

    printf("initing motors...\n");
    rc_motor_init();
    printf("initing encoders...\n");
    rc_encoder_init();

    // Pins
    // for the i2c to the IMU
    // const uint sda_pin = 4;
    // const uint scl_pin = 5;

    // Ports
    // Initialize I2C port at 400 kHz
    i2c_init(i2c, 400 * 1000);
    // Initialize I2C pins
    printf("setting i2c functions...\n");
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    printf("setting i2c pullups...\n");
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    printf("setting heartbeat LED GPIOs...\n");
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    printf("initializing DMP...\n");
    mb_mpu_config_t mpu_config = mb_mpu_default_config();
    mpu_config.i2c_bus = i2c;
    mpu_config.dmp_fetch_accel_gyro = 1;
    mpu_config.enable_magnetometer = 0;
    mpu_config.read_mag_after_callback = 0;
    mpu_config.orient = ORIENTATION_Z_UP;
    mpu_config.dmp_sample_rate = 200;
    mb_mpu_reset_accel_cal(mpu_config.i2c_bus);
    sleep_ms(100);
    mb_mpu_calibrate_gyro_routine(mpu_config);
    // sleep_ms(2000);
    // mb_mpu_calibrate_accel_routine(mpu_config);
    sleep_ms(500);
    mb_mpu_initialize_dmp(&mpu_data, mpu_config);
    sleep_ms(100);
    gpio_set_irq_enabled_with_callback(MB_MPU_INTERRUPT_GPIO, GPIO_IRQ_EDGE_FALL, true, &mb_dmp_callback);
    printf("MPU Initialized!\n");
    sleep_ms(100);

    // create topics and register the serialize/deserialize functions
    printf("init comms...\r\n");
    comms_init_protocol();
    comms_init_topic_data();
    register_topics();
    sleep_ms(100);

    // launch the other core's comm loop
    printf("starting comms on core 1...\r\n");
    multicore_launch_core1(comms_listener_loop);

    // wait for other core to get rolling
    sleep_ms(500);

    // int running = 1;

    // run the main loop as a timed interrupt
    // printf("starting the timed interrupt...\r\n");
    left_pid = rc_filter_empty();
    rc_filter_pid(&left_pid, left_pid_params.kp, left_pid_params.ki, left_pid_params.kd, 1.0 / MAIN_LOOP_HZ, 1.0 / MAIN_LOOP_HZ);

    right_pid = rc_filter_empty();
    rc_filter_pid(&right_pid, right_pid_params.kp, right_pid_params.ki, right_pid_params.kd, 1.0 / MAIN_LOOP_HZ, 1.0 / MAIN_LOOP_HZ);

    fwd_vel_pid = rc_filter_empty();
    rc_filter_pid(&fwd_vel_pid, fwd_vel_pid_params.kp, fwd_vel_pid_params.ki, fwd_vel_pid_params.kd, 1.0 / MAIN_LOOP_HZ, 1.0 / MAIN_LOOP_HZ);

    turn_vel_pid = rc_filter_empty();
    rc_filter_pid(&turn_vel_pid, turn_vel_pid_params.kp, turn_vel_pid_params.ki, turn_vel_pid_params.kd, 1.0 / MAIN_LOOP_HZ, 1.0 / MAIN_LOOP_HZ);

    left_i_pid = rc_filter_empty();
    rc_filter_pid(&left_i_pid, left_i_pid_params.kp, left_i_pid_params.ki, left_i_pid_params.kd, 1.0 / MAIN_LOOP_HZ, 1.0 / MAIN_LOOP_HZ);
    rc_filter_enable_saturation(&left_i_pid, -0.1, 0.1);

    right_i_pid = rc_filter_empty();
    rc_filter_pid(&right_i_pid, right_i_pid_params.kp, right_i_pid_params.ki, right_i_pid_params.kd, 1.0 / MAIN_LOOP_HZ, 1.0 / MAIN_LOOP_HZ);
    rc_filter_enable_saturation(&right_i_pid, -0.1, 0.1);

    fwd_vel_i_pid = rc_filter_empty();
    rc_filter_pid(&fwd_vel_i_pid, fwd_vel_i_pid_params.kp, fwd_vel_i_pid_params.ki, fwd_vel_i_pid_params.kd, 1.0 / MAIN_LOOP_HZ, 1.0 / MAIN_LOOP_HZ);
    rc_filter_enable_saturation(&fwd_vel_i_pid, -0.1, 0.1);

    turn_vel_i_pid = rc_filter_empty();
    rc_filter_pid(&turn_vel_i_pid, turn_vel_i_pid_params.kp, turn_vel_i_pid_params.ki, turn_vel_i_pid_params.kd, 1.0 / MAIN_LOOP_HZ, 1.0 / MAIN_LOOP_HZ);
    rc_filter_enable_saturation(&turn_vel_i_pid, -0.1, 0.1);

    sleep_ms(100);

    repeating_timer_t loop_timer;
    add_repeating_timer_ms(MAIN_LOOP_PERIOD * 1000, timer_cb, NULL, &loop_timer); // 1000x to convert to ms
    
    printf("Done Booting Up!\n\n");
    
    /*************************************************************
     * TODO:
     *      - Initilize the PID Filters rc_filter_empty()
     *      - Set the PID gains rc_filter_pid()
     * TODO:
     *      - Initialize the PID filters for translation and rotation vel
     *      - Set the PID gains for translation and rotation vel
     *
     *************************************************************/

    // Example initialization of a PID filter defined in mbot.h
    // my_filter = rc_filter_empty();

    // Example of assigning PID parameters (using pid_parameters_t from mbot.h)
    // rc_filter_pid(&my_filter,
    //             pid_params.kp,
    //             pid_params.ki,
    //             pid_params.kd,
    //             1.0 / pid_params.dFilterHz,
    //             1.0 / MAIN_LOOP_HZ);

    // Example of setting limits to the output of the filter
    // rc_filter_enable_saturation(&my_filter, min_val, max_val);

    int running = 1;
    while (running)
    {
        printf("\033[2A\r|                     SENSORS                     |                         ODOMETRY                          |     SETPOINTS     |\n\r|  L_ENC  |  R_ENC  |  gyror  |  gyrop  |  gyroy  |    X    |    Y    |    θ    |  θgyro  |  θodom  |  delta_g_o  |   FWD   |   ANG   |  Left PID  |  Right PID  |  Fwd PID | Turn PID\n\r|%7lld  |%7lld  |%7.3f  |%7.3f  |%7.3f  |%7.3f  |%7.3f  |%7.5f  |%7.3f  |%7.3f  |", 
                                                                                                                                            current_encoders.leftticks, current_encoders.rightticks, mpu_data.dmp_TaitBryan[0], mpu_data.dmp_TaitBryan[1], mpu_data.dmp_TaitBryan[2], current_odom.x, current_odom.y, current_odom.theta, current_cmd.trans_v, current_cmd.angular_v);
        // printf("Left\n");
        // rc_filter_print(left_pid);
        // printf("\n");
        // printf("Right\n");
        // rc_filter_print(right_pid);
        // printf("\n");
        // printf("Fwd\n");
        // rc_filter_print(fwd_vel_pid);
        // printf("\n");
        // printf("Turn\n");
        // rc_filter_print(turn_vel_pid);
        // printf("\n");
        // printf("NEXT\n");
        // sleep_ms(1);
    }
    /*************************************************************
     * End of TODO
     *************************************************************/

    // if (OPEN_LOOP)
    // {
    //     printf("Running in open loop mode\n");
    // }
    // else
    // {
    //     printf("Running in closed loop mode\n");
    // }
}

/**
 * @brief Clamp duty cycle between -1 and 1. If not applied, robot drives in reverse only
 *
 * @param duty
 */
float clamp_duty(float duty)
{
    if (duty > 1.0)
    {
        return 1.0;
    }
    else if (duty < -1.0)
    {
        return -1.0;
    }
    return duty;
}

/**
 * @brief Clamp the orientation between 0 and 2pi
 *
 * @param orientation
 */
float clamp_orientation(float orientation)
{
    // if (orientation > (2 * PI))
    // {
    //     return (2 * PI);
    // }
    // else if (orientation < 0)
    // {
    //     return 0;
    // }
    if (orientation >= (2*PI)) {
        while (orientation >= (2*PI)) {
            orientation -= 2*PI;
        }
    }
    else if (orientation < (-2*PI)) {
        while (orientation < (-2*PI)) {
            orientation += 2*PI;
        }
    }

    return orientation;
}

