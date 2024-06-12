#ifndef MBOT_H
#define MBOT_H

#include <pico/stdlib.h>
#include <pico/mutex.h>
#include <pico/multicore.h>
#include <pico/time.h>
#include <rc/motor/motor.h>
#include <rc/encoder/encoder.h>
#include <rc/motor/motor.h>
#include <rc/defs/common_defs.h>
#include <rc/defs/mbot_diff_defs.h>
#include <rc/fram/fram.h>
#include <rc/math/filter.h>
#include <rc/mpu/mpu.h>

#include <math.h>
#include <inttypes.h>

// data to hold current mpu state (not used)
//static mb_mpu_data_t mpu_data;
// static i2c_inst_t *i2c;

uint64_t timestep_us = 0;

// data to hold calibration coefficients
float coeffs[4];

float clamp_duty(float duty);

#endif

