#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include "main.h"
#include <motors.h>
#include "pi_regulator.h"

static int16_t speeds[2] = {0, 0};

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {
    chRegSetThreadName(__FUNCTION__); (void)arg;
    systime_t time;
    float sum_error = 0;
    int16_t Kp = 600, Kps = 800;
    int8_t Ki = 4;
    while(1) {
        time = chVTGetSystemTime();
        // main speed PI controller (https://github.com/pms67/PID/blob/master/PID.c)
        //float error = get_distance_cm() - 10.f;
        int16_t speed = 0;
        if (fabs(error) > 1.f) {
			sum_error += error;
			if (sum_error > MOTOR_SPEED_LIMIT) sum_error = MOTOR_SPEED_LIMIT;
			else if (sum_error < -MOTOR_SPEED_LIMIT) sum_error = -MOTOR_SPEED_LIMIT;
			speed = Kp * error + Ki * sum_error;
        }
        // steering P controller
        //float position = get_line_position();
        int16_t rotation = 0;
        if (fabs(position) > 0.1f) rotation = Kps * position;
		int16_t rspeed = speed - rotation;
		int16_t lspeed = speed + rotation;
		right_motor_set_speed(rspeed);
		left_motor_set_speed(lspeed);
        chThdSleepUntilWindowed(time, time + MS2ST(10)); // 100Hz
    }
}

int16_t* get_speeds(void){return speeds;}

void pi_regulator_start(void){chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);}
