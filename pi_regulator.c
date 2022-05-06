#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include "main.h"
#include "bot.h"
#include <motors.h>
#include "pi_regulator.h"

#define max_D_error 1.f
#define max_T_error 1.f

static int16_t speeds[2] = {0, 0};

static THD_WORKING_AREA(waPiRegulator, 512);
static THD_FUNCTION(PiRegulator, arg) {
    chRegSetThreadName(__FUNCTION__); (void)arg;
	float* position;
	float* destination;
    float X_error, Y_error, D_error, Tm_error, Td_error;
    float D_error_sum = 0;
    int16_t speed;
    systime_t time;
    int16_t Kp = 600, Kps = 800;
    int8_t Ki = 4;
    while(1) { // PI controller: https://github.com/pms67/PID/blob/master/PID.c
        time = chVTGetSystemTime();
        position = get_position();
        destination = get_destination();
        X_error = destination[X_]-position[X_];
		Y_error = destination[Y_]-position[Y_];
		D_error = X_error*X_error+Y_error*Y_error; // distance to goal (squared)
		Tm_error = atan(Y_error/X_error)-position[T_]; // heading error
		Td_error = destination[T_]-position[T_]; // angle error (at destination)
        if (fabs(Tm_error) > max_T_error) { // if heading is wrong
        	speed = Kps*Tm_error;
        	right_motor_set_speed(-speed);
        	left_motor_set_speed(speed);
        } else if (fabs(D_error) > max_D_error) { // if position is wrong
        	D_error_sum += D_error;
        	if (D_error_sum > MOTOR_SPEED_LIMIT) D_error_sum = MOTOR_SPEED_LIMIT;
        	else if (D_error_sum < -MOTOR_SPEED_LIMIT) D_error_sum = -MOTOR_SPEED_LIMIT;
        	speed = Kp * D_error + Ki * D_error_sum;
        	right_motor_set_speed(speed);
			left_motor_set_speed(speed);
        } else if (fabs(Td_error) > max_T_error) { // if Theta is wrong
        	speed = Kps*Td_error;
			right_motor_set_speed(-speed);
			left_motor_set_speed(speed);
        }
//        sum_error += error;
//		if (sum_error > MOTOR_SPEED_LIMIT) sum_error = MOTOR_SPEED_LIMIT;
//		else if (sum_error < -MOTOR_SPEED_LIMIT) sum_error = -MOTOR_SPEED_LIMIT;
//		speed = Kp * error + Ki * sum_error;
//        float position = get_line_position();
//        int16_t rotation = 0;
//        if (fabs(position) > 0.1f) rotation = Kps * position;
//		int16_t rspeed = speed - rotation;
//		int16_t lspeed = speed + rotation;
//		right_motor_set_speed(rspeed);
//		left_motor_set_speed(lspeed);
        chThdSleepUntilWindowed(time, time + MS2ST(10)); // 100Hz
    }
}

int16_t* get_speeds(void){return speeds;}

void pi_regulator_start(void){chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);}
