#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include "main.h"
#include "bot.h"
#include "leds.h"
#include <motors.h>
#include "pi_regulator.h"

#define max_D_error 5.f // mm
#define max_T_error 0.1f // rad
#define Kp 600
#define Ki 4
#define Kps 400

static int16_t speeds[2] = {0, 0};
static float errors[5] = {0., 0., 0., 0., 0.};

static THD_WORKING_AREA(waPiRegulator, 512);
static THD_FUNCTION(PiRegulator, arg) {
    chRegSetThreadName(__FUNCTION__); (void)arg;
	float* position;
	float* destination;
    float D_error_sum = 0;
    int16_t speed, lspeed, rspeed;
    systime_t time;
    while(1) { // PI controller: https://github.com/pms67/PID/blob/master/PID.c
        time = chVTGetSystemTime();
        lspeed = 0; rspeed = 0;
        position = get_position();
        destination = get_destination();
        errors[X_] = destination[X_] - position[X_];
		errors[Y_] = destination[Y_] - position[Y_];
		errors[T_] = destination[T_] - position[T_]; // heading error at dest
		errors[Tm] = atan2(errors[Y_], errors[X_]) - position[T_]; // heading error
		errors[D_] = errors[X_] * errors[X_] + errors[Y_] * errors[Y_]; // distance to dest (squared)
		clear_leds();
		if (fabs(errors[Tm]) > max_T_error) { // if heading is wrong
        	speed = Kps * errors[Tm]; lspeed = -speed; rspeed = speed;
        	set_led(LED5, 10);
        } else if (fabs(errors[D_]) > max_D_error) { // if position is wrong
        	D_error_sum += errors[D_];
        	if (D_error_sum > MOTOR_SPEED_LIMIT) D_error_sum = MOTOR_SPEED_LIMIT;
        	else if (D_error_sum < -MOTOR_SPEED_LIMIT) D_error_sum = -MOTOR_SPEED_LIMIT;
        	speed = Kp * errors[D_] + Ki * D_error_sum; lspeed = speed; rspeed = speed;
        	set_led(LED1, 10);
        } else if (fabs(errors[T_]) > max_T_error) { // if final heading is wrong
        	speed = Kps * errors[T_]; lspeed = -speed; rspeed = speed;
        	set_led(LED3, 10); set_led(LED7, 10);
        }
		if (lspeed > MOTOR_SPEED_LIMIT) lspeed = MOTOR_SPEED_LIMIT;
		else if (lspeed < -MOTOR_SPEED_LIMIT) lspeed = -MOTOR_SPEED_LIMIT;
		if (rspeed > MOTOR_SPEED_LIMIT) rspeed = MOTOR_SPEED_LIMIT;
		else if (rspeed < -MOTOR_SPEED_LIMIT) rspeed = -MOTOR_SPEED_LIMIT;
        left_motor_set_speed(lspeed); right_motor_set_speed(rspeed);
        speeds[0] = lspeed; speeds[1] = rspeed;
        chThdSleepUntilWindowed(time, time + MS2ST(10)); // 100Hz
    }
}

int16_t* get_speeds(void){return speeds;}

void pi_regulator_start(void){chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);}
