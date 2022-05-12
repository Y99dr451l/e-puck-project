#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include "main.h"
#include "bot.h"
#include "leds.h"
#include "motors.h"
#include "pi_regulator.h"

#define SPEED_LIMIT 800
#define ROTATION_LIMIT MOTOR_SPEED_LIMIT-SPEED_LIMIT
#define max_D_error 2.f // mm
#define max_T_error 0.05f // rad, = 3°
#define Kp 50
#define Ki 200
#define Kps 50
#define Kis 200

static int16_t speeds[2] = {0, 0};
static int16_t speedrot[2] = {0., 0.};

static THD_WORKING_AREA(waPiRegulator, 512);
static THD_FUNCTION(PiRegulator, arg) {
    chRegSetThreadName(__FUNCTION__); (void)arg;
	float* position;
	float* destination;
    float D_error_sum = 0, Tm_error_sum = 0, T_error_sum = 0;
    int32_t speed = 0, rotation = 0;
    float errors[5] = {0., 0., 0., 0., 0.};
    systime_t time;
    while(1) { // https://github.com/pms67/PID/blob/master/PID.c
        time = chVTGetSystemTime();
        position = get_position(); destination = get_destination();
        errors[X_] = destination[X_] - position[X_];
		errors[Y_] = destination[Y_] - position[Y_];
		errors[T_] = destination[T_] - position[T_]; // heading error at dest
		errors[Tm] = atan2f(errors[Y_], errors[X_]) - position[T_]; // heading error
		errors[D_] = sqrt(errors[X_] * errors[X_] + errors[Y_] * errors[Y_]); // distance to dest
		clear_leds();


		// waddling
		rotation = 0; speed = 0;
		if (fabs(errors[D_]) > max_D_error) {
        	if (fabs(errors[Tm]) > max_T_error) {
				Tm_error_sum += errors[Tm];
				if (Tm_error_sum > ROTATION_LIMIT) Tm_error_sum = ROTATION_LIMIT;
				else if (Tm_error_sum < -ROTATION_LIMIT) Tm_error_sum = -ROTATION_LIMIT;
				rotation = Kps * errors[Tm] + Kis * Tm_error_sum;
				set_led(LED5, 10);
        	} else {
        		D_error_sum += errors[D_];
				if (D_error_sum > SPEED_LIMIT) D_error_sum = SPEED_LIMIT;
				else if (D_error_sum < -SPEED_LIMIT) D_error_sum = -SPEED_LIMIT;
				speed = Kp * errors[D_] + Ki * D_error_sum;
				set_led(LED1, 10);
        	}
        } else if (fabs(errors[T_]) > max_T_error) { // if final heading is wrong
        	T_error_sum += errors[T_];
			if (T_error_sum > ROTATION_LIMIT) T_error_sum = ROTATION_LIMIT;
			else if (T_error_sum < -ROTATION_LIMIT) T_error_sum = -ROTATION_LIMIT;
			rotation = Kps * errors[T_] + Kis * T_error_sum;
        	set_led(LED3, 10); set_led(LED7, 10);
        }
        if (speed > SPEED_LIMIT) speed = SPEED_LIMIT;
		else if (speed < -SPEED_LIMIT) speed = -SPEED_LIMIT;
        if (rotation > ROTATION_LIMIT) rotation = ROTATION_LIMIT;
		else if (rotation < -ROTATION_LIMIT) rotation = -ROTATION_LIMIT;
        speedrot[0] = speed; speedrot[1] = rotation;
        speeds[0] = speed - rotation; speeds[1] = speed + rotation;
        chThdSleepUntilWindowed(time, time + MS2ST(10)); // 100Hz
    }
}

int16_t* get_speeds(void){return speeds;}
int16_t* get_speedrot(void){return speedrot;}

void pi_regulator_start(void){chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);}
