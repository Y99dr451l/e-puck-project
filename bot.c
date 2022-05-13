#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <motors.h>
#include <math.h>
#include "button.h"
#include "leds.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include "main.h"
#include "bot.h"
#include "communications.h"
#include "pi_regulator.h"

#define MM_PER_STEP (0.12566f*0.322f) // actual value * weird correction
#define INV_ROBOT_WIDTH (0.01852f * 2) // 1/54, same thing here
#define TOF_DIST 100 // round-trip, so ~50mm in real distance

static float position[3] = {0., 0., 0.}; // mm, mm, rad
static float destination[3] = {100., 100., 0.}; // mm, mm, rad
static int16_t* speeds;

static THD_WORKING_AREA(waRunBot, 512);
static THD_FUNCTION(RunBot, arg) { // higher priority for no PI interruptions
    chRegSetThreadName(__FUNCTION__); (void)arg;
    bool first_start = true;
    uint8_t null_speed_counter = 0;
    systime_t time;
    while(1) {
    	time = chVTGetSystemTime();
    	speeds = get_speeds();
		if (speeds[0] == 0 && speeds[1] == 0) null_speed_counter++;
		if(button_is_pressed() || null_speed_counter > 20 || first_start) {
			clear_leds(); set_body_led(0); set_front_led(0);
			left_motor_set_speed(0); right_motor_set_speed(0);
			for (uint8_t i = 0; i < 4; i++) set_rgb_led(i, 0, 10, 10); // cyan = waiting for button
			while(!button_is_pressed()) {} // press button to start receiving
			for (uint8_t i = 0; i < 4; i++) set_rgb_led(i, 0, 0, 10); // blue = receiving
			ReceiveFloatFromComputer((BaseSequentialStream *) &SD3, destination, 3);
			first_start = false; null_speed_counter = 0; clear_leds();
			GPTD12.tim->CNT = 0; // time correction for odometry
		}
		set_body_led(1);
		chThdSleepUntilWindowed(time, time + MS2ST(100)); // 10Hz
    }
}

static THD_WORKING_AREA(waOdometry, 256);
static THD_FUNCTION(Odometry, arg) {
    chRegSetThreadName(__FUNCTION__); (void)arg;
    systime_t time;
    float dleft, dright, dcenter, phi;
    while(1) {
    	time = chVTGetSystemTime();
    	if (VL53L0X_get_dist_mm() < TOF_DIST) {
    		left_motor_set_speed(0); right_motor_set_speed(0);
//    		for (uint8_t i = 0; i < 4; i++) set_rgb_led(i, 10, 0, 10); // purple
    		set_front_led(1);
    		GPTD12.tim->CNT = 0;
    		continue;
    	} else set_front_led(0); //for (uint8_t i = 0; i < 4; i++) set_rgb_led(i, 0, 0, 0);
    	chSysLock(); // time-sensitive calculations
    	uint16_t timestep = GPTD12.tim->CNT; // /!\ max time of ~650ms
		if (speeds[0] != 0 && speeds[1] != 0) { // http://faculty.salina.k-state.edu/tim/robotics_sg/Control/kinematics/odometry.html
			dleft =  MM_PER_STEP*speeds[0]*timestep*1e-5; // timestep is in 10us
			dright = MM_PER_STEP*speeds[1]*timestep*1e-5;
			dcenter = 0.5*(dleft+dright);
			phi = INV_ROBOT_WIDTH*(dright-dleft);
			position[X_] += dcenter*cos(position[T_]); //+0.5*phi);
			position[Y_] += dcenter*sin(position[T_]); //+0.5*phi);
			position[T_] += phi;
		}
		chSysUnlock();
		speeds = get_speeds();
		left_motor_set_speed(speeds[0]); right_motor_set_speed(speeds[1]);
		GPTD12.tim->CNT = 0;
		chThdSleepUntilWindowed(time, time + MS2ST(10)); // 100Hz
    }
}

float* get_position(void) {return position;}
float* get_destination(void) {return destination;}

void bot_start(void) {
	chThdCreateStatic(waRunBot, sizeof(waRunBot), NORMALPRIO+1, RunBot, NULL);
	chThdCreateStatic(waOdometry, sizeof(waOdometry), NORMALPRIO, Odometry, NULL);
}
