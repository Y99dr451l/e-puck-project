#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <motors.h>
#include <math.h>
#include "button.h"
#include "leds.h"
#include "main.h"
#include "bot.h"
#include "communications.h"
#include "pi_regulator.h"

#define MM_PER_STEP 0.12566f
#define ROBOT_WIDTH 54 // mm
#define INV_ROBOT_WIDTH 0.0185185185f // 1/54
#define STANDARD_GRAVITY 9.80665f

static float position[3] = {0., 0., 0.}; // mm, mm, rad
static float destination[3] = {0., 100., 0.}; // mm, mm, rad
static int16_t* speeds;

static THD_WORKING_AREA(waRunBot, 128);
static THD_FUNCTION(RunBot, arg) {
    chRegSetThreadName(__FUNCTION__); (void)arg;
    bool first_start = false;
    uint8_t null_speed_counter = 0;
    set_body_led(1);
    while(1) {
		if (speeds[0] == 0 && speeds[1] == 0) null_speed_counter++;
		if(button_is_pressed() || first_start || null_speed_counter > 20) {
	    	set_body_led(0); left_motor_set_speed(0); right_motor_set_speed(0);
	    	while(button_is_pressed()) {} // wait for button release without PI interruptions
			for (uint8_t i = 0; i < 4; i++) set_rgb_led(i, 0, 10, 10); // cyan = reset
			GPTD12.tim->CNT = 0;
			while(GPTD12.tim->CNT < 100000) // await new position first if button-press in the next second
				if(button_is_pressed() || first_start || null_speed_counter > 20) {
					for (uint8_t i = 0; i < 4; i++) set_rgb_led(i, 10, 10, 0); // orange = full reset
					ReceiveFloatFromComputer((BaseSequentialStream *) &SD3, position, 3);
					break;
				}
			ReceiveFloatFromComputer((BaseSequentialStream *) &SD3, destination, 3);
			for (uint8_t i = 0; i < 4; i++) set_rgb_led(i, 0, 0, 0);
			set_body_led(1);
			first_start = false; null_speed_counter = 0;
			GPTD12.tim->CNT = 0; // time correction for odometry
			chThdYield(); // takes any free slot without set frequency
		}
    }
}

static THD_WORKING_AREA(waOdometry, 256);
static THD_FUNCTION(Odometry, arg) {
    chRegSetThreadName(__FUNCTION__); (void)arg;
    systime_t time;
    float dleft, dright, dcenter, phi;
    while(1) {
    	time = chVTGetSystemTime();
    	speeds = get_speeds();
		uint16_t timestep = GPTD12.tim->CNT; // /!\ max time of ~650ms
		if (speeds[0] != 0 && speeds[1] != 0) { // odometry: http://faculty.salina.k-state.edu/tim/robotics_sg/Control/kinematics/odometry.html
			dleft =  MM_PER_STEP*speeds[0]*timestep*1e-5; // timestep is in 10us
			dright = MM_PER_STEP*speeds[1]*timestep*1e-5;
			dcenter = 0.5*(dleft+dright);
			phi = (dright-dleft)*INV_ROBOT_WIDTH;
			position[X_] += dcenter*cos(position[T_]+0.5*phi); // front at heading 0
			position[Y_] += dcenter*sin(position[T_]+0.5*phi); // left at heading 0
			position[T_] += phi;
		}
		GPTD12.tim->CNT = 0;
		chThdSleepUntilWindowed(time, time + MS2ST(10)); // 100Hz
    }
}

float* get_position(void) {return position;}
float* get_destination(void) {return destination;}

void bot_start(void) {
	chThdCreateStatic(waRunBot, sizeof(waRunBot), NORMALPRIO, RunBot, NULL);
	chThdCreateStatic(waOdometry, sizeof(waOdometry), NORMALPRIO, Odometry, NULL);
}
