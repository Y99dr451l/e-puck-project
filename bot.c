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
#define WHEEL_D 4
#define INV_ROBOT_WIDTH 0.0185185185f // 1/54

static float position[3] = {0., 0., 0.}; // mm, mm, rad
static float destination[3] = {100., 0., 0.}; // mm, mm, rad
static int16_t* speeds;
static void bot_reset(void);

static THD_WORKING_AREA(waRunBot, 512);
static THD_FUNCTION(RunBot, arg) { // higher priority for no PI interruptions
    chRegSetThreadName(__FUNCTION__); (void)arg;
    bool first_start = false;
    uint8_t null_speed_counter = 0;
    systime_t time;
    while(1) {
    	time = chVTGetSystemTime();
    	speeds = get_speeds();
		if (speeds[0] == 0 && speeds[1] == 0) null_speed_counter++;
		if(button_is_pressed() || first_start || null_speed_counter > 20) {
			clear_leds();
			left_motor_set_speed(0); right_motor_set_speed(0);
			for (uint8_t i = 0; i < 4; i++) set_rgb_led(i, 0, 10, 10); // cyan
//			while(1) {
//				uint16_t size = ReceiveFloatFromComputer((BaseSequentialStream *) &SD3, destination, 3);
//				if (size == 3) break;
//			}
			first_start = false; null_speed_counter = 0; clear_leds(); bot_reset();
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
    	speeds = get_speeds();
    	chSysLock(); // time-sensitive calculations
    	uint16_t timestep = GPTD12.tim->CNT; // /!\ max time of ~650ms
		if (speeds[0] != 0 && speeds[1] != 0) { // http://faculty.salina.k-state.edu/tim/robotics_sg/Control/kinematics/odometry.html
			dleft =  WHEEL_D*speeds[0]*timestep*1e-4; // timestep is in 10us
			dright = WHEEL_D*speeds[1]*timestep*1e-4;
			dcenter = 0.5*(dleft+dright);
			phi = INV_ROBOT_WIDTH*(dright-dleft);
			position[X_] += dcenter*cos(position[T_]+0.5*phi); // front at heading 0
			position[Y_] += dcenter*sin(position[T_]+0.5*phi); // left at heading 0
			position[T_] += phi;
		}
		chSysUnlock();
		GPTD12.tim->CNT = 0;
		chThdSleepUntilWindowed(time, time + MS2ST(10)); // 100Hz
    }
}

float* get_position(void) {return position;}
float* get_destination(void) {return destination;}

void bot_start(void) {
	chThdCreateStatic(waOdometry, sizeof(waOdometry), NORMALPRIO, Odometry, NULL);
	chThdCreateStatic(waRunBot, sizeof(waRunBot), NORMALPRIO+1, RunBot, NULL);
}

void bot_reset(void) {
	// speeds to 0
	// error_sum to 0
}
