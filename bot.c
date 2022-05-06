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

#define MMPERSTEP 0.12566f
#define ROBOT_WIDTH 54 // mm
#define INV_ROBOT_WIDTH 0.0185185185f // 1/54
#define STANDARD_GRAVITY 9.80665f
#define X_ 0
#define Y_ 1
#define T_ 2 // Theta
#define T_Error 0.1f // rad

static float position[3] = {0., 0., 0.};
static float destination[3] = {0., 0., 0.};
static int16_t* speeds;
static bool full_reset = true;
static BSEMAPHORE_DECL(init_start_sem, TRUE);
static BSEMAPHORE_DECL(bot_start_sem, TRUE);

static THD_WORKING_AREA(waInitBot, 128); // UPDATE THIS NUMBER
static THD_FUNCTION(InitBot, arg) { // triggered on start or on reset button
    chRegSetThreadName(__FUNCTION__); (void)arg;
    while(1) {
    	set_body_led(0);
    	for (uint8_t i = 0; i < 4; i++) set_rgb_led(i, 0, 10, 10); // cyan = reset
    	left_motor_set_speed(0); right_motor_set_speed(0);
		chThdSleep(500); // wait for button release
		GPTD12.tim->CNT = 0;
		while(GPTD12.tim->CNT < 1000) // await new position first if button-press in the next second
			if(button_is_pressed() || full_reset) {
				for (uint8_t i = 0; i < 4; i++) set_rgb_led(i, 10, 10, 0); // orange = full reset
				ReceiveFloatFromComputer((BaseSequentialStream *) &SD3, position, 3);
				break;
			}
		ReceiveFloatFromComputer((BaseSequentialStream *) &SD3, destination, 3);
		for (uint8_t i = 0; i < 4; i++) set_rgb_led(i, 0, 0, 0);
		full_reset = false;
		chBSemSignal(&bot_start_sem);
		chBSemWait(&init_start_sem);
    }
}

static THD_WORKING_AREA(waRunBot, 256); // UPDATE THIS NUMBER
static THD_FUNCTION(RunBot, arg) { // movement and odometry
    chRegSetThreadName(__FUNCTION__); (void)arg;
    float dleft, dright, dcenter, phi;
    while(1) {
		speeds = get_speeds();
		if(button_is_pressed() || (speeds[0] == 0 && speeds[1] == 0)) {
			chBSemSignal(&init_start_sem);
			chBSemWait(&bot_start_sem);
		}
		set_body_led(1);
		// odometry: http://faculty.salina.k-state.edu/tim/robotics_sg/Control/kinematics/odometry.html
		uint16_t timestep = GPTD12.tim->CNT; // /!\ max time of 65ms
		if (speeds[0] != 0 && speeds[1] != 0) {
			dleft = MMPERSTEP*speeds[0]*timestep*1e-6; // timestep is in us
			dright = MMPERSTEP*speeds[1]*timestep*1e-6;
			dcenter = 0.5*(dleft+dright);
			phi = (dright-dleft)*INV_ROBOT_WIDTH;
			position[X_] += dcenter*cos(position[T_]+0.5*phi);
			position[Y_] += dcenter*sin(position[T_]+0.5*phi);
			position[T_] += phi;
		} GPTD12.tim->CNT = 0;
    }
}

float* get_position(void) {return position;}
float* get_destination(void) {return destination;}

void bot_start(void) {
	chThdCreateStatic(waInitBot, sizeof(waInitBot), NORMALPRIO, InitBot, NULL);
	chThdCreateStatic(waRunBot, sizeof(waRunBot), NORMALPRIO, RunBot, NULL);
}
