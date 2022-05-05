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
#define Xpos 0
#define Ypos 1
#define Theta 2
#define ThetaError 0.1f // rad

static float position[3] = {0., 0., 0.}, destination[3] = {0., 0., 0.}, path[3] = {0., 0., 0.};
static float* speeds;
static bool full_reset = true, running = false;
static BSEMAPHORE_DECL(init_ready_sem, TRUE);

void SetMotor(int16_t MotorInput[4], int16_t* speeds) {
	int16_t speed = 0, rotation = 0;
	if (MotorInput[0] == 1) speed += 800;
	if (MotorInput[1] == 1) rotation += 300;
	if (MotorInput[2] == 1) speed -= 800;
	if (MotorInput[3] == 1) rotation -= 300;
	speeds[0] = speed - rotation;
	speeds[1] = speed + rotation;
	left_motor_set_speed(speeds[0]);
	right_motor_set_speed(speeds[1]);
}

static THD_WORKING_AREA(waInitBot, 128); // UPDATE THIS NUMBER
static THD_FUNCTION(InitBot, arg) { // triggered on start or on reset button
    chRegSetThreadName(__FUNCTION__); (void)arg;
    if(button_is_pressed() || full_reset) {
    	set_body_led(0); for (uint8_t i = 0; i < 4; i++) set_rgb_led(i, 0, 0, 10); // blue = reset
    	left_motor_set_speed(0); right_motor_set_speed(0);
		chThdSleep(500); // wait for button release
		GPTD12.tim->CNT = 0;
		while(GPTD12.tim->CNT < 1000) // also await new position if button is pressed in the next 1s
			if(button_is_pressed() || full_reset) {
				for (uint8_t i = 0; i < 4; i++) set_rgb_led(i, 0, 10, 10); // cyan = full reset
				ReceiveFloatFromComputer((BaseSequentialStream *) &SD3, position, 3);
				break;
			}
		ReceiveFloatFromComputer((BaseSequentialStream *) &SD3, destination, 3);
		full_reset = false;
    }
    for (uint8_t i = 0; i < 4; i++) set_rgb_led(i, 0, 0, 0);
    running = true;
    //chBSemSignal(&init_ready_sem); // toggle semaphore?
    // yield
}

static THD_WORKING_AREA(waRunBot, 256); // UPDATE THIS NUMBER
static THD_FUNCTION(RunBot, arg) { // movement and odometry
    chRegSetThreadName(__FUNCTION__); (void)arg;
    if (running) {
		set_body_led(1);
		float Xdiff = destination[Xpos]-position[Xpos];
		float Ydiff = destination[Ypos]-position[Ypos];
		float Thetadiff = destination[Theta]-position[Theta];
		path[0] = Xdiff; path[1] = Ydiff; path[2] = atan(Ydiff/Xdiff); // update path
		// update odometry (http://faculty.salina.k-state.edu/tim/robotics_sg/Control/kinematics/odometry.html)
		speeds = get_speeds;
		uint16_t timestep = GPTD12.tim->CNT; // /!\ max time of 65ms
		if (speeds[0] != 0 && speeds[1] != 0) {
			float dleft = MMPERSTEP*speeds[0]*timestep*1e-6; // timestep is in us
			float dright = MMPERSTEP*speeds[1]*timestep*1e-6;
			float dcenter = 0.5*(dleft+dright);
			float phi = (dright-dleft)*INV_ROBOT_WIDTH;
			position[Xpos]  += dcenter*cos(position[Theta]+0.5*phi);
			position[Ypos]  += dcenter*sin(position[Theta]+0.5*phi);
			position[Theta] += phi;
		} GPTD12.tim->CNT = 0;
		// update speeds with regulator
    }
}

float* get_position(void) {return position;}
float* get_destination(void) {return destination;}

void bot_start(void){
	chThdCreateStatic(waInitBot, sizeof(waInitBot), NORMALPRIO, InitBot, NULL);
	chThdCreateStatic(waRunBot, sizeof(waRunBot), NORMALPRIO, RunBot, NULL);
}
