#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <motors.h>
#include <math.h>
#include "arm_math.h"
#include "button.h"
#include "leds.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include "sensors/imu.h"
#include "main.h"
#include "bot.h"
#include "communications.h"
#include "pi_regulator.h"

#define USE_IMU
#define USE_TOF

#define TOF_DIST 100 // round-trip, so ~50mm in real distance
#define STANDARD_GRAVITY 9.80665f
#define ZACC_LIMIT 7 // m/s²
#define ZACC_CONVERSION (4*STANDARD_GRAVITY/65535) // converts raw acc value to m/s²

static float position[3] = {0., 0., 0.}; // mm, mm, rad
static float destination[3] = {0., 0., 0.}; // mm, mm, rad
static int16_t* speeds;
static bool obstacle = false;

static THD_WORKING_AREA(waRunBot, 512);
static THD_FUNCTION(RunBot, arg) { // higher priority and empty while-loops for no PI interruptions
    chRegSetThreadName(__FUNCTION__); (void)arg;
    bool first_start = true;
    systime_t time;
    float zacc = 0.;
    messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
    imu_msg_t imu_values;
    while(1) {
    	time = chVTGetSystemTime();
		if(button_is_pressed() || first_start) {
			left_motor_set_speed(0); right_motor_set_speed(0);
			clear_leds(); set_body_led(0); set_front_led(0);
			while(button_is_pressed()) {} // in case of activation by button press, wait until release
			for (uint8_t i = 0; i < 4; i++) set_rgb_led(i, 0, 10, 10); // cyan = waiting for button
			while(!button_is_pressed()) {} while(button_is_pressed()) {} // press and release to start receiving
			ReceiveFloatFromComputer((BaseSequentialStream *) &SD3, destination, 3);
			first_start = false; clear_leds();
			GPTD12.tim->CNT = 0; // time correction for odometry
		}
#ifdef USE_IMU
		else {
			messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));
			zacc = (get_acc(2)-get_acc_offset(2))*ZACC_CONVERSION;
			if (zacc > ZACC_LIMIT) {
				left_motor_set_speed(0); right_motor_set_speed(0);
				clear_leds(); set_body_led(0); set_front_led(0);
				for (uint8_t i = 0; i < 4; i++) set_rgb_led(i, 10, 10, 0); // yellow = bot lifted
				while(!button_is_pressed()) {} while(button_is_pressed()) {}
			}
		}
#endif
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
#ifdef USE_TOF
    	if (VL53L0X_get_dist_mm() < TOF_DIST) {
    		left_motor_set_speed(0); right_motor_set_speed(0);
    		set_front_led(1);
    		obstacle = true;
    		GPTD12.tim->CNT = 0;
    		continue;
    	}
    	set_front_led(0); obstacle = false;// remove effects from obstacle detection
#endif
    	speeds = get_speeds();
    	chSysLock(); // time-sensitive calculations
    	uint16_t timestep = GPTD12.tim->CNT; // /!\ max time of ~650ms
		if (speeds[0] != 0 && speeds[1] != 0) { // http://faculty.salina.k-state.edu/tim/robotics_sg/Control/kinematics/odometry.html
			dleft =  MM_PER_STEP*speeds[0]*timestep*1e-5; // timestep is in 10us
			dright = MM_PER_STEP*speeds[1]*timestep*1e-5;
			dcenter = 0.5*(dleft+dright);
			phi = INV_ROBOT_WIDTH*(dright-dleft);
			position[X_] += dcenter*cos(position[T_]);
			position[Y_] += dcenter*sin(position[T_]);
			position[T_] += phi;
		}
		chSysUnlock();
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
