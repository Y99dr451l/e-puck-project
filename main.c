#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>
#include <motors.h>
#include "leds.h"
#include "button.h"
#include <sensors/proximity.h>
#include <communications.h>
#include <math.h>

#define MMPERSTEP 0.12566f
#define ROBOT_WIDTH 54 // mm
#define INV_ROBOT_WIDTH 0.0185185185f // 1/54
#define STANDARD_GRAVITY 9.80665f
#define Xpos 0
#define Ypos 1
#define Theta 2
#define ThetaError 0.1f // rad

//messagebus_t bus;
//MUTEX_DECL(bus_lock);
//CONDVAR_DECL(bus_condvar);

static float position[3] = {0, 0, 0};
static float destination[3] = {0, 0, 0};
static bool full_reset = false;
static BSEMAPHORE_DECL(init_ready_sem, TRUE);

static void serial_start(void) {
	static SerialConfig ser_cfg = {115200, 0, 0, 0};
	sdStart(&SD3, &ser_cfg); // UART3.
}

static void timer12_start(void){
    // timer 12 is a 16 bit timer so we can measure time to about 65ms with a 1Mhz counter
    static const GPTConfig gpt12cfg = {1000000, NULL, 0, 0}; // 1MHz timer clock in order to measure uS
    gptStart(&GPTD12, &gpt12cfg);
    gptStartContinuous(&GPTD12, 0xFFFF); // let the timer count to max value
}

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

//void SetLed() {
//	for (int i = 0; i < )
//	set_rgb_led(0, 10, 0, 0);
//	set_rgb_led(1, 10, 0, 0);
//	set_rgb_led(2, 10, 0, 0);
//	set_rgb_led(3, 10, 0, 0);
//}

int main(void) {
//	messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
//	proximity_msg_t prox_values;
//	messagebus_init(&bus, &bus_lock, &bus_condvar);
    halInit();
    chSysInit();
    serial_start();
    usb_start();
    timer12_start();
    motors_init();
//    proximity_start();
//    calibrate_ir();

    int16_t MotorInput[NB_DIRECTIONS] = {1, 1, 0, 0};
    int16_t speeds[2] = {0, 0};
    volatile systime_t time = 0;
    uint16_t timestep = 0;
    float dleft, dright, dcenter, phi;

//    SetMotor(MotorInput, speeds);
    while (1) {
        ReceiveInt16FromComputer((BaseSequentialStream *) &SD3, MotorInput, 4);
        timestep = GPTD12.tim->CNT;
        SetMotor(MotorInput, speeds);
        GPTD12.tim->CNT = 0;
//        wheelpos[0] = (uint16_t)left_motor_get_pos(); wheelpos[1] = (uint16_t)right_motor_get_pos();
//        SendInt16ToComputer((BaseSequentialStream *) &SD3, wheelpos, 2);
        if (speeds[0] != 0 && speeds[1] != 0) { // http://faculty.salina.k-state.edu/tim/robotics_sg/Control/kinematics/odometry.html
			dleft = MMPERSTEP*speeds[0]*timestep*1e-6;
			dright = MMPERSTEP*speeds[1]*timestep*1e-6;
			dcenter = 0.5*(dleft+dright);
			phi = (dright-dleft)*INV_ROBOT_WIDTH;
			position[Xpos]  += dcenter*cos(position[Theta]+0.5*phi);
			position[Ypos]  += dcenter*sin(position[Theta]+0.5*phi);
			position[Theta] += phi;
        }
        if (SD3.state == SD_READY) {
//        if (SDU1.config->usbp->state == USB_ACTIVE) {
			time = chVTGetSystemTime();
			SendFloatToComputer((BaseSequentialStream *) &SD3, position, 3);
        }
        chThdSleepUntilWindowed(time, time + MS2ST(1000));

//        messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));
//        leftSpeed = MOTOR_SPEED_LIMIT - prox_values.delta[0]*2 - prox_values.delta[1];
//		  rightSpeed = MOTOR_SPEED_LIMIT - prox_values.delta[7]*2 - prox_values.delta[6];
//		  right_motor_set_speed(rightSpeed);
//		  left_motor_set_speed(leftSpeed);
    }
}

static THD_WORKING_AREA(waInitBot, 1024); // UPDATE THIS NUMBER
static THD_FUNCTION(InitBot, arg) { // triggered on start or on reset button
    chRegSetThreadName(__FUNCTION__); (void)arg;
    for (uint8_t i = 0; i < 4; i++) set_rgb_led(i, 0, 0, 10); // blue = reset
    chThdSleep(500); // wait for button release
    GPTD12.tim->CNT = 0;
    while(GPTD12.tim->CNT < 1000) // also await new position if button is pressed again within 1s
    	if(button_is_pressed() || full_reset) {
    		for (int i = 0; i < 4; i++) set_rgb_led(i, 0, 10, 10); // cyan = full reset
    		ReceiveFloatFromComputer((BaseSequentialStream *) &SD3, position, 3);
    		full_reset = false;
    		break;
    	}
    ReceiveFloatFromComputer((BaseSequentialStream *) &SD3, destination, 3);
    chBSemSignal(&image_ready_sem); // toggle semaphore?
    // yield
}

static THD_WORKING_AREA(waRunBot, 1024); // UPDATE THIS NUMBER
static THD_FUNCTION(RunBot, arg) { // movement and odometry
    chRegSetThreadName(__FUNCTION__); (void)arg;
    for (uint8_t i = 0; i < 4; i++) toggle_rgb_led(i, GREEN_LED, 10); // blinking
    float Xdiff = destination[Xpos]-position[Xpos];
    float Ydiff = destination[Ypos]-position[Ypos];
    float Thetadiff = destination[Theta]-position[Theta];
    float path[3] = {Xdiff, Ydiff, arctan(Ydiff/Xdiff)}; // initialize path
    while(abs(Thetadiff > ThetaError)) { // adjust orientation

    }
    while(1) { // infinite loop for movement, odometry and sensors

    	// event if sensors detect smth, maybe new thread

    }
}

//void process_image_start(void){
//	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
//	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
//}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void) {chSysHalt("Stack smashing detected");}
