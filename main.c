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
#include <sensors/proximity.h>
#include <communications.h>

#define NB_DIRECTIONS 4

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

static void serial_start(void) {
	static SerialConfig ser_cfg = {115200, 0, 0, 0,};
	sdStart(&SD3, &ser_cfg); // UART3.
}

static void timer12_start(void){
    //General Purpose Timer configuration   
    //timer 12 is a 16 bit timer so we can measure time to about 65ms with a 1Mhz counter
    static const GPTConfig gpt12cfg = {
        1000000,        /* 1MHz timer clock in order to measure uS.*/
        NULL,           /* Timer callback.*/
        0,
        0};
    gptStart(&GPTD12, &gpt12cfg);
    gptStartContinuous(&GPTD12, 0xFFFF); //let the timer count to max value
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

int main(void) {
	messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
	proximity_msg_t prox_values;
	messagebus_init(&bus, &bus_lock, &bus_condvar);
    halInit();
    chSysInit();
    //mpu_init();
    serial_start(); //starts the serial communication
    usb_start(); //starts the USB communication
    timer12_start(); //starts timer 12
    motors_init(); //inits the motors
//    proximity_start(); // undefined reference to 'bus'
//    calibrate_ir();
    int16_t MotorInput[NB_DIRECTIONS] = {0, 0, 0, 0};
    int16_t speeds[2] = {0, 0};
    float xpos = 0, ypos = 0, theta = 0; //(x, y, theta)
    while (1) {
        ReceiveInt16FromComputer((BaseSequentialStream *) &SD3, MotorInput, NB_DIRECTIONS);
        SetMotor(MotorInput, speeds);
        int16_t wheelpos[2] = {(uint16_t)left_motor_get_pos(), (uint16_t)right_motor_get_pos()};
        SendInt16ToComputer((BaseSequentialStream *) &SD3, wheelpos, 2);
        // 1000 STEPS PER FULL ROTATION, R = 4 CM -> D = 12.566 CM -> 0.012566 CM/STEP
//        if (speeds[0] != 0 && speeds[1] != 0) { // http://faculty.salina.k-state.edu/tim/robotics_sg/Control/kinematics/odometry.html
//			float timestep = ;
//			float dleft = 0.012566*speeds[0]*timestep;
//			float dright = 0.012566*speeds[1]*timestep;
//			float dcenter = 0.5*(dleft+dright);
//			float phi = (dright-dleft)/ROBOT_WIDTH;
//			xpos += dcenter*cos(position[2]+0.5*phi);
//			ypos += dcenter*sin(position[2]+0.5*phi);
//			theta += phi;
//        }
//        messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));
//        for (uint8_t i = 0; i < 8; i++) {
//        	int8_t prox = get_calibrated_prox(i);
//        	SendInt8ToComputer((BaseSequentialStream *) &SDU1, prox, sizeof(int8_t));
//        }
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void) {chSysHalt("Stack smashing detected");}
