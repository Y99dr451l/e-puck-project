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
#include "spi_comm.h"
#include <sensors/proximity.h>
#include <math.h>
#include "communications.h"
#include "bot.h"
#include "pi_regulator.h"

//static messagebus_t bus;
//static MUTEX_DECL(bus_lock);
//static CONDVAR_DECL(bus_condvar);

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

int main(void) {
	// order from src/main.c
	halInit();
	chSysInit();

//	messagebus_init(&bus, &bus_lock, &bus_condvar);

	clear_leds();
	set_body_led(0);
	set_front_led(0);
	usb_start();
	motors_init();
	//proximity_start();
//	spi_comm_start();
	serial_start();
	//mic_start(NULL);

	timer12_start();
//    calibrate_ir();
    bot_start();
    pi_regulator_start();

    //use these 2 lines at start of thread using the prox sensors
//    messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
//	proximity_msg_t prox_values;

//	messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));
//	leftSpeed = MOTOR_SPEED_LIMIT - prox_values.delta[0]*2 - prox_values.delta[1];
//	rightSpeed = MOTOR_SPEED_LIMIT - prox_values.delta[7]*2 - prox_values.delta[6];
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void) {chSysHalt("Stack smashing detected");}
