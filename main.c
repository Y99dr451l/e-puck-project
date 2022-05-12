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
    static const GPTConfig gpt12cfg = {100000, NULL, 0, 0}; // 100KHz timer clock to measure 10us
    gptStart(&GPTD12, &gpt12cfg);
    gptStartContinuous(&GPTD12, 0xFFFF);
}

int main(void) {
	// order from src/main.c
	halInit();
	chSysInit();
	mpu_init();
//	messagebus_init(&bus, &bus_lock, &bus_condvar);
	usb_start();
	motors_init();
//	proximity_start();
	spi_comm_start();
	serial_start();
//	mic_start(NULL);
	timer12_start();
//  calibrate_ir();

	clear_leds();
	set_body_led(0);
	set_front_led(0);
	pi_regulator_start();
    bot_start();

    while (1) {chThdSleepMilliseconds(1000);}

    //use these 2 lines at start of thread using the prox sensors
//  messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
//	proximity_msg_t prox_values;
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void) {chSysHalt("Stack smashing detected");}
