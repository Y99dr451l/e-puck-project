#include <stdio.h>
#include <stdlib.h>
#include "ch.h"
//#include "chmtx.h"
//#include "chcond.h"
#include "hal.h"
#include "memory_protection.h"
#include "usbcfg.h"
#include "chprintf.h"
#include "motors.h"
#include "leds.h"
#include "button.h"
#include "spi_comm.h"
#include "sensors/proximity.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include "audio/microphone.h"
#include "sensors/imu.h"
#include "sensors/mpu9250.h"
#include "main.h"
#include "communications.h"
#include "bot.h"
#include "pi_regulator.h"
#include "kalman.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

static void serial_start(void) {
	static SerialConfig ser_cfg = {115200, 0, 0, 0};
	sdStart(&SD3, &ser_cfg); // UART3.
}

//static void timer11_start(void){
//    static const GPTConfig gpt11cfg = {10000, NULL, 0, 0}; // 10KHz timer clock to measure 100us
//    gptStart(&GPTD11, &gpt11cfg);
//    gptStartContinuous(&GPTD11, 0xFFFF);
//}

static void timer12_start(void){
    static const GPTConfig gpt12cfg = {100000, NULL, 0, 0}; // 100KHz timer clock to measure 10us
    gptStart(&GPTD12, &gpt12cfg);
    gptStartContinuous(&GPTD12, 0xFFFF);
}

int main(void) { // order from src/main.c
	halInit();
	chSysInit();
	mpu_init();
	messagebus_init(&bus, &bus_lock, &bus_condvar);
	usb_start();
	motors_init();
	imu_start();
	spi_comm_start();
	VL53L0X_start();
	serial_start();
	mic_start(&audio_processing);
//	timer11_start();
	timer12_start();
	calibrate_acc();
	clear_leds(); set_body_led(0); set_front_led(0);
	bot_start();
	pi_regulator_start();
	kalman_start();
    while (1) chThdSleepMilliseconds(1000);
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void) {chSysHalt("Stack smashing detected");}
