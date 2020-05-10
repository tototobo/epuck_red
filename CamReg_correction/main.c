/*

File    : main.c
Author  : Thomas Bonnaud & Louis Rosset
Date    : 10 may 2020

Initialize the robot's components and starts the different threads

*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <motors.h>
#include <camera/po8030.h>
#include <chprintf.h>
#include <audio/audio_thread.h>
#include <audio/play_melody.h>
#include "sensors/proximity.h"
#include <spi_comm.h>
#include <leds.h>

#include <move.h>
#include <process_image.h>
#include <obstacle.h>


messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

// send information to plot python's graph
void SendUint8ToComputer(uint8_t* data, uint16_t size) 
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}


int main(void)
{

	//INITIALISATION
    halInit();
    chSysInit();
    mpu_init();
    messagebus_init(&bus, &bus_lock, &bus_condvar);
	//inits the motors
	motors_init();

	//COMMUNICATION
    //starts the serial communication
    serial_start();
    //start the USB communication
    usb_start();
    //start the SPI communication
    spi_comm_start();

    //CAMERA
    //starts the camera
    dcmi_start();
	po8030_start();
	//enable auto white balance
	po8030_set_awb(1);

	//SENSORS
	// Starts the proximity measurement module
	proximity_start();
	calibrate_ir();

	//AUDIO
	// Powers ON the alimentation of the speaker
	dac_power_speaker(true);
	dac_start();
	//creates the Melody thread
	playMelodyStart();

	//THREADS
	//stars the threads for the pi regulator and the processing of the image
//	obstacle_start();
//	pi_regulator_start();
	process_image_start();

	//disable auto white balance
	po8030_set_awb(0);

    /* Infinite loop. */
    while (1) {
    	//waits 1 second
        chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
