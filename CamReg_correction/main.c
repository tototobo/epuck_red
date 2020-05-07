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
#include <sensors/VL53L0X/VL53L0X.h>
#include <spi_comm.h>
#include <leds.h>
#include <move.h>

#include <process_image.h>
#include <obstacle.h>

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

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
//	po8030_set_brightness(-40);
	//activate auto white balance
	po8030_set_awb(1);
//	po8030_set_rgb_gain(0xB8,0x80,0xB8);
//	po8030_set_ae(0);
//	po8030_set_exposure(0x0020, 0x00);

	//SENSORS
	//inits the distance sensor
	//VL53L0X_start();
	//	Starts the proximity measurement module
	proximity_start();
	calibrate_ir();

	//AUDIO
	//Powers ON the alimentation of the speaker
	dac_power_speaker(true);
	dac_start();
	//creates the Melody thread
	playMelodyStart();

	//THREADS
	//stars the threads for the pi regulator and the processing of the image
	obstacle_start();
	pi_regulator_start();
	process_image_start();

	//inactivate auto white balance
	po8030_set_awb(0);

    /* Infinite loop. */
    while (1) {
//    	chprintf((BaseSequentialStream *)&SDU1, "front_sensor_value=%d\n\r", get_front_sensor_value());
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
