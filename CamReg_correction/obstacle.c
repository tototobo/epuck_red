#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <main.h>
#include <obstacle.h>
#include "sensors/proximity.h"

static uint16_t obstacle = 0;

/**
 *  Defines the IR sensor which detects an obstacle in the vicinity
 * 	Returns the sensor number (from 1 to 8) or 0 if there is no obstacle
 */
uint16_t search_obstacle(void){

	int sensors [8];
	//takes all the value given by the proximity sensors and put them in a table
	for (unsigned int i=0; i<8 ; i++){
	sensors [i] = get_prox(i);
	}

	// compares the sensor's value with the threshold and returns the sensor number
	for (unsigned int i=0; i<8 ; i++){
		if(sensors[i]>PROXIMITY_THRESHOLD){
			return i+1;
		}
	}
	return 0;
}


static THD_WORKING_AREA(waObstacle, 256);
static THD_FUNCTION(Obstacle, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    		while(1){
			time = chVTGetSystemTime();

    			obstacle = search_obstacle();

			//100Hz
			chThdSleepUntilWindowed(time, time + MS2ST(10));
    		}

}

uint16_t get_obstacle(void){
	return obstacle;
}

void obstacle_start(void){
	chThdCreateStatic(waObstacle, sizeof(waObstacle), NORMALPRIO, Obstacle, NULL);
}
