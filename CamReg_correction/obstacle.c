#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <main.h>
#include <obstacle.h>
#include "sensors/proximity.h"

static uint8_t obstacle = 0;
static uint16_t front_sensor_value=0;

/**
 *  Defines the IR sensor which detects an obstacle in the vicinity
 * 	Returns the sensor number (from 1 to 8) or 0 if there is no obstacle
 */
uint8_t search_obstacle(void){

	uint16_t sensors [8];
	uint8_t sensor_num= 0;

	//takes all the value given by the proximity sensors and put them in a table
	for (unsigned int i=0; i<8 ; i++){
	sensors [i] = get_prox(i);
	}

	if(sensors[0]>sensors[7]){
		front_sensor_value=sensors[0];
	}
	else{
		front_sensor_value=sensors[7];
	}

	// compares the sensor's value with the threshold and returns the sensor number
	for (unsigned int i=1; i<8 ; i++){
		if(sensors[i]>sensors[sensor_num]){
			sensor_num=i;
		}
	}

	if(sensors[sensor_num]>PROXIMITY_THRESHOLD){
			return sensor_num+1;
		}
		else return 0;
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

uint8_t get_obstacle(void){
	return obstacle;
}

uint16_t get_front_sensor_value(void){
	return front_sensor_value;
}

void obstacle_start(void){
	chThdCreateStatic(waObstacle, sizeof(waObstacle), NORMALPRIO, Obstacle, NULL);
}
