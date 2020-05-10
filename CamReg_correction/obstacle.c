/*

File    : main.c
Author  : Thomas Bonnaud & Louis Rosset
Date    : 10 may 2020

Detects the presence of an obstacle thanks to the IR sensors and returns its proximity.

*/
#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <main.h>
#include <obstacle.h>
#include "sensors/proximity.h"

static uint8_t obstacle = 0;
static uint16_t front_sensor_value = 0;
static uint16_t sensor_value = 0;

/**
 *  Converts the sensor's value into a millimeters value
 *  Take a sensor's value
 * 	Returns a distance in millimeters
 */
uint16_t convert_distance (uint16_t sensors){
	uint16_t distance_mm = 0;

	distance_mm= (1426.6)/pow(sensors,0.725); //

	return distance_mm;
}

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

	//take the biggest front sensor value and save it
	if(sensors[0]>sensors[7]){
		front_sensor_value=convert_distance(sensors[0]);
	}
	else{
		front_sensor_value=convert_distance(sensors[7]);
	}

	// compares the sensor's value with the threshold and returns the sensor number (0 if no obstacle)
	for (unsigned int i=1; i<8 ; i++){
		if(sensors[i]>sensors[sensor_num]){
			sensor_num=i;
		}
	}

	sensor_value = convert_distance(sensors[sensor_num]);

	if(sensor_value < PROXIMITY_THRESHOLD){
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

//send the sensor's number (from 1 to 8 or 0 if no obstacle) to the "move" module
uint8_t get_obstacle(void){
	return obstacle;
}

//send the smallest distance recorded by the front sensors in millimeters to the "move" module
uint16_t get_front_sensor_value(void){
	return front_sensor_value;
}

//send the smallest distance recorded by the sensors in millimeters to the "move" module
uint16_t get_sensor_value(void){
	return sensor_value;
}

void obstacle_start(void){
	chThdCreateStatic(waObstacle, sizeof(waObstacle), NORMALPRIO, Obstacle, NULL);
}
