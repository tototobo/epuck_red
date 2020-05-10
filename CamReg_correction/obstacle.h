/*

File    : main.c
Author  : Thomas Bonnaud & Louis Rosset
Date    : 10 may 2020

Detects the presence of an obstacle thanks to the IR sensors and returns its proximity.

*/
#ifndef OBSTACLE_H_
#define OBSTACLE_H_

/**
 * @brief send the sensor's number (from 1 to 8 or 0 if no obstacle) to the "move" module
 *
 */
uint8_t get_obstacle(void);

/**
 * @brief send the smallest distance recorded by the front sensors in millimeters to the "move" module
 *
 */
uint16_t get_front_sensor_value(void);

/**
 * @brief send the smallest distance recorded by the sensors in millimeters to the "move" module
 *
 */
uint16_t get_sensor_value(void);

//start the obstacle  thread
void obstacle_start(void);

#endif /* OBSTACLE_H_ */
