/*

File    : main.h
Author  : Thomas Bonnaud & Louis Rosset
Date    : 10 may 2020

Initialize the robot's components and starts the different threads

*/

#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"
#include "audio/audio_thread.h"
#include "audio/play_melody.h"
#include "sensors/proximity.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include "leds.h"


//constants for the differents parts of the project
//process_image constants
#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			40
#define COEFF_COLOR				0.75 //experimental value
#define THRESHOLD_RED			100  //experimental value

//move constants //experimental values
#define ROTATION_THRESHOLD		30
#define ROTATION_COEFF			2
#define KP						0.7
#define KI						0.5
#define MAX_SUM_ERROR			30
#define SPEED_VALUE				450.0f
#define ROTATION_SPEED			25
#define THRESHOLD_DISTANCE_RED	50 //[mm]
#define ROT_OBST_COEFF			4
#define SPEED_OBST_COEFF			3
#define PERIOD					100

//obstacle constants
#define PROXIMITY_THRESHOLD   	20 //[mm]

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
