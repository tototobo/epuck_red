#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"
#include "audio/audio_thread.h"
#include <audio/play_melody.h>
#include "sensors/proximity.h"
#include <sensors/VL53L0X/VL53L0X.h>
#include <leds.h>


//constants for the differents parts of the project
#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			40
#define ROTATION_THRESHOLD		5
#define ROTATION_COEFF			2
#define PXTOCM					1570.0f //experimental value
#define MAX_DISTANCE 			25.0f
#define ERROR_THRESHOLD			0.1f //[cm] because of the noise of the camera
#define KP						400.0f
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)
#define SPEED_VALUE				300.0f
#define ROTATION_SPEED			25
#define SOUND_FREQ 				220.0f
#define COEFF_MEAN				1.5
#define COEFF_RED				1.1
#define THRESHOLD_RED			100
#define COEFF_COLOR				0.75
#define FRONT_LED				GPIOD, 14
#define PROXIMITY_THRESHOLD   	300






/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
