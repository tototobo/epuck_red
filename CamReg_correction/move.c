#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <move.h>
#include <process_image.h>
#include <obstacle.h>
#include "sensors/proximity.h"

//PASO_DOBLE

//En er Mundo - Paso Doble
static const uint16_t paso_doble_melody[] = {
  0,
  NOTE_E4, NOTE_F4, NOTE_G4, NOTE_F4, NOTE_E4, NOTE_D4,
  NOTE_E4, NOTE_F4, NOTE_G4, NOTE_F4, NOTE_E4,

};

//Paso doble tempo
static const float paso_doble_tempo[] = {
  5,
  10, 10, 10, 10, 10, 10,
  10, 20, 40, 20, 5,
};

static const melody_t paso_doble={
		//PASO_DOBLE

		  .notes = paso_doble_melody,
		  .tempo = paso_doble_tempo,
		  .length = sizeof(paso_doble_melody)/sizeof(uint16_t),
};


/**
 *  Defines the robot speed according to his distance from the red object
 * 	Returns the speed
 * 	distance : space between the red object and the robot
 */
//simple P regulator implementation
int16_t p_regulator(float distance){

	float speed = 0;

	//disables the P regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and 
	//the camera is a bit noisy
	if(fabs(distance) < ERROR_THRESHOLD){
		return 0;
	}

	speed = KP * distance;

    return (int16_t)speed;
}


static THD_WORKING_AREA(waPRegulator, 256);
static THD_FUNCTION(PRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    int16_t speed = 0;
    uint32_t speed_correction = 0;
    uint8_t rgb_state = 0, rgb_counter = 0;
    int8_t direct_rot=-1;

    systime_t time;

    			while(1){
					time = chVTGetSystemTime();

					set_body_led(0);

					if(get_obstacle()){
					//motor reaction if a obstacle
						clear_leds();
						set_body_led(1);
						switch (get_obstacle()){
							case 1:
							case 8:
							speed = -SPEED_VALUE;
							speed_correction = 0;
								break;

							case 2:
							case 3:
							speed = SPEED_VALUE/2;
							speed_correction = -4*ROTATION_SPEED;
								break;

							case 6:
							case 7:
							speed = SPEED_VALUE/2;
							speed_correction = 4*ROTATION_SPEED;
								break;

							case 4:
							case 5:
							speed = SPEED_VALUE;
							speed_correction = 0;
								break;
						}
					}
					//if no line and no obstacle in front, turn around
					else if (!get_line_position()){
			            switch(rgb_state) {
							case 0: // Red.
								set_rgb_led(0, 10, 0, 0);
								set_rgb_led(1, 10, 0, 0);
								set_rgb_led(2, 10, 0, 0);
								set_rgb_led(3, 10, 0, 0);
								break;
							case 1: // White.
								set_rgb_led(0, 10, 10, 10);
								set_rgb_led(1, 10, 10, 10);
								set_rgb_led(2, 10, 10, 10);
								set_rgb_led(3, 10, 10, 10);
								break;
			            }
						rgb_counter++;
						if(rgb_counter == 100) {
							rgb_counter = 0;
							rgb_state = (rgb_state+1)%2;
						}

						speed=0;
						speed_correction = direct_rot*ROTATION_SPEED;
//						dac_stop();
//						stopCurrentMelody();
					}

					//if no obstacle or obstacle in front but with red line, go for the red
					if(get_line_position() && !(1<get_obstacle() && get_obstacle()<8)){
						//Switch on RGB LED in red
						set_rgb_led(0, 10, 0, 0);
						set_rgb_led(1, 10, 0, 0);
						set_rgb_led(2, 10, 0, 0);
						set_rgb_led(3, 10, 0, 0);
						playMelody(EXTERNAL_SONG, ML_SIMPLE_PLAY, &paso_doble);
//						waitMelodyHasFinished();

						if(get_line_position()<(IMAGE_BUFFER_SIZE/2)){
							direct_rot=-1;
						}
						else{
							direct_rot=1;
						}

						//computes the speed to give to the motors
						//distance_cm is modified by the image processing thread
//						speed = p_regulator(get_distance_cm());
						speed = SPEED_VALUE;
						//computes a correction factor to let the robot rotate to be in front of the line
						speed_correction = (get_line_position() - (IMAGE_BUFFER_SIZE/2));

//						//if the line is nearly in front of the camera, don't rotate
//						if(abs(speed_correction) < ROTATION_THRESHOLD){
//							speed_correction = 0;
//						}
					}

					//applies the speed from the P regulator and the correction for the rotation

					right_motor_set_speed (speed - ROTATION_COEFF * speed_correction);
					left_motor_set_speed (speed + ROTATION_COEFF * speed_correction);


					//100Hz
					chThdSleepUntilWindowed(time, time + MS2ST(10));
    			}
}

void pi_regulator_start(void){
	chThdCreateStatic(waPRegulator, sizeof(waPRegulator), NORMALPRIO, PRegulator, NULL);
}
