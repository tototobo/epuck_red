#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>
#include "sensors/proximity.h"


uint16_t search_obstacle(void){

	int sensors [8];
	for (unsigned int i=0; i<8 ; i++){
	sensors [i] = get_prox(i);

//	chprintf((BaseSequentialStream *)&SDU1, "sensor_%d = %d\n\r", i , sensors [i]);
	}

	for (unsigned int i=0; i<8 ; i++){
		if(sensors[i]>300){
//		chprintf((BaseSequentialStream *)&SDU1, "obstacle = %d\n\r", i);
			return i+1;
		}
	}
	return 0;
}


//simple PI regulator implementation
int16_t pi_regulator(float distance){

	float speed = 0;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and 
	//the camera is a bit noisy
	if(fabs(distance) < ERROR_THRESHOLD){
		return 0;
	}

//	sum_error += error;

//	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
//	if(sum_error > MAX_SUM_ERROR){
//		sum_error = MAX_SUM_ERROR;
//	}else if(sum_error < -MAX_SUM_ERROR){
//		sum_error = -MAX_SUM_ERROR;
//	}

	speed = KP * distance;

    return (int16_t)speed;
}


static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    int16_t speed = 0;
    int16_t speed_correction = 0;
    uint16_t obstacle = 0;
    uint16_t distance_mm=0;
    uint8_t rgb_state = 0, rgb_counter = 0;

    systime_t time;

    			while(1){
					time = chVTGetSystemTime();

					set_body_led(0);

					//search for an obstacle in the close environment
					obstacle = search_obstacle();

					//get distance mm front sensor
					distance_mm = VL53L0X_get_dist_mm();

					if(obstacle){
					//motor reaction if a obstacle
						clear_leds();
						set_body_led(1);
						switch (obstacle){
							case 1:
							case 8:
							speed = -SPEED_VALUE;
							speed_correction = ROTATION_SPEED;
								break;

							case 2:
							case 3:
							speed = SPEED_VALUE/2;
							speed_correction = -ROTATION_SPEED;
								break;

							case 6:
							case 7:
							speed = SPEED_VALUE/2;
							speed_correction = ROTATION_SPEED;
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
						speed_correction = ROTATION_SPEED;
//						dac_stop();
						stopCurrentMelody();
					}

					//if no obstacle or obstacle in front but with red line, go for the red
					if(get_line_position() && !(1<obstacle && obstacle<8)){
						//Switch on RGB LED in red
						set_rgb_led(0, 10, 0, 0);
						set_rgb_led(1, 10, 0, 0);
						set_rgb_led(2, 10, 0, 0);
						set_rgb_led(3, 10, 0, 0);

						//computes the speed to give to the motors
						//distance_cm is modified by the image processing thread
						speed = pi_regulator(get_distance_cm());
						//computes a correction factor to let the robot rotate to be in front of the line
						speed_correction = 2*(get_line_position() - (IMAGE_BUFFER_SIZE/2));

						//if the line is nearly in front of the camera, don't rotate
						if(abs(speed_correction) < ROTATION_THRESHOLD){
							speed_correction = 0;
						}
//						dac_play(SOUND_FREQ);
						playMelody(PIRATES_OF_THE_CARIBBEAN, ML_SIMPLE_PLAY, NULL);
					}

					//applies the speed from the PI regulator and the correction for the rotation

					right_motor_set_speed (speed - ROTATION_COEFF * speed_correction);
					left_motor_set_speed (speed + ROTATION_COEFF * speed_correction);


					//100Hz
					chThdSleepUntilWindowed(time, time + MS2ST(10));
    			}
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
