#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>

#include <process_image.h>


static float distance_cm = 0;
static uint16_t line_position = IMAGE_BUFFER_SIZE/2;	//middle
static uint16_t width = 0;


//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

//extract blue and green pixels and compare mean with the red slope.
bool extract_color_pixel(uint8_t *buffer, uint16_t begin_red, uint16_t end_red, uint32_t mean_color){

	uint32_t mean_target = 0;

//	//performs an average & exclude buffer[0]
//	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
//		mean_image += buffer[i];
//	}
//	mean_image /= IMAGE_BUFFER_SIZE;

	//performs an average & exclude buffer[0]
	for(uint16_t i = begin_red ; i <= end_red ; i++){
		mean_target += buffer[i];
	}
	mean_target /= (end_red-begin_red);

	if(mean_target > COEFF_COLOR * mean_color)
	{
		return true;
	}
	else
	{
		return false;
	}
}

/*
 *  Returns the line's width extracted from the image buffer given
 *  Returns 0 if line not found
 */
uint16_t extract_line_position_width (uint8_t *buffer){

	uint16_t i = 0, last_width = 0, begin = 0, end=0;
	uint8_t line_not_found = 0, direct_end=1;
	uint32_t mean_red_object = 0;
	begin=0; end =0;
	bool color_blue= false;
	bool color_green= false;

	i=0, line_position = 0;

	while(i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE)){

		//search for a begin or an end directly
		while(i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE)){
		    i++;
			//the slope must at least be WIDTH_SLOPE wide and is compared
		    //to the mean of the image
		    if(buffer[i] < THRESHOLD_RED && buffer[i+WIDTH_SLOPE] > THRESHOLD_RED)
		    {
		        begin = i;
		        direct_end=0;
		        break;
		    }
////		    no begin but an end
		    else if (buffer[i] > THRESHOLD_RED && buffer[i+WIDTH_SLOPE] <THRESHOLD_RED && direct_end){
		    		begin = 0;
		    		end = i;
		    		break;
		    }
		}

		//if a begin was found, search for an end
		if (begin)
		{
			while(i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
		    {
		        if(buffer[i] > THRESHOLD_RED && buffer[i + WIDTH_SLOPE] < THRESHOLD_RED)
		        {
		            end = i;
		            break;
		        }
		        i++;
		    }
		    //if a begin was found and an end was not found
		    if (!end)
		    {
		        end=IMAGE_BUFFER_SIZE;
		    }
		}
		else if (!end){//if no begin was found
			line_not_found = 1;
		}

		//		//if a line too small has been detected, continues the search
		if(!line_not_found && (end-begin) < MIN_LINE_WIDTH){
			if(end==IMAGE_BUFFER_SIZE)break;
			begin = 0;
			end = 0;
		}

		//calculate mean of red intensity of the "red" object seen
		for(uint16_t i = begin ; i <= end ; i++){
			mean_red_object += buffer[i];
		}
		mean_red_object /= (end-begin);

		//if bigger object and really red, compare with other colors to see confirm if red
		if(end-begin>last_width){

			//blue analysis////////////////

			uint8_t buffer_color[IMAGE_BUFFER_SIZE] = {0};
			//gets the pointer to the array filled with the last image in RGB565
			uint8_t *img_buff_ptr_color = dcmi_get_last_image_ptr();
	//
	//		Extracts only the blue pixels
			for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2)
			{
				//extracts last 5bits of the second byte
				//takes nothing from the first byte
				buffer_color[i/2] = (uint8_t)(img_buff_ptr_color[i+1]&0x1F)<<3;
			}

			color_blue = extract_color_pixel(buffer_color, begin, end, mean_red_object);

			//green analysis////////////////////

			//Extracts only the green pixels
			for(uint16_t i = 0 ; i < (2*IMAGE_BUFFER_SIZE) ; i+=2)
			{
				//extracts last 3bits of the first byte
				// and first 3bits of the second byte
				buffer_color[i/2] = ((uint8_t)(img_buff_ptr_color[i]&0x07)<<5)+((uint8_t)(img_buff_ptr_color[i+1]&0xE0)>>3);
			}

			color_green = extract_color_pixel(buffer_color, begin, end, mean_red_object);


			if(!color_blue && !color_green){
				last_width = end-begin;
				line_position=(begin + end)/2;
			}
		}
		end = 0; begin = 0;
	}

	if(line_position){
		width = last_width;
//		chprintf((BaseSequentialStream *)&SDU1, "line_position = %d\n\r", line_position);
	}

	//sets a maximum width or returns the measured width
	if((PXTOCM/width) > MAX_DISTANCE){
		return PXTOCM/MAX_DISTANCE;
	}else{
		return width;
	}
}


static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 100, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);

	dcmi_prepare();
	//laisser un temps d'adaptation
	chThdSleepMilliseconds(10);
//	 //starts a capture
//	dcmi_capture_start();
//	//waits for the capture to be done
//	wait_image_ready();

    while(1){
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }
}


static THD_WORKING_AREA(waProcessImage, 2048);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
//	main_value=0;
//	counter_no_line= 0;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};
	uint16_t lineWidth = 0;

	bool send_to_computer = true;

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		//Extracts only the red pixels
		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
			//extracts first 5bits of the first byte
			//takes nothing from the second byte
			image[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;
		}

		////////////////////////////////////////////////////
		if(send_to_computer){
			//sends to the computer the image
			SendUint8ToComputer(image, IMAGE_BUFFER_SIZE);
		}
		//invert the bool
		send_to_computer = !send_to_computer;
		////////////////////////////////////////////////////

		//search for a line in the image and gets its width in pixels
		lineWidth = extract_line_position_width(image);

		//converts the width into a distance between the robot and the camera
		if(lineWidth){
			distance_cm = PXTOCM/lineWidth;
		}

//		chprintf((BaseSequentialStream *)&SDU1, "line = %d\n\r", !no_line);

	}
}

float get_distance_cm(void){
	return distance_cm;
}

uint16_t get_line_position(void){
	return line_position;
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
//	chThdCreateStatic(waProcessImageBlue, sizeof(waProcessImageBlue), NORMALPRIO, ProcessImageBlue, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
