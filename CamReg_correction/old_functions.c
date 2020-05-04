/*
 * old_functions.c
 *
 *  Created on: Apr 28, 2020
 *      Author: louisrosset
 */


/*
 *  Returns the line's width extracted from the image buffer given
 *  Returns 0 if line not found
 */
uint16_t extract_line_position_width (uint8_t *buffer){

	uint16_t i = 0, width = 0;
	uint8_t wrong_line = 0, line_not_found = 0;
	uint32_t mean = 0;
	begin=0; end =0;

//	static uint16_t last_width = PXTOCM/GOAL_DISTANCE;

	//performs an average & exclude buffer[0]
	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
		mean += buffer[i];
	}
	mean /= IMAGE_BUFFER_SIZE;
	i=0 ;

	do{
		wrong_line = 0;
		//search for a begin
		while(i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
		{
		    i++;
			//the slope must at least be WIDTH_SLOPE wide and is compared
		    //to the mean of the image
		    if(buffer[i] < mean && buffer[i+WIDTH_SLOPE] > mean+20)
		    {
		        begin = i;
		        break;
		    }
//		    no begin but an end
		    else if (buffer[i] > mean+20 && buffer[i+WIDTH_SLOPE] < mean){
		    		begin = 0;
		    		end = i;
		    		break;
		    }
		}

		//if a begin was found, search for an end
		if (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && begin)
		{
		    while(i < IMAGE_BUFFER_SIZE - WIDTH_SLOPE)
		    {
		        if(buffer[i] > mean+20 && buffer[i + WIDTH_SLOPE] < mean)
		        {
		            end = i;
		            break;
		        }
		        i++;
		    }
		    //if an end was not found
		    if (i > IMAGE_BUFFER_SIZE - WIDTH_SLOPE || !end)
		    {
		        end=IMAGE_BUFFER_SIZE;
		    }
		}
		else if (!end){//if no begin was found
			line_not_found = 1;
		}

//		//if a line too small has been detected, continues the search
		if(!line_not_found && (end-begin) < MIN_LINE_WIDTH){
			i = end;
			if(i==IMAGE_BUFFER_SIZE)break;
			begin = 0;
			end = 0;
			wrong_line = 1;
		}
	}while(wrong_line);

	if(line_not_found){
		begin = 0;
		end = 0;
		no_line = 1;
	}
	else{
		no_line= 0;
		width = (end - begin);
		line_position = (begin + end)/2; //gives the line position.
	}

	//sets a maximum width or returns the measured width
	if((PXTOCM/width) > MAX_DISTANCE){
		return PXTOCM/MAX_DISTANCE;
	}else{
		return width;
	}
}


/////////////////////////////////////////////////////
//
////			Extracts only the blue pixels
//for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2)
//{
//	//extracts last 5bits of the second byte
//	//takes nothing from the first byte
//	image[i/2] = (uint8_t)(img_buff_ptr[i+1]&0x1F)<<3;
//}
//
////performs the blue average
//for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
//	mean_init_blue += image[i];
//}
//mean_init_blue /= IMAGE_BUFFER_SIZE;
//
///////////////////////////////////////////////////////
//
////Extracts only the green pixels
//for(uint16_t i = 0 ; i < (2*IMAGE_BUFFER_SIZE) ; i+=2)
//{
//	//extracts last 3bits of the first byte
//	// and first 3bits of the second byte
//	image[i/2] = ((uint8_t)(img_buff_ptr[i]&0x07)<<5)+((uint8_t)(img_buff_ptr[i+1]&0xE0)>>3);
//}
//
////performs the blue average
//for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
//	mean_init_green += image[i];
//}
//mean_init_green /= IMAGE_BUFFER_SIZE;


//		avoid pics problems

//		if(main_value!=no_line){
//			counter_no_line += 1;
//			if(counter_no_line > 3)
//			{
//				//no_line = !no_line;
//				main_value=!main_value;
//				counter_no_line=0;
//			}
//			else{
//				no_line=main_value;
//			}
//		}
//		else{
//		counter_no_line=0;
//		}


