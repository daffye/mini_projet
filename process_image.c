#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>
#include "camera/dcmi_camera.h"
#include <motors.h>


#include <process_image.h>


static float distance_cm = 0;
static uint16_t line_position = IMAGE_BUFFER_SIZE/2;	//middle
static uint8_t target_color = 0;
static uint8_t intersection_bool;

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

/*
 *  Returns the line's width extracted from the image buffer given
 *  Returns 0 if line not found
 */
uint16_t extract_line_width(uint8_t *buffer){

	uint16_t i = 0, begin = 0, end = 0, width = 0;
	uint8_t stop = 0, wrong_line = 0, line_not_found = 0;
	uint32_t mean = 0;

	static uint16_t last_width = PXTOCM/GOAL_DISTANCE;

	//performs an average
	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
		mean += buffer[i];
	}
	mean /= IMAGE_BUFFER_SIZE;

	do{
		wrong_line = 0;
		//search for a begin
		while(stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
		{ 
			//the slope must at least be WIDTH_SLOPE wide and is compared
		    //to the mean of the image
		    if(buffer[i] > mean && buffer[i+WIDTH_SLOPE] < mean)
		    {
		        begin = i;
		        stop = 1;
		    }
		    i++;
		}
		//if a begin was found, search for an end
		if (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && begin)
		{
		    stop = 0;
		    
		    while(stop == 0 && i < IMAGE_BUFFER_SIZE)
		    {
		        if(buffer[i] > mean && buffer[i-WIDTH_SLOPE] < mean)
		        {
		            end = i;
		            stop = 1;
		        }
		        i++;
		    }
		    //if an end was not found
		    if (i > IMAGE_BUFFER_SIZE || !end)
		    {
		        line_not_found = 1;
		    }
		}
		else//if no begin was found
		{
		    line_not_found = 1;
		}

		//if a line too small has been detected, continues the search
		if(!line_not_found && (end-begin) < MIN_LINE_WIDTH){
			i = end;
			begin = 0;
			end = 0;
			stop = 0;
			wrong_line = 1;
		}
	}while(wrong_line);

	if(line_not_found){
		begin = 0;
		end = 0;
		width = last_width;
	}else{
		last_width = width = (end - begin);
		line_position = (begin + end)/2; //gives the line position.
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
	po8030_advanced_config(FORMAT_RGB565, 0, 150, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1){
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }
}


static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};
	uint16_t lineWidth = 0;

	uint16_t red_counter = 0;
	uint16_t blue_counter = 0;
	uint16_t yellow_counter = 0;
	uint8_t temporary_color = 0;
	uint16_t j=0;

	bool send_to_computer = true;

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		if(target_color == 0){
			for (j = 0; j < 10; ++j){
				temporary_color = color_detection();
				if (temporary_color == RED){
					red_counter = red_counter + 1;}
				else if (temporary_color == BLUE){
						blue_counter = blue_counter + 1;}
					else if (temporary_color == YELLOW){
							yellow_counter = yellow_counter + 1;}
			}
			if((red_counter>blue_counter) & (red_counter>yellow_counter)){   //verif si rouge est majoritaire
				target_color = RED;
			}
			else if((blue_counter>red_counter) & (blue_counter>yellow_counter)){ //verif si bleu est majoritaire
					target_color = BLUE;
			}
			else if((yellow_counter>blue_counter) & (yellow_counter>red_counter)){
					target_color = YELLOW;
			}
		}

		chprintf((BaseSequentialStream *)&SD3, "ref color: %d\r\n", target_color);

		//Extracts only the red pixels ==> permet voir ligne noire
		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
			//extracts first 5bits of the first byte
			//takes nothing from the second byte
			image[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;
		}

		//search for a line in the image and gets its width in pixels
		lineWidth = extract_line_width(image);

		//converts the width into a distance between the robot and the camera
		if(lineWidth){
			distance_cm = PXTOCM/lineWidth;
		}

		if(send_to_computer){
			//sends to the computer the image
			SendUint8ToComputer(image, IMAGE_BUFFER_SIZE);
		}
		//invert the bool
		send_to_computer = !send_to_computer;
		intersection_bool = intersect_detec(lineWidth);

		if(intersection_bool == 1){
			chprintf((BaseSequentialStream *)&SD3, "intersection detexted %d\r\n");
			left_motor_set_speed(0);
			right_motor_set_speed(0);
			intersection();
		}
	}
}

float get_distance_cm(void){
	return distance_cm;
}

uint16_t get_line_position(void){
	return line_position;
}

uint8_t intersect_detec(uint16_t line_width){
	if (line_width > THRESH_INTERSECT){
		return 1;
	}
	else {
		return 0;
	}
}

uint8_t get_target_color(void){
	uint8_t color;
	color = target_color;
	return color;
}

uint8_t get_intersection_bool(void){
	return intersection_bool;
}

uint8_t color_detection(void){

	uint8_t *img_buff_ptr;
//	uint8_t red[IMAGE_BUFFER_SIZE] = {0};
//	uint8_t green[IMAGE_BUFFER_SIZE] = {0};
//	uint8_t blue[IMAGE_BUFFER_SIZE] = {0};
//	uint8_t average_red = 0;
//	uint8_t average_blue = 0;
//	uint8_t average_green = 0;
	uint8_t average = 0;

	uint8_t image_R[100] = {0};
	uint8_t image_G[100] = {0};
	uint8_t image_B[100] = {0};
	uint32_t mean_R = 0, mean_G = 0, mean_B = 0;

	//waits until an image has been captured
	chBSemWait(&image_ready_sem);
	//gets the pointer to the array filled with the last image in RGB565
	img_buff_ptr = dcmi_get_last_image_ptr();

	for(uint16_t i = 0 ; i < 100 ; i++){
		image_R[i] = (img_buff_ptr[i*2] & 0b11111000);
		mean_R += image_R[i];
		image_G[i] = ((img_buff_ptr[i*2] & 0b00000111)<<5) + ((img_buff_ptr[i*2+1] & 0b11100000)>>3);
		mean_G += image_G[i];
		image_B[i] = ((img_buff_ptr[i*2+1] & 0b00011111)<<3);
		mean_B += image_B[i];
	}


	mean_R /= 100;
	mean_G /= 100;
	mean_B /= 100;
	average = (mean_R + mean_G + mean_B) /3;

	if (mean_R > average){
			return RED;
		}

		else if (mean_B > average){
			return BLUE;
		}
		else{
			return 0;
		}
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
