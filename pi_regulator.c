#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>
#include <mouvement.h>


static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	int16_t speed = 0;
	int16_t speed_correction = 0;

	go_forward();

	chThdSleepMilliseconds(500);

	while(1){
		//computes the speed to give to the motors
		//distance_cm is modified by the image processing thread
		speed = SPEED;
		//computes a correction factor to let the robot rotate to be in front of the line
		speed_correction = (get_line_position() - (IMAGE_BUFFER_SIZE/2));

		//if the line is nearly in front of the camera, don't rotate
		if(abs(speed_correction) < ROTATION_THRESHOLD){
			speed_correction = 0;
		}

		//applies the speed from the PI regulator and the correction for the rotation
		if(get_intersection_bool() != 1){		//if there is no intersection then it continues following the black line
		 	right_motor_set_speed(speed - ROTATION_COEFF * speed_correction);
		    left_motor_set_speed(speed + ROTATION_COEFF * speed_correction);
		    }

		chThdSleepMilliseconds(100);
	}
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
