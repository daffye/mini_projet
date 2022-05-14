#include "ch.h"
#include <stdlib.h>
#include "pi_regulator.h"
#include "process_image.h"
#include "sensors/proximity.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include "motors.h"
#include <mouvement.h>
#include <main.h>
#include "chprintf.h"
#include "leds.h"


static struct movement{

	// Determines current turning direction, Determines general side to get arround the obstacle
	enum {
		LEFT=-1,
		RIGHT=1
	} turn_direction;

} movement_info;

void halt(void){

	left_motor_set_speed(0);
	right_motor_set_speed(0);
}

void go_forward(void){

	left_motor_set_speed(SPEED);
	right_motor_set_speed(SPEED);
}

void advance_distance(uint16_t distance){

	go_forward();
	left_motor_set_pos(0);
	right_motor_set_pos(0);

	while (left_motor_get_pos() < (distance/CONVERSION_CM_MM)* NSTEP_ONE_TURN / WHEEL_PERIMETER) {
	}
	halt();
}

void init_movement(void){
	movement_info.turn_direction = RIGHT;
}

void turn_to(float angle){

	left_motor_set_pos(0);
	right_motor_set_pos(0);
	// The e-puck will pivot on itself
	left_motor_set_speed(movement_info.turn_direction*SPEED);
	right_motor_set_speed(-movement_info.turn_direction*SPEED);

	// Turns until the desired angle is reached
	while ((abs(left_motor_get_pos()) < abs((angle/FULL_PERIMETER_DEG)*NSTEP_ONE_TURN*CORRECTION))
			&& (abs(right_motor_get_pos()) < abs((angle/FULL_PERIMETER_DEG)*NSTEP_ONE_TURN*CORRECTION))) {
	}
	movement_info.turn_direction = LEFT;
	halt();
}

uint8_t tof_adjust(void){
	if(VL53L0X_get_dist_mm() < TOF_FAR){
		return TRUE;
	}
	return FALSE;
}

uint8_t obstacle_right(void){
	if(get_prox(IR2) > DETECTION_DISTANCE
			|| get_prox(IR3) > DETECTION_DISTANCE){
		return TRUE;
	}
	return FALSE;
}

uint8_t obstacle_left(void){
	if(get_prox(IR6) > DETECTION_DISTANCE
			|| get_prox(IR7) > DETECTION_DISTANCE){
		return TRUE;
	}
	return FALSE;
}

uint8_t obstacle_on_front(void){

	//chprintf((BaseSequentialStream *)&SD3, "%d\n\t", VL53L0X_get_dist_mm());
	if(VL53L0X_get_dist_mm() < TOF_MIN){
		return TRUE;
	}
	return FALSE;
}

void intersection(void){
	uint8_t temp_color;
	uint8_t target_color = get_target_color();
	bool color_found = false;

	advance_distance(47);
	halt();
	//left_motor_set_speed(0);
	//right_motor_set_speed(0);

	chprintf((BaseSequentialStream *)&SD3, "90 %d\r\n");
	init_movement();
	turn_to(90);  //tourne d'un angle droit à droite
	temp_color = color_detection();
	chprintf((BaseSequentialStream *)&SD3, "boi %d\r\n");


	if(temp_color == target_color){
		left_motor_set_speed(0);
		right_motor_set_speed(0);
		advance_distance(30);
		left_motor_set_speed(0);
		right_motor_set_speed(0);
		color_found = true;
	}
	//si la couleur vue est la même que celle qu'il a enregistrée alors il fonce + break
	if(color_found == false){
		halt();
		turn_to(180); //tourne d'un angle droit à gauche
		temp_color = color_detection();
		if(temp_color == target_color){
			left_motor_set_speed(0);
			right_motor_set_speed(0);
			advance_distance(30);
			left_motor_set_speed(0);
			right_motor_set_speed(0);

			color_found=true;
		}
	}
}

static THD_WORKING_AREA(waAvoidObstacle, 128);
static THD_FUNCTION(AvoidObstacle, arg) {

	chRegSetThreadName(__FUNCTION__);
	//uint8_t left=0;
	(void)arg;

	while(1) {

		init_movement();
		//go_forward();

		if(obstacle_on_front()){
			halt();
			while(tof_adjust()){
				left_motor_set_speed(movement_info.turn_direction*SPEED);
				right_motor_set_speed(-movement_info.turn_direction*SPEED);
			}
			halt();
			advance_distance(PETITE_DIST);
			turn_to(CORRECTION_ANGLE);
			init_movement();
			advance_distance(PETITE_DIST);
			turn_to(CORRECTION_ANGLE);
			halt();
			while(obstacle_left()){
				go_forward();
			}
			halt();
			advance_distance(EPUCK_RADIUS);
			turn_to(STANDARD_TURN_ANGLE); //ca le fait tourner a droite au lieu d'aller a gauche
			advance_distance(DISTANCE);
			while(obstacle_left()){
				go_forward();
			}
			halt();
			advance_distance(EPUCK_RADIUS);
			turn_to(STANDARD_TURN_ANGLE);
			advance_distance(GRD_DIST);
			init_movement();
			turn_to(STANDARD_TURN_ANGLE);
		}
		chThdSleepMilliseconds(100);
	}
}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	//systime_t time;

	int16_t speed = 0;
	int16_t speed_correction = 0;

	right_motor_set_speed(SPEED);
	left_motor_set_speed(SPEED);

	chThdSleepMilliseconds(500);   // wait until the rotation is completed before going into the pi control speed

	while(1){
		//time = chVTGetSystemTime();
		//waits until the object detection has passed
		//chBSemWait(&no_obstacle_sem);

		//computes the speed to give to the motors
		//distance_cm is modified by the image processing thread
		//speed = pi_regulator(get_distance_cm(), GOAL_DISTANCE);
		speed = SPEED;
		//computes a correction factor to let the robot rotate to be in front of the line
		speed_correction = (get_line_position() - (IMAGE_BUFFER_SIZE/2));

		//if the line is nearly in front of the camera, don't rotate
		if(abs(speed_correction) < ROTATION_THRESHOLD){
			speed_correction = 0;
		}

		//applies the speed from the PI regulator and the correction for the rotation
		if(get_intersection_bool() != 1){
		 	right_motor_set_speed(speed - ROTATION_COEFF * speed_correction);
		    left_motor_set_speed(speed + ROTATION_COEFF * speed_correction);
		    }


		//		if ((speed == 0) ) {  // when line is found, stop the motors and start light choreography
		//		}
		//		else {
		//		    //signals that the goal has not been reached and that the object detection can start again
		//			chBSemSignal(&goal_not_reached_sem);
		//}
		chThdSleepMilliseconds(100);
	}
}


void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}

void avoid_start(void){
	//has a higher priority compared to the ProcessImage thread to avoid hitting an obstacle, which is more important
	chThdCreateStatic(waAvoidObstacle, sizeof(waAvoidObstacle), NORMALPRIO+1, AvoidObstacle, NULL);
}
