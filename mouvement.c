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

static struct movement{

	// Determines current turning direction. Determines general side to get arround the obstacle
	enum {
		LEFT=-1,
		RIGHT=1
	} turn_direction;

} movement_info;

void halt(void){  //Stop the motors

	left_motor_set_speed(0);
	right_motor_set_speed(0);
}

void go_forward(void){  //advance at a constant speed

	left_motor_set_speed(SPEED);
	right_motor_set_speed(SPEED);
}

void advance_distance(uint16_t distance){  //advance at a given distance [mm]

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

void turn_to(float angle){	//allows to rotate the robot by a given angle

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

uint8_t tof_adjust(void){		//allows to turn until the TOF sensor detects under a certain threshold
	if(VL53L0X_get_dist_mm() < TOF_FAR){
		return TRUE;
	}
	return FALSE;
}

uint8_t obstacle_left(void){	//uses the IR sensors to detect an obstacle on its left
	if(get_prox(IR6) > DETECTION_DISTANCE
			|| get_prox(IR7) > DETECTION_DISTANCE){
		return TRUE;
	}
	return FALSE;
}

uint8_t obstacle_on_front(void){	//uses the TOF to detect an object in front

	//chprintf((BaseSequentialStream *)&SD3, "%d\n\t", VL53L0X_get_dist_mm());
	if(VL53L0X_get_dist_mm() < TOF_MIN){
		return TRUE;
	}
	return FALSE;
}

void intersection(void){	//deals with intersections
	uint8_t temp_color;
	uint8_t target_color = get_target_color();
	bool color_found = false;

	advance_distance(DIST_INTERSECT);
	halt();

	init_movement();
	turn_to(STANDARD_TURN_ANGLE);
	temp_color = color_detection();

	if(temp_color == target_color){		//scans the color on the ground and if it matches the reference color then it continues on that path.
		halt();
		advance_distance(DIST_TO_PATH);
		color_found = true;
	}

	if(color_found == false){			//same test as above but for another color
		halt();
		turn_to(2*STANDARD_TURN_ANGLE);
		temp_color = color_detection();
		if(temp_color == target_color){
			halt();
			advance_distance(DIST_TO_PATH);
			color_found=true;
		}
	}
}

static THD_WORKING_AREA(waAvoidObstacle, 128);
static THD_FUNCTION(AvoidObstacle, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	while(1) {

		init_movement();

		if(obstacle_on_front()){					//initializes obstacle detection if TOF picks up a signal under a threshold
			halt();									//Rotates to be parallel with the obstacle
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
			while(obstacle_left()){					//Begins passing by the obstacle
				go_forward();
			}
			halt();
			advance_distance(BYPASS_CORRECTION);
			turn_to(STANDARD_TURN_ANGLE); 			//90° angle to be parallel to the side of the object
			advance_distance(BYPASS_CORRECTION);	//previously DISTANCE
			while(obstacle_left()){					//begins passing by the obstacle
				go_forward();
			}
			halt();
			advance_distance(BYPASS_CORRECTION);
			turn_to(STANDARD_TURN_ANGLE);			// 90° angle to be parallel to the back of the object
			advance_distance(GRD_DIST);
			init_movement();
			turn_to(STANDARD_TURN_ANGLE);			//90° angle to go back to its initial trajectory
		}
		chThdSleepMilliseconds(100);
	}
}

void avoid_start(void){
	//has a higher priority compared to the ProcessImage thread to avoid hitting an obstacle, which is more important
	chThdCreateStatic(waAvoidObstacle, sizeof(waAvoidObstacle), NORMALPRIO+1, AvoidObstacle, NULL);
}
