#include <stdio.h>
#include <string.h>
#include "parameter/parameter.h"
#include "motors.h"
#include "mouvement.h"
#include <math.h>
#include "selector.h"
#include "sensors/proximity.h"
#include "orientation.h"

static struct movement{

    enum {
        NO = 0,
        YES = 1
    } on_right_track;

    uint8_t obstacle_detection[2];            // 1 if there is an obstacle on : [FRONT,SIDE]
    uint8_t front_sensor;                     // Determines the current front sensor
    uint8_t side_sensor;                      // Determines the current side sensor
    uint8_t diag_sensor;                      // Determines the current diagonal sensor

    enum {
        UNDER=0,
        FORWARD=1,
        DIVERGING=2,
        BACKWARD = 3,
        CONVERGING = 4,
        OVER = 5
    } orientation;

    // Determines current turning direction, Determines general side to get arround the obstacle
    enum {
        LEFT=-1,
        CENTER=0,
		RIGHT=1
    } turn_direction, obstacle_avoiding_side;

} movement_info;

void halt(void){

    left_motor_set_speed(0);
    right_motor_set_speed(0);
}

void movement_init(void){     //Initiates some values and turns to selected direction.

    movement_info.orientation = 0;
    int selector_angle = 0;   // init_selector() returns an int

    //Changes angle format from [0 ; 360] to [-180 ; 180]
    switch(get_selector()) {
		case 0:
		case 1:
		case 2:
		case 3:
		case 4:
			selector_angle = get_selector()*FULL_PERIMETER_DEG/SELECTOR_MAX + SELECTOR_OFFSET;
			break;
		case 5:
		case 6:
		case 7:
		case 8:
		case 9:
		case 10:
		case 11:
		case 12:
		case 13:
		case 14:
		case 15:
			selector_angle = get_selector()*FULL_PERIMETER_DEG/SELECTOR_MAX - 3*SELECTOR_OFFSET;
			break;
    }

    if(selector_angle < 0){
    	movement_info.turn_direction = LEFT;
    	selector_angle -= FULL_PERIMETER_DEG/SELECTOR_MAX;
    }
    else if (selector_angle > 0) {
    	selector_angle += FULL_PERIMETER_DEG/SELECTOR_MAX;
    	movement_info.turn_direction = RIGHT;
    }
    turn_to(selector_angle);
}

void turn_to(int angle){

    left_motor_set_pos(0);
    right_motor_set_pos(0);

    // The e-puck will pivot on itself
    left_motor_set_speed(movement_info.turn_direction*MOTOR_SPEED_LIMIT/2);
    right_motor_set_speed(-movement_info.turn_direction*MOTOR_SPEED_LIMIT/2);

    // Turns until the desired angle is reached
    //while ((abs(left_motor_get_pos()) < abs((angle/FULL_PERIMETER_DEG)*NSTEP_ONE_TURN*CORRECTION_FACTOR))
    	//&& (abs(right_motor_get_pos()) < abs((angle/FULL_PERIMETER_DEG)*NSTEP_ONE_TURN*CORRECTION_FACTOR))) {
	//}
    halt();
}
