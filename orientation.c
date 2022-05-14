#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <stdio.h>
#include <string.h>
#include "parameter/parameter.h"
#include "motors.h"
#include "mouvement.h"
#include <math.h>
#include "selector.h"
#include "sensors/proximity.h"
#include "orientation.h"
#include "process_image.h"
#include "spi_comm.h"

//void turn_to(float angle);
//void advance_distance(uint16_t distance);

//static struct movement{
//
//    enum {
//        LEFT=-1,
//		RIGHT=1
//    } turn_direction;
//
//} movement_info;




//void init_movement(void){
//	movement_info.turn_direction = RIGHT;
//}

//void turn_to(float angle){
//
//    left_motor_set_pos(0);
//    right_motor_set_pos(0);
//    // The e-puck will pivot on itself
//    left_motor_set_speed(movement_info.turn_direction*MOTOR_SPEED_LIMIT/2);
//    right_motor_set_speed(-movement_info.turn_direction*MOTOR_SPEED_LIMIT/2);
//    volatile int debug = movement_info.turn_direction;
//    // Turns until the desired angle is reached
//   /*
//    if((abs(left_motor_get_pos()) < abs((angle/FULL_PERIMETER_DEG)*NSTEP_ONE_TURN*CORRECTION_FACTOR))
//    	&& (abs(right_motor_get_pos()) < abs((angle/FULL_PERIMETER_DEG)*NSTEP_ONE_TURN*CORRECTION_FACTOR))){
//    	return 1;
//    }else{
//    	return 0;
//    }*/
//    while ((abs(left_motor_get_pos()) < abs((angle/FULL_PERIMETER_DEG)*NSTEP_ONE_TURN*CORRECTION_FACTOR))
//    	&& (abs(right_motor_get_pos()) < abs((angle/FULL_PERIMETER_DEG)*NSTEP_ONE_TURN*CORRECTION_FACTOR))) {
//	}
//    movement_info.turn_direction = LEFT;
//    halt();
//}

//void intersection(void){
//	uint8_t temp_color;
//	uint8_t target_color = get_target_color();
//	bool color_found = false;
//
//	advance_distance(47);
//	halt();
//	//left_motor_set_speed(0);
//	//right_motor_set_speed(0);
//
//	chprintf((BaseSequentialStream *)&SD3, "90 %d\r\n");
//	init_movement();
//	turn_to(90);  //tourne d'un angle droit à droite
//	temp_color = color_detection();
//	chprintf((BaseSequentialStream *)&SD3, "boi %d\r\n");
//
//
//	if(temp_color == target_color){
//		left_motor_set_speed(0);
//		right_motor_set_speed(0);
//		advance_distance(30);
//		left_motor_set_speed(0);
//		right_motor_set_speed(0);
//		color_found = true;
//	}
//	//si la couleur vue est la même que celle qu'il a enregistrée alors il fonce + break
//	if(color_found == false){
//		halt();
//		turn_to(180); //tourne d'un angle droit à gauche
//		temp_color = color_detection();
//		if(temp_color == target_color){
//			left_motor_set_speed(0);
//			right_motor_set_speed(0);
//			advance_distance(30);
//			left_motor_set_speed(0);
//			right_motor_set_speed(0);
//
//			color_found=true;
//		}
//	}
//}
//
//void forward(void){
//    left_motor_set_speed(MOTOR_SPEED_LIMIT/2);
//    right_motor_set_speed(MOTOR_SPEED_LIMIT/2);
//}

//void advance_distance(uint16_t distance){
//
//    forward();
//    left_motor_set_pos(0);
//    right_motor_set_pos(0);
//
//    while (left_motor_get_pos() < (distance/CONVERSION_CM_MM)* NSTEP_ONE_TURN / WHEEL_PERIMETER) {
//    }
//    halt();
//}
