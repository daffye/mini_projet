#include "ch.h"
#include <stdlib.h>
#include <orientation.h>
#include "pi_regulator.h"
#include "process_image.h"
#include "sensors/proximity.h"
#include "motors.h"

static BSEMAPHORE_DECL(no_obstacle_sem, TRUE);
static BSEMAPHORE_DECL(goal_not_reached_sem, FALSE); //FALSE so MovementControl can start first

uint8_t obstacle_on_front(void){

    if(get_prox(FRONT_FRONT_RIGHT_SENSOR) > DETECTION_DISTANCE
    || get_prox(FRONT_FRONT_LEFT_SENSOR) > DETECTION_DISTANCE
    || get_prox(FRONT_LEFT_SENSOR) > DETECTION_DISTANCE
    || get_prox(FRONT_LEFT_SENSOR) > DETECTION_DISTANCE){
		return TRUE;
	}
    return FALSE;
}

static THD_WORKING_AREA(waMovementControl, 128);
static THD_FUNCTION(MovementControl, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1) {

        //waits until the PiRegulator has set up the motors
        chBSemWait(&goal_not_reached_sem);

		if(obstacle_on_front()){
            avoid_obstacle();
		}
		chBSemSignal(&no_obstacle_sem);
    }
}

void movement_start(void){

    //has a higher priority compared to the ProcessImage thread to avoid hitting an obstacle, which is more important
	chThdCreateStatic(waMovementControl, sizeof(waMovementControl), NORMALPRIO+1, MovementControl, NULL);
}
