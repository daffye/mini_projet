#include "ch.h"
#include <stdlib.h>
#include <orientation.h>
#include "pi_regulator.h"
#include "process_image.h"
#include "sensors/proximity.h"
#include "motors.h"


void movement_start(void){

    //has a higher priority compared to the ProcessImage thread to avoid hitting an obstacle, which is more important
	//chThdCreateStatic(waMovementControl, sizeof(waMovementControl), NORMALPRIO+1, MovementControl, NULL);
}
