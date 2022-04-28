#ifndef ORIENTATION_H
#define ORIENTATION_H

//changer commentaires
#define SELECTOR_OFFSET 	        90     //angle offset due to position of 0 on the selector wheel compared to the front
#define SELECTOR_MAX		        16      //maximum value for the selector
#define FULL_PERIMETER_DEG          360.0f     //Degrees needed for the full perimeter
#define FRONT_FRONT_RIGHT_SENSOR    0       // Sensor number for front sensor slightly to the right
#define FRONT_FRONT_LEFT_SENSOR     7       // Sensor number for front sensor slightly to the left
#define FRONT_RIGHT_SENSOR          1       // Sensor number for right sensor at 45°
#define FRONT_LEFT_SENSOR           6       // Sensor number for left sensor at -45°
#define DETECTION_DISTANCE          300    // Desired sensor distance detection, in delta
#define CORRECTION_FACTOR           1.05    // correct the angle of rotation to be more precise
#define NSTEP_ONE_TURN              1000    // number of step for 1 turn of the motor

#define TRUE                        1
#define FALSE                       0

void movement_init(void);
void avoid_obstacle(void);

#endif /* OBSTACLE_AVOID_H */
