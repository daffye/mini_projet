#ifndef ORIENTATION_H
#define ORIENTATION_H

#define CONVERSION_CM_MM            10      // Decimal difference between cm and mm
#define NSTEP_ONE_TURN              1000    // number of step for 1 turn of the motor
#define WHEEL_PERIMETER             13      // cm
#define FULL_PERIMETER_DEG          360.0f     //Degrees needed for the full perimeter
#define CORRECTION_FACTOR           1.05    // correct the angle of rotation to be more precise
#define DETECTION_DISTANCE          300    // Desired sensor distance detection, in delta
#define CLOSE_COEFF					10 		// correction coefficient to adjust the desired precision on the proximity sensors
#define ERROR_TOLERANCE             150      // Distance error range,  in delta
#define FAR_COEFF					3		// correction coefficient to adjust the desired precision on the proximity sensors
#define EPUCK_RADIUS                40      //mm
#define FRONT_FRONT_RIGHT_SENSOR    0       // Sensor number for front sensor slightly to the right
#define FRONT_FRONT_LEFT_SENSOR     7       // Sensor number for front sensor slightly to the left
#define FRONT_RIGHT_SENSOR          1       // Sensor number for right sensor at 45°
#define FRONT_LEFT_SENSOR           6       // Sensor number for left sensor at -45°
#define RIGHT_SENSOR                2       // Sensor number for right sensor at 90°
#define LEFT_SENSOR                 5       // Sensor number for left sensor at -90°
#define STANDARD_TURN_ANGLE         112     // Standard turning angle with a correction coefficient found experimentally (90 +360/16)
#define CLOSER                      -1      // Multiplier used for distance calculation when getting closer to the objective
#define FURTHER                     1       // Multiplier used for distance calculation when getting closer to the objective


#define TRUE                        1
#define FALSE                       0

//void movement_init(void);
void avoid_obstacle(void);

#endif /* OBSTACLE_AVOID_H */
