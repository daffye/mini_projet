#ifndef MOUVEMENT_H
#define MOUVEMENT_H

#define IR1							0
#define IR2							1
#define IR3							2
#define IR4							3
#define IR5							4
#define IR6							5
#define IR7							6
#define IR8							7
#define STANDARD_TURN_ANGLE         90
#define DETECTION_DISTANCE          300    // Desired sensor distance detection, in delta
#define FULL_PERIMETER_DEG          360.0f     //Degrees needed for the full perimeter
#define NSTEP_ONE_TURN              1000    // number of step for 1 turn of the motor
#define DISTANCE                	50      //mm
#define CONVERSION_CM_MM            10      // Decimal difference between cm and mm
#define NSTEP_ONE_TURN              1000    // number of step for 1 turn of the motor
#define WHEEL_PERIMETER             13      // cm
#define CORRECTION					1.3
#define CORRECTION_ANGLE			21		//degre�
#define SPEED					    500
#define EPUCK_RADIUS                40      //mm
#define PETITE_DIST					20
#define MOY_DIST					24
#define GRD_DIST					130
#define TOF_MIN						85
#define TOF_FAR						100

void avoid_start(void);
void pi_regulator_start(void);

#endif /* MOUVEMENTL_H */
