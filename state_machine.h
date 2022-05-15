#ifndef AVOIDANCE_H
#define AVOIDANCE_H

#include <main.h>

//%%%%%%%%%%%%%%%%%%%% GENERAL %%%%%%%%%%%%%%%%%%%%%%%%
#define IR1						dist_values[0]
#define IR2						dist_values[1]
#define IR3						dist_values[2]
#define IR4						dist_values[3]
#define IR5						dist_values[4]
#define IR6						dist_values[5]
#define IR7						dist_values[6]
#define IR8						dist_values[7]

#define LARGEST		 			largest(dist_values,PROXIMITY_NB_CHANNELS, &idx)

//%%%%%%%%%%%%%%%%%%%% State machine %%%%%%%%%%%%%%%%%%%%
#define SPEED_AVOID 			300				// state = Roam
#define SPEED_WALL				200				// state = Wall
#define SPEED_TURN				150				// state = Place
#define SPEED_FOLLOW			250				// state = Follow

#define THRESHOLD_IR			7				// To filter out sensor noise
#define THRESHOLD_WALL			970				// Wall detected
#define THRESHOLD_PICTURE		40				// Picture in front of robot
#define THRESHOLD_ANGLE			(M_PI/4)		// Subdivision of the map into 8 zones; 8*pi/4 = 2PI
#define THRESHOLD_OVERSHOOT		10				// To ensure max value of IR_REF found

#define CLOSE_TO_WALL			(ePuck.x>EAST_WALL || ePuck.x<WEST_WALL || ePuck.y>NORTH_WALL || ePuck.y<SOUTH_WALL)
#define WALL_DETECTED			(LARGEST > THRESHOLD_WALL)
#define GOOD_TO_GO				(abs(IR_REF - IR_max) > THRESHOLD_OVERSHOOT)
#define PICTURE					(IR1 > THRESHOLD_PICTURE)

//%%%%%%%%%%%%%%%%%%%% Avoidance: ROAM %%%%%%%%%%%%%%%%%%%%
#define W_FRONT					96				// poids w1 (dans rapport)
#define W_SFRONT				64				// poids w2
#define W_SIDE					32				// poids w3
#define W_BACK					0

//%%%%%%%%%%%%%%%%%%%% Regulation: FOLLOW %%%%%%%%%%%%%%%%%%%%
#define IR_REF					IR6
#define IR_TARGET				IR7
#define KP_F	 				1.2f
#define KI_F					1.2f
#define MAX_SUM_ERROR_F			30

//%%%%%%%%%%%%%%%%%%%% Detection de ligne : REACHED %%%%%%%%%%%%%%%%%%%%
#define AVG_LINE				20				// # of required evaluation before to return guess


void stateMachine_start(void);
void stateMachine_stop(void);
void motorSetSpeed(wspeed speed_l, wspeed speed_r);

#endif /* AVOIDANCE_H */
