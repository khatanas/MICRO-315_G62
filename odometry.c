/*
 * odometry.c
 * This library allows to compute the position and angle of the ePuck
 * It contains a thread and functions
 * INPUT: motor's encoder steps values
 * OUTPUT: x and y position of the robot. Angle of the robot. All wrt its starting position
 */

//GENERAL INCLUDES
#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

//MATERIAL
#include <selector.h>

//LIBRARY INCLUDES
#include <main.h>
#include <motors.h>
#include <odometry.h>

//GENERAL VARIABLES TO THE LIBRARY
static Position ePuck_thd = {0,0,0};

//%%%%%%%%%%%%%%%%%%%%%%%%%%% INTERNAL FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
/*
 * update position based on wheel angle position
 */
void updatePosition(Position* robot, dist_mm dist_r, dist_mm dist_l){
    robot->x += cos(robot->theta)*(dist_r + dist_l)/2;
    robot->y += sin(robot->theta)*(dist_r + dist_l)/2;
    robot->theta += (dist_r - dist_l)/(2*EPUCK_RADIUS);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% THREAD %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//The thread that computes the odometry
static THD_WORKING_AREA(waOdometry, 256);
static THD_FUNCTION(Odometry, arg) {
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    //Variables in the thread
    wpos pos_r = 0;
    wpos pos_l = 0;
    wpos prev_pos_r = 0;
    wpos prev_pos_l = 0;
	dist_mm dist_r = 0;
	dist_mm dist_l = 0;

    //To control the thread's frequency
    systime_t time = chVTGetSystemTime();

	while(1){
	    //Get the values from the encoder in [steps]
		prev_pos_r = pos_r;
		prev_pos_l = pos_l;
	    pos_r = right_motor_get_pos();
	    pos_l = left_motor_get_pos();

	    //Convert step measurement into mm
	    dist_r = (pos_r - prev_pos_r)*STP2MM;
	    dist_l = (pos_l - prev_pos_l)*STP2MM;

	    // updatePosition
	    updatePosition(&ePuck_thd, dist_r, dist_l);

	    //Reset the angles to 0 so we stay in +- 2*pi
	    if(ePuck_thd.theta > 2*M_PI){
	    	ePuck_thd.theta -= 2*M_PI;
	    }
	    else if(ePuck_thd.theta < -2*M_PI){
	    	ePuck_thd.theta += 2*M_PI;
	    }

	    // Reset
	    if (SELECTOR==0){memset(&ePuck_thd,0,3*sizeof(wrld_pos));}

	    //To control the thread's frequency
	    time = chVTGetSystemTime();
	    chThdSleepUntilWindowed(time, time + MS2ST(THD_SPEED_ODOMETRY));
	}
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PUBLIC FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
/*
 * update robot's Position
 */
void get_position(Position* robot){
	robot->x = get_xPos();
	robot->y = get_yPos();
	robot->theta = get_angle();
}

/*
 * Returns the ePuck's x position
 */
wrld_pos get_xPos(void){
	return (ePuck_thd.x);
}

/*
 * Returns the ePuck's y position
 */
wrld_pos get_yPos(void){
	return (ePuck_thd.y);
}

/*
 * Returns the ePuck's angle
 */
wrld_pos get_angle(void){
	return (ePuck_thd.theta);
}

void odometry_start(void){
	chThdCreateStatic(waOdometry, sizeof(waOdometry), NORMALPRIO, Odometry, NULL);
}
