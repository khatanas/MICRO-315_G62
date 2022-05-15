//GENERAL
#include "ch.h"
#include "hal.h"
#include <usbcfg.h>
#include <chprintf.h>
#include <arm_math.h>
#include <string.h>
//MATERIAL
#include <sensors/proximity.h>
#include <selector.h>
#include <motors.h>
#include <leds.h>
//LIBRARY INCLUDES
#include <main.h>
#include "state_machine.h"
#include <odometry.h>
#include <process_image.h>

extern messagebus_t bus;

//%%%%%%%%%%%%%%%%%%%%%%%% INTERNAL FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
/*
 * used to print if(show == [0-9]), avoiding panic when printing
 */
void cycleCount(count* show){
	*show +=1;
	if(*show >= 10){*show = 0;}
}

/*
 * returns largest measured IR distance, update corresponding IR index
 */
dist_ir largest(dist_ir array[], uint8_t length, uint8_t* idx){
  dist_ir max = array[0];
  for(uint8_t i=1; i<length; i++){
	  if(max < array[i]){
		*idx = i+1;
    	max = array[i];
	  }
  }
  return max;
 }

/*
 * add bias to initial motor speed to avoid obstacles while roaming
 */
void computeAvoidance(int8_t weights[], dist_ir values[], wspeed* acc_l, wspeed* acc_r){
	// reset acc
	*acc_l = 0;
	*acc_r = 0;
	// compute acc
	for(int i=0;i<PROXIMITY_NB_CHANNELS;i++){
	    *acc_l += ((int32_t)(weights[i]*values[i]))>>7;
	    *acc_r -= ((int32_t)(weights[i]*values[i]))>>7;
	}
	// filter noise
	(abs(*acc_l) < THRESHOLD_IR) && (*acc_l = 0);
	(abs(*acc_r) < THRESHOLD_IR) && (*acc_r = 0);
	// set speed
	motorSetSpeed(*acc_l+SPEED_AVOID, *acc_r+SPEED_AVOID);
}

/*
 *	returns toward which quarter of the map the robot is heading
 */
bool getOrientation(wrld_pos ePuck_angle){
	for(int i=1;i<8;i+=2){
		//if [0 < theta < 2*pi]:
		//[0 < theta < pi/4] || [2*pi/4 < theta < 3*pi/4] || [4*pi/4 < theta < 5*pi/4] || [6*pi/4 < theta < 7*pi/4] => positif
		if((i-1)*THRESHOLD_ANGLE < ePuck_angle && ePuck_angle < i*THRESHOLD_ANGLE){
			return true;
		}
		//[pi/4 < theta < 2*pi/4] || [3*pi/4 < theta < 4*pi/4] || [5*pi/4 < theta < 6*pi/4] || [7*pi/4 < theta < 8*pi/4] => negatif
		if(i*THRESHOLD_ANGLE < ePuck_angle && ePuck_angle < (i+1)*THRESHOLD_ANGLE){
			return false;
		}
		//if [0 > theta > -2*pi]:
		//[0 > theta > (-pi/4)] || [-2*pi/4 - (-3*pi/4)]... => negatif
		if(-(i-1)*THRESHOLD_ANGLE > ePuck_angle && ePuck_angle > -i*THRESHOLD_ANGLE){
			return false;
		}
		//[(-pi/4) > theta > (-2*pi/4)] || [-3*pi/4 - (-4*pi/4)]... => positif
		if(-i*THRESHOLD_ANGLE > ePuck_angle && ePuck_angle > -(i+1)*THRESHOLD_ANGLE){
			return true;
		}
	}
	return false;
}

/*
 * PI controller to follow wall until picture
 */
wspeed computeCorrection(dist_ir target, dist_ir values[], float* err, float* acc){
	*err = (float)(target-values[6]);
	*acc += *err;
	if(*acc > MAX_SUM_ERROR_F){
		*acc = MAX_SUM_ERROR_F;
	}else if(*acc < -MAX_SUM_ERROR_F){
		*acc = -MAX_SUM_ERROR_F;
	}
	return (wspeed)((KP_F)*(*err)+(KI_F)*(*acc));
}

/*
 * swap (a,b) values to perform "broad search" when close to wall
 * orientation_set param allows to swap the initial setting of (a,b) depending on angle between robot and normal of the wall
 */
void switchSpeed(wspeed* a, wspeed* b, count* occ, bool* orientation_set, bool theta_pos){
	if (!(*orientation_set)){ 				// orientation setting not set yet
		if(theta_pos){*occ = 50;} 			//by-pass following condition to switch initial setting
		*orientation_set = true;
		}
	if (*occ > 20){
		int tmp = *a;
		*a = *b;
		*b = tmp;
		*occ = 0;
	}else{*occ += 1;}
}

/*
 * turns on n LEDs, where n is the number of interval detected (i.e nbrLines-1)
 */
void line2LED(uint16_t* number_lines, float* mean_line){
	*number_lines = 0;
	*mean_line = 0;

	// avg over 10 measurement to get rid of noise
	for(count average_line = 0; average_line < AVG_LINE; average_line++){
		*number_lines = get_nbrLines();
		*mean_line += *number_lines;
	}

	 //Choose the right LED to light
	 switch((uint8_t)round(*mean_line/AVG_LINE)){
	 case 2:
		 set_led(LED1,1);set_led(LED3,0);set_led(LED5,0);set_led(LED7,0);
		 break;
	 case 3:
		 set_led(LED1,1);set_led(LED3,1);set_led(LED5,0);set_led(LED7,0);
		 break;
	 case 4:
		 set_led(LED1,1);set_led(LED3,1);set_led(LED5,1);set_led(LED7,0);
		 break;
	 case 5:
		 set_led(LED1,1);set_led(LED3,1);set_led(LED5,1);set_led(LED7,1);
		 break;
	 default:
		 set_led(LED1,0);set_led(LED3,0);set_led(LED5,0);set_led(LED7,0);
	 	break;
	 }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% THREAD %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
static thread_t *stateMachineThd;
static THD_FUNCTION(StateMachine, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time = chVTGetSystemTime();

	// messagebus variables
	messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
    proximity_msg_t thd_prox_values;

    // print and debug
    count show = 0;

	// state machine
    enum State{Set, Roam, Wall, Place, Follow, Reached};
    enum State state = Roam;

    // distance values, weights and result of dot product
    static dist_ir dist_values[PROXIMITY_NB_CHANNELS];
    static int8_t weights[PROXIMITY_NB_CHANNELS] = {-W_FRONT,-W_SFRONT,-W_SIDE,-W_BACK,W_BACK,W_SIDE,W_SFRONT,W_FRONT};
    wspeed acc_r = 0;
    wspeed acc_l = 0;

    // activation of motor
    count init = 0;
    bool activated = false;

    // wall detection
    wspeed a = SPEED_WALL, b = 0;		// speed of motors for broad search
    count swap = 0;						// swap speed for broad search
    uint8_t idx = 0;					// index of IR detecting wall
    bool theta_pos = false;				// orientation of the robot with respect to the reached wall: (true if pos., false if neg.)
    bool orientation_set = false;		// indicates if orientation already verified or not

    // place
    dist_ir IR_max = 0;					// max measurement
    dist_ir IR_target = 0;				// target for PI wall tracking

    //follow
    float err = 0;						// error for for P
    float err_acc = 0;					// error acc for I
    float corr = 0;						// result of: Kp*error + Ki*err_acc

	//For the odometry
    Position ePuck = {0, 0, 0};			// x,y,theta

    //For line detection
    uint16_t number_lines = 0;			// singleton guess
    float mean_line = 0;				// avged guess (over n = AVG_LINE measurements)

	while (1) {
		// Update cycle counter (for print purpose)
		cycleCount(&show);

		// Update proximity values
		messagebus_topic_wait(prox_topic, &thd_prox_values, sizeof(thd_prox_values));			//wait for all values
		for(int i=0;i<PROXIMITY_NB_CHANNELS;i++){
		    dist_values[i] = thd_prox_values.delta[i]-thd_prox_values.initValue[i];				//update prox values
		   	if(dist_values[i]<0){dist_values[i]=0;}		    									//check positiveness
		}

		// Define state
		if (SELECTOR == 0){state = Set;}
		if (state == Set && SELECTOR == 1){state = Roam;}
		switch(state){
		case Set:
			//stops robot
			init = 0;activated = false;motorSetSpeed(0,0);

			//reset position
			memset(&ePuck, 0, 3*sizeof(wrld_pos));

			//reset PI
			IR_target = 0; IR_max = 0; err_acc = 0;

			//turns LEDs off
			set_led(LED1,0);set_led(LED3,0);set_led(LED5,0);set_led(LED7,0);
			break;

		case Roam:
			if(activated){
				computeAvoidance(weights, dist_values, &acc_l, &acc_r);
				get_position(&ePuck);
				if(CLOSE_TO_WALL){
					theta_pos = getOrientation(get_angle());
					state = Wall;
				}
			}else{
				// activate the motors after n = 10 cycles = 1s (time to remove hand)
				(init >= 10) ? (activated = true) : (init++);
			}
			break;

		case Wall:
			// Broad search
			switchSpeed(&a, &b, &swap, &orientation_set, theta_pos);
			motorSetSpeed(b,a);
			if(WALL_DETECTED){state = Place;}
			break;

		case Place:
			if (IR_REF > IR_max){
				IR_max = IR_REF;			//align
				IR_target = IR_TARGET;		//get target
			}
			motorSetSpeed(SPEED_TURN, -SPEED_TURN);
			if(GOOD_TO_GO){state = Follow;}
			break;

		case Follow:
			corr = computeCorrection(IR_target, dist_values, &err, &err_acc);
			motorSetSpeed(SPEED_FOLLOW-corr, SPEED_FOLLOW+corr);

			if(PICTURE){state = Reached;}
			break;

		case Reached:
			motorSetSpeed(0,0);
			line2LED(&number_lines, &mean_line);
			break;

		default:
			break;
		}

    	time = chVTGetSystemTime();
    	chThdSleepUntilWindowed(time, time + MS2ST(THD_SPEED_AVOIDANCE));
	}
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PUBLIC FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void stateMachine_start(void){
	static THD_WORKING_AREA(waStateMachine, 512);
    calibrate_ir();
	stateMachineThd = chThdCreateStatic(waStateMachine, sizeof(waStateMachine), NORMALPRIO, StateMachine, NULL);
}

void stateMachine_stop(void){
    chThdTerminate(stateMachineThd);
    chThdWait(stateMachineThd);
    stateMachineThd = NULL;
}

void motorSetSpeed(wspeed speed_l, wspeed speed_r){
	left_motor_set_speed(speed_l);
	right_motor_set_speed(speed_r);
}
