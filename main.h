#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

//%%%%%%%%%%%%%%%%%%% TYPEDEF %%%%%%%%%%%%%%%%%%%
typedef int32_t wpos;			//position des roues
typedef int16_t wspeed;			//vitesse rotation des roues
typedef float wrld_pos;			//postion robot dans le référentiel "world"
typedef int16_t dist_ir;		//distance mesurée par IR
typedef int32_t dist_mm;		//distance parcourue par les roues
typedef uint8_t count;			//compteur de cycle/occurance

typedef struct{
	wrld_pos x;					//position x
	wrld_pos y;					//position y
	wrld_pos theta;				//orientation
}Position;

//%%%%%%%%%%%%%%%%%%% THD_SPEED %%%%%%%%%%%%%%%%%%%
#define THD_SPEED_PROCESS_IMG 	10 // 1000/10 = 100Hz
#define THD_SPEED_ODOMETRY	 	100 // 1000/100 = 10Hz
#define THD_SPEED_AVOIDANCE		100 // 1000/100 = 10Hz

//%%%%%%%%%%%%%%%%%%% FUNCTIONS %%%%%%%%%%%%%%%%%%%
#define SELECTOR 				get_selector()

//%%%%%%%%%%%%%% EPUCK CHARACTERISTICS %%%%%%%%%%%%
#define EPUCK_RADIUS 			26.5 //[mm]
#define WHEEL_DIA 				45 //[mm]

//%%%%%%%%%%%%%% WORLD CHARACTERISTICS %%%%%%%%%%%%
#define WIDTH					175 // [mm]
#define EAST_WALL 				WIDTH //[mm]
#define WEST_WALL 				(-WIDTH) //[mm]
#define NORTH_WALL 				WIDTH //[mm]
#define SOUTH_WALL 				(-WIDTH) //[mm]

//%%%%%%%%%%%%%%%%%%% UNIT CONVERSION %%%%%%%%%%%%%
#define STEPPERTURN 			1000
#define STP2MM 					((M_PI*WHEEL_DIA)/STEPPERTURN) //[step] --> [mm]
#define MS2S 					0.001  //[ms]-->[s]
#define RAD2DEG 				57.2958 //[rad]-->[deg]

/** Robot wide IPC bus. */
extern messagebus_t bus;
extern parameter_namespace_t parameter_root;
void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
