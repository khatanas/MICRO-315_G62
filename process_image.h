/*
 * process_image.c
 * This library allows to process the image from the camera.
 * It is strongly base on the library used for TP4 of the MICRO-315 course
 * INPUT: image from camera
 * OUTPUT: number of lines seen by the camera
 */

#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

#define IMAGE_BUFFER_SIZE		640
#define GOAL_DISTANCE 			10.0f

#define WIDTH_SLOPE 			5
#define MIN_LINE_WIDTH 			25

#define THRESHOLD_SIDE			25

#define CAM_MEAN_COUNT 			8

void processImg_start(void);
void processImg_stop(void);
uint16_t get_linePosition(void);
uint16_t get_nbrLines(void);

#endif /* PROCESS_IMAGE_H */
