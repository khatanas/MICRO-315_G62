/*
 * process_image.c
 * This library allows to process the image from the camera.
 * It is strongly base on the library used for TP4 of the MICRO-315 course
 * INPUT: image from camera
 * OUTPUT: number of lines seen by the camera
 */

//GENERAL INCLUDES
#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <math.h>

//MATERIAL
#include <leds.h>
#include <camera/po8030.h>

//ADD
#include <main.h>
#include <process_image.h>

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

//THREAD GENERAL VARIABLES
static uint16_t number_lines = 0;


//%%%%%%%%%%%%%%%%%%%%%%%% INTERNAL FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
/*
 * Returns the number of lines seen on an image
 */
uint16_t compute_line_number(uint8_t *buffer){
	uint16_t i = 0;
	uint16_t begin = 0;
	uint16_t end = 0;
	uint8_t stop = 0;
	uint8_t line_not_found = 0;
	uint32_t mean = 0;
	uint8_t line_counter = 0;

	//Computing the mean of the pixels' values
	for(uint32_t j = THRESHOLD_SIDE;j<IMAGE_BUFFER_SIZE-THRESHOLD_SIDE;j++){
		mean += buffer[j];
	}
	mean /= (IMAGE_BUFFER_SIZE-(2*THRESHOLD_SIDE));

	//Starting the line detection algorithm
	//i = THRESHOLD_SIDE;

	do{
		// Find beginning of line (intensity is "going down")
		while(stop == 0 && i<(IMAGE_BUFFER_SIZE - WIDTH_SLOPE)){
			// Negative slope detected
			if(buffer[i]>mean && buffer[i+WIDTH_SLOPE]<mean){
				begin = i;
				stop = i;
			}
			i++;
		}
		//If beginning found, find the end
		if(i<(IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && begin){
			stop = 0;
			//Search for positive slope
			while(stop == 0 && i < IMAGE_BUFFER_SIZE){
				if(buffer[i]>mean && buffer[i-WIDTH_SLOPE]<mean){
					end = i;
					stop = i;
				}
				i++;
			}
			// Beginning found but no end
			if (i> IMAGE_BUFFER_SIZE || !end){
				line_not_found = 1;
			}
		}
		//First iteration OR beginning of line not found
		else{line_not_found = 1;}
		//Line found with coherent width
		if(!line_not_found && ((end-begin) > MIN_LINE_WIDTH)){
			begin = end;
			end = 0;
			stop = 0;
			line_counter++;
		}
	}while(i<(IMAGE_BUFFER_SIZE - WIDTH_SLOPE));

	return line_counter;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% THREADS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
/*
 * Thread that captures the image
 */
static thread_t *captureImgThd;
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 10, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1){
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }
}

/*
 * Thread that processes the image
 */
static thread_t *processImgThd;
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	//Only the red color of the image is used as it provides better results. Other are possible
	static uint8_t image_red[IMAGE_BUFFER_SIZE] = {0};
	static float mean_img_buffer[IMAGE_BUFFER_SIZE] = {0};
	static uint8_t buffer_to_pass[IMAGE_BUFFER_SIZE] = {0};
	uint8_t mean_counter = 0;

	bool send_to_computer = true;

    //To control the thread's frequency
    systime_t time = chVTGetSystemTime();

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565
		img_buff_ptr = dcmi_get_last_image_ptr();

		//Reading the color
		for(int i=0; i<2*IMAGE_BUFFER_SIZE; i+=2){
			image_red[i/2] = img_buff_ptr[i]&0xF8;
			mean_img_buffer[i/2] += image_red[i/2];
		}
		mean_counter++;
		if(mean_counter >= CAM_MEAN_COUNT-1){
			for(uint32_t i = 0; i< IMAGE_BUFFER_SIZE; i++){
				mean_img_buffer[i] /= CAM_MEAN_COUNT;
				buffer_to_pass[i] = mean_img_buffer[i];
			}
			//Computing number of lines
			number_lines = compute_line_number(buffer_to_pass);
			//chprintf((BaseSequentialStream *)&SDU1, "\n number_lines in process %d \r\n", number_lines);

			//sends the data buffer of the given size to the computer
			if(send_to_computer){
				//To see the camera output using the scripts from TP4
				SendUint8ToComputer(buffer_to_pass, IMAGE_BUFFER_SIZE);
			}
			send_to_computer = !send_to_computer;
			mean_counter = 0;
			for(uint32_t i = 0; i< IMAGE_BUFFER_SIZE; i++){
				mean_img_buffer[i] += 0;
			}
		}


	    time = chVTGetSystemTime();
	    chThdSleepUntilWindowed(time, time + MS2ST(THD_SPEED_PROCESS_IMG));
    }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PUBLIC FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
uint16_t get_nbrLines(void){
	return number_lines;
}

void processImg_start(void){
	static THD_WORKING_AREA(waCaptureImage, 256);
	captureImgThd = chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);

	static THD_WORKING_AREA(waProcessImage, 1024);
	processImgThd = chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
}

void processImg_stop(void) {
    chThdTerminate(captureImgThd);
    chThdWait(captureImgThd);
    captureImgThd = NULL;
    chThdTerminate(processImgThd);
    chThdWait(processImgThd);
    processImgThd = NULL;

}
