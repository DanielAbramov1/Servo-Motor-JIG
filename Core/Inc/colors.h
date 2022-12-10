/*
 * colors.h
 *
 *  Created on: Sep 3, 2020
 *      Author: dans
 */

#ifndef INC_COLORS_H_
#define INC_COLORS_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "stm32f0xx_hal.h"
// ---------------------------------------- colors

#define scale 0.1

/*
#define caps_led1 0
#define caps_led2 1
#define joint_led 2
#define door_led 3
#define touch_led 4


#define green 3,229,155

#define red 242,29,129
#define red2 242*scale,29*scale,129*scale

//
#define color_done 0,255,127
#define color_done2 127*scale,255*scale,212*scale
//
#define color_work 30,144,255
#define color_work2 30*scale,144*scale,255*scale
*/
// -----------------------------------------------------------------------------


#define white 255, 170, 90
#define white2 255, 170, 90

#define warning 220,10,10
#define warning2 220*scale,10*scale,10*scale

//#define color_sleep 13,8,4 // legacy, not used
//#define color_sleep 0,0,0 // not used
#define color_sleep_1515 25,17,9 // moved form functions.c
#define color_sleep_sideways 3,2,1 // new for testing sideways LEDs
#define off 0,0,0

extern const float fade_array[400];
extern uint8_t sine[100][3];

#ifdef __cplusplus
}
#endif
#endif /* INC_COLORS_H_ */
