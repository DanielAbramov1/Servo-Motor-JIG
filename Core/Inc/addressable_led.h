/*
 * addressable_led.h
 *
 *  Created on: Mar 22, 2020
 *      Author: dans
 */

#ifndef INC_ADDRESSABLE_LED_H_
#define INC_ADDRESSABLE_LED_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "stm32f0xx_hal.h"


#define DELAY_LEN 64
#define FRONT_DELAY 8
#define LED_COUNT 12
#define ARRAY_LEN (FRONT_DELAY + LED_COUNT*24 + DELAY_LEN)


#define HIGH 35		//42
#define LOW 15

typedef struct
{
	uint8_t r,g,b;
	uint8_t state;

}single_led;


typedef struct
{
	TIM_HandleTypeDef * htim;
	uint32_t Channel;

	single_led led_array[LED_COUNT];

}led_controller;




extern void leds_init(led_controller *led, TIM_HandleTypeDef * htim, uint32_t Channel);
extern void set_color(uint8_t R, uint8_t G, uint8_t B, uint16_t pos);
extern void led_display(led_controller *led);

void set_led_color(led_controller *leds, uint8_t R, uint8_t G, uint8_t B, uint16_t pos);


extern uint8_t bin_to_dec(uint16_t pos);
extern void test_array();
extern void DMA_Callback();

void blink(led_controller *leds, uint8_t r1, uint8_t g1, uint8_t b1, uint8_t r2, uint8_t g2, uint8_t b2, uint8_t pos);

//void fade_in_response(led_controller *leds, uint8_t r, uint8_t g, uint8_t b, uint8_t start, uint8_t end);

#ifdef __cplusplus
}
#endif

#endif /* INC_ADDRESSABLE_LED_H_ */
