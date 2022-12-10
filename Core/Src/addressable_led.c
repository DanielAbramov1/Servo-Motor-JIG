/*
 * addressable_led.c
 *
 *  Created on: Mar 22, 2020
 *      Author: dans
 */

/*
 * we want resolution of 30, so we put 29 (28 to adjust the freq) in the counter period
 * we want 800khz timer so we set up a (clk_freq)/(3*x) = 800k , x = 20 so our prescaler is 19
 * we have to add HAL_DMA_PWM_STOP_DMA(&htim16, TIM_CHANNEL_1); in the DMA1_Channel2_3_IRQHandler
 *
 */
#include "addressable_led.h"
#include "colors.h"

#define BitIsSet(reg,bit) ((reg & (1 << (bit))) != 0)

uint16_t BUF_DMA [ARRAY_LEN] = {0};
uint8_t dma_ready = 1;


void leds_init(led_controller *led, TIM_HandleTypeDef * htim, uint32_t Channel)
{
	led->htim = htim;
	led->Channel = Channel;


	for(int i = 0; i < ARRAY_LEN - DELAY_LEN - FRONT_DELAY; i++)
	{
		BUF_DMA[i+FRONT_DELAY] = LOW;
	}
}

void set_led_color(led_controller *leds, uint8_t R, uint8_t G, uint8_t B, uint16_t pos)
{
	leds->led_array[pos].r = R;
	leds->led_array[pos].g = G;
	leds->led_array[pos].b = B;
}

void set_color(uint8_t R, uint8_t G, uint8_t B, uint16_t pos)
{
	int i;
	for(i = 0; i<8; i++)
	{
		if(BitIsSet(G, 7-i))
			BUF_DMA[i+pos*24 + FRONT_DELAY] = HIGH;
		else
			BUF_DMA[i+pos*24+ FRONT_DELAY] = LOW;

		if(BitIsSet(R, 7-i))
			BUF_DMA[i+8+pos*24+ FRONT_DELAY] = HIGH;
		else
			BUF_DMA[i+8+pos*24+ FRONT_DELAY] = LOW;

		if(BitIsSet(B, 7-i))
			BUF_DMA[i+16+pos*24+ FRONT_DELAY] = HIGH;
		else
			BUF_DMA[i+16+pos*24+ FRONT_DELAY] = LOW;
	}
}

void led_display(led_controller *led)
{
	//while(!dma_ready);
	dma_ready = 0;
	HAL_TIM_PWM_Start_DMA(led->htim, led->Channel, (uint32_t*)&BUF_DMA, ARRAY_LEN);	//ARRAY_LEN
}

uint8_t bin_to_dec(uint16_t pos)
{
	int i;
	int tmp;
	uint8_t num = 0;
	for (i = 0; i < 8; i++)
	{
		tmp = 8*pos + i;
		tmp = BUF_DMA[FRONT_DELAY+ tmp];
		if (tmp == HIGH)
			tmp = 0;
		else
			tmp = 1;
		num = num*2 + tmp;
	}
	return num;
}


void DMA_Callback()
{
	dma_ready = 1;
}

void blink(led_controller *leds, uint8_t r1, uint8_t g1, uint8_t b1, uint8_t r2, uint8_t g2, uint8_t b2, uint8_t pos)
{
	if(leds->led_array[pos].state == 0)
	{
		set_color(r1, g1, b1, pos);
	}
	else
	{
		set_color(r2, g2, b2, pos);
	}

	leds->led_array[pos].state = ! leds->led_array[pos].state;
}

/**
 * @brief fade_in_response - fade addressable LEDs using exp(sin())
 * @param *leds pointer to led_controller (type)
 * @param r Red value (0-255)
 * @param g Green value (0-255)
 * @param b Blue value (0-255)
 * @param start addressable position to start fading from
 * @param end addressable position to stop fading to
 */
//void fade_in_response(led_controller *leds, uint8_t r, uint8_t g, uint8_t b, uint8_t start, uint8_t end)
//{
//    for (uint16_t i = 0; i < 400; i++)
//    {
////        float val = (exp(sin(((float)i+100.0)/2000.0*M_PI*10.0)) - 0.36787944);
//
//        int r_new = fade_array[i] * (float)r / 2.5;
//        int g_new = fade_array[i] * (float)g / 2.5;
//        int b_new = fade_array[i] * (float)b / 2.5;
//        for (uint8_t pos = start; pos < end; pos++)
//        	set_color(r_new, g_new, b_new, pos);
//
//        HAL_Delay(2);
////        delay_us(500);
//		led_display(leds);
//    }
//    return;
//}

