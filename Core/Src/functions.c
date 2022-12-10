/*
 * functions.c
 *
 *  Created on: May 29, 2022
 *      Author: Daniel
 */

#include <stdbool.h>
#include <stdio.h>
#include "uart.h"
#include "servo.h"
#include "decode_packet.h"
#include "colors.h"
#include "addressable_led.h"

extern led_controller led_indicator;
extern servo_controller servo_motor_1;

#define MAX_Presses 5
#define MAX_Retry 3
#define Long_press_duration 1000 //in milisec
#define Release_angle 300

/*
	right button in PA8 for rotor assembly
	left button is PA9 for toque check
*/

int microswitch_pin; // 1 - open position, 0 - pressed position
int right_button_pin; // 1 - open position, 0 - pressed position
int left_button_pin;// 1 - open position, 0 - pressed position

int right_button_flag;
int left_button_flag = 0;



int press_type_detection() // 1 - long press , 0-short press
{
	long press_time=0, release_time=0;
	while(1)
	{
		left_button_pin = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
		if(!left_button_pin)
		{
			press_time = HAL_GetTick();
			set_color(0, 0, 0, 1);
			led_display(&led_indicator);
			while(1)
			{
				left_button_pin = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
				if(left_button_pin)
					break;
			}
			release_time=HAL_GetTick();
			if( release_time - press_time > Long_press_duration)
				return 1;
			else
				return 0;
		}
	}
}


void motor_assy_logic()
{
	while(right_button_flag < 3)
	{
		right_button_pin = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_9);
		if(!right_button_pin && right_button_flag==0)
			right_button_flag=1;

		else if(!right_button_pin && right_button_flag==1)
			right_button_flag = 2;

		if(right_button_flag == 1) //assy mode of the rotor
		{
			set_color(255, 100, 0, 0); // Yellow led - assy mode in progress
			set_color(0, 255, 0, 1);
			led_display(&led_indicator);
		}

		if(right_button_flag == 2)
		{
			set_color(0, 255, 0, 0); // Green led - assy mode is done
			led_display(&led_indicator);
			right_button_flag=3; // to stop the loop. flag goes to 0 after torque test is done
		}

		HAL_Delay(250);
	}
}


void motor_torque_check()
{
	int press_count = 0;
	int retry_count = 0;
	while(1)
	{
		left_button_flag=press_type_detection();
		if(left_button_flag == 1)
		{
			set_color(0, 0, 255, 0);
			set_color(0, 0, 255, 1);
			led_display(&led_indicator);
			HAL_Delay(2000);
			set_color(0, 255, 0, 0);
			set_color(0, 255, 0, 1);
			led_display(&led_indicator);
			break;
		}
		if(left_button_flag == 0)
		{
			for(int i = 0;i < MAX_Presses ;i++)
			{
				servo_move(&servo_motor_1,80);
				HAL_Delay(500);
				microswitch_pin = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_10);
				if(microswitch_pin == 0)
					press_count++;
				servo_move(&servo_motor_1,0);
				HAL_Delay(500);
			}
			retry_count++;
			left_button_flag=2; // finish movement waiting for  press detection

			if(press_count == MAX_Presses) //passed mode
			{
				set_color(0, 255, 0, 1);
				led_display(&led_indicator);
				servo_move(&servo_motor_1,Release_angle);
				break;
			}

			else if(retry_count < MAX_Retry) //retry mode
			{
				set_color(255, 100, 0, 1);
				led_display(&led_indicator);
			}

			else
			{
				set_color(255, 0, 0, 1); //failure mode
				led_display(&led_indicator);
				servo_move(&servo_motor_1,Release_angle);
				break;
			}
		}
	}
	press_count = 0;
	right_button_flag = 0;
	left_button_flag = 0;
}
