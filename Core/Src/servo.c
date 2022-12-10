/*
 * servo.c
 *
 *  Created on: Mar 22, 2020
 *      Author: dans
 */

#include "servo.h"
#include "main.h"
extern long last_action_time;


/*
 *	we want resolution of 1 deg (alittle better than that in practice) so we need resolution of 2000 (1999) in the timer period
 *	we want a total of 50hz signal, so (clk_freq)/(prescaler* 2000) = 50hz, in our case, prescaler = 479
 *
 */
void servo_init(servo_controller *motor, TIM_HandleTypeDef * htim, uint32_t Channel, int16_t offset,
		 GPIO_TypeDef * PORT, uint16_t PIN)
{
	motor->htim = htim;
	motor->Channel = Channel;
	motor->offset = offset;

	motor->PORT = PORT;
	motor->PIN = PIN;

	motor->FAULTS = 0x00;

	HAL_TIM_PWM_Start(htim, Channel);

	__HAL_TIM_SET_COMPARE(motor->htim, motor->Channel, 0);
}

void servo_move(servo_controller *motor, int loc)
{
	// (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	// (loc - 0) * (220 - 70) / (180 - 0) + 70;
	int mapped = (float)loc * (SERVO_OUT_MAX - SERVO_OUT_MIN) / 180.0 + SERVO_OUT_MIN; // TODO use config.h to easily change servo scale

	// TODO: enter limits while still allowing servo release

	HAL_GPIO_WritePin (motor->PORT, motor->PIN, GPIO_PIN_SET);
	__HAL_TIM_SET_COMPARE(motor->htim, motor->Channel, mapped);
}

void servo_release(servo_controller *motor)
{
	__HAL_TIM_SET_COMPARE(motor->htim, motor->Channel, 0);
//	HAL_GPIO_WritePin (motor->PORT, motor->PIN, GPIO_PIN_RESET);
}

//char calibrate_servo_offset(servo_controller *servo_pointer, uint16_t delay, uint8_t open, uint8_t movement, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
//{
//	uint16_t _loc_movement = movement-30;
//
//	servo_move(servo_pointer, open + movement/2);
//
//	HAL_Delay(delay);
//
//	while (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin))
//	{
//		_loc_movement++;
//		servo_move(servo_pointer, open + movement/2);
//		HAL_Delay(delay);
//		servo_move(servo_pointer, open + _loc_movement);
//		HAL_Delay(delay);
//		last_action_time = HAL_GetTick();
//	}
//
//	return _loc_movement - movement;
//}
//
//char cal_caps(servo_controller *servo_pointer, uint8_t up, uint8_t movement, int8_t _caps_offset, int8_t direction)
//{
//	if (direction == 0)
//		return _caps_offset;
//	else
//		_caps_offset = (direction > 0) ? _caps_offset + 1 : _caps_offset - 1;
//
//	carriage_backward();
//	servo_move(servo_pointer, up + movement/4 + _caps_offset);
//	HAL_Delay(350);
//	servo_move(servo_pointer, up + _caps_offset);
//	HAL_Delay(350);
//
//	return _caps_offset;
//}
