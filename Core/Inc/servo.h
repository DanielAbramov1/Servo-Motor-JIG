/*
 * servo.h
 *
 *  Created on: Mar 22, 2020
 *      Author: dans
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "stm32f0xx_hal.h"

typedef struct
{
	TIM_HandleTypeDef * htim;
	uint32_t Channel;
	int16_t offset;

	GPIO_TypeDef * PORT;
	uint16_t PIN;

	uint8_t FAULTS;

}servo_controller;


/**
  * @brief  Servo status structures definition
  */
typedef enum
{
  Servo_OK       = 0x00U,
  Servo_FAULT    = 0x01U,
  Servo_BUSY     = 0x02U,
  Servo_TIMEOUT  = 0x03U
} Servo_StatusTypeDef;

extern void servo_init(servo_controller *motor, TIM_HandleTypeDef * htim, uint32_t Channel, int16_t offset,
		 GPIO_TypeDef * PORT, uint16_t PIN);

extern void servo_move(servo_controller *motor, int loc);
extern void servo_release(servo_controller *motor);

char calibrate_servo_offset(servo_controller *servo_pointer, uint16_t delay, uint8_t open, uint8_t movement, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

char cal_caps(servo_controller *servo_pointer, uint8_t up, uint8_t movement, int8_t __offset, int8_t direction);


#ifdef __cplusplus
}
#endif

#endif /* INC_SERVO_H_ */
