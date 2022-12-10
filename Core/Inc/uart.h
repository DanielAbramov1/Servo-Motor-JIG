/*
 * uart.h
 *
 *  Created on: Dec 7, 2021
 *      Author: dans
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include "stm32f0xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

void USER_UART_IRQHandler(UART_HandleTypeDef *huart);
void USAR_UART_IDLECallback(UART_HandleTypeDef *huart);
void setup_uart();

int IsDataAvailable();
int read_serial_all(char * rxBuffer);


//void on_packet_received();
//int string_to_int();
//void print_menu();



#endif /* INC_UART_H_ */
