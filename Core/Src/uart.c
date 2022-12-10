/*
 * uart.c
 *
 *  Created on: Dec 7, 2021
 *      Author: dans
 */

#include "uart.h"
#include <string.h>
#include "main.h"
#include <stdio.h>

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;

#define DMA_BUFFER_SIZE 255
char DMA_rx_buffer[DMA_BUFFER_SIZE];

#define CIRCULAR_BUFFER_SIZE 1023
char circular_rx_buffer[CIRCULAR_BUFFER_SIZE];

int head = 0, tail = 0;


void USER_UART_IRQHandler(UART_HandleTypeDef *huart)
{
    if(USART2 == huart2.Instance)
    {
        if(RESET != __HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE))
        {
            __HAL_UART_CLEAR_IDLEFLAG(&huart2);
//            USAR_UART_IDLECallback(huart);
            HAL_UART_DMAStop(&huart2);
			int new_data_length  = DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
			//TODO check if data_length is bigger than 255

			//circular buffer implementation
			if(head + new_data_length > CIRCULAR_BUFFER_SIZE)	//if roll over
			{
				int data_len_to_cpy = CIRCULAR_BUFFER_SIZE - head;
				memcpy(circular_rx_buffer + head, DMA_rx_buffer, data_len_to_cpy);
				memcpy(circular_rx_buffer, DMA_rx_buffer + data_len_to_cpy, new_data_length - data_len_to_cpy);
			}
			else
			{
				memcpy(circular_rx_buffer + head, DMA_rx_buffer, new_data_length);
			}

			head = (head + new_data_length) % CIRCULAR_BUFFER_SIZE;

			HAL_UART_Receive_DMA(&huart2, (uint8_t*)DMA_rx_buffer, DMA_BUFFER_SIZE);
        }
    }
}


void setup_uart()
{
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart2, (uint8_t*)DMA_rx_buffer, 255);

	memset(circular_rx_buffer,0 , sizeof(circular_rx_buffer));
}

int IsDataAvailable()
{
  return (CIRCULAR_BUFFER_SIZE + head - tail) % CIRCULAR_BUFFER_SIZE;
}

//maybe do a read_until function
int read_serial_all(char * rxBuffer)
{
	//get data from circular buffer
	int length = (CIRCULAR_BUFFER_SIZE + head - tail) % CIRCULAR_BUFFER_SIZE;
	if(length < 0)
		return -1;

	memset(rxBuffer, 0, sizeof(rxBuffer));

	if(head > tail)	//if didn't roll over yet
	{
		memcpy(rxBuffer, circular_rx_buffer + tail, head - tail);
	}
	else			//if roll over occurred
	{
		int data_len_to_cpy = CIRCULAR_BUFFER_SIZE - tail;
		memcpy(rxBuffer, circular_rx_buffer + tail, data_len_to_cpy);
		memcpy(rxBuffer + data_len_to_cpy, circular_rx_buffer, IsDataAvailable() - data_len_to_cpy);

	}

	tail = head;

	return length;
}



