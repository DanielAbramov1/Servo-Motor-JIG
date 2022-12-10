/*
 * decode_packet.c
 *
 *  Created on: Dec 23, 2021
 *      Author: dans
 */
#include <decode_packet.h>

#include "stm32f0xx_hal.h"


extern UART_HandleTypeDef huart2;


int decode_packet(char * rx_msg, int length)
{
	int tmp;
	if(length < 1 || rx_msg[0] == 0)
		return 0;


	char arg1[20] = {0}, arg2[20] = {0};
	split_command(rx_msg, length, arg1, arg2);

	char tmp_buf[50];
	memset(tmp_buf, 0, sizeof(tmp_buf));
	snprintf(tmp_buf, 50, "LEN: %d, DATA: %s\n", length, rx_msg);
	HAL_UART_Transmit(&huart2, (uint8_t *)&tmp_buf, strlen(tmp_buf), 0xFFFF);


	switch(rx_msg[0])
	{
		case '-':
			HAL_NVIC_SystemReset();
			break;

		case 'a':
		{
			//int tmp = atoi(arg1);
			tmp = atoi(arg1);
			memset(tmp_buf, 0, sizeof(tmp_buf));
			snprintf(tmp_buf, 50, "spin motor A %d\n", tmp);
			HAL_UART_Transmit(&huart2, (uint8_t *)&tmp_buf, strlen(tmp_buf), 0xFFFF);

			//spin_motor(&dc_motor_1, tmp);
		}
			break;

		case 'A':
		{
			//int tmp = atoi(arg1);
			tmp = atoi(arg1);
			memset(tmp_buf, 0, sizeof(tmp_buf));
			snprintf(tmp_buf, 50, "spin motor A %d\n", tmp);
			HAL_UART_Transmit(&huart2, (uint8_t *)&tmp_buf, strlen(tmp_buf), 0xFFFF);

			//spin_motor(&dc_motor_1, tmp);
		}
			break;


		default:
			memset(tmp_buf, 0, sizeof(tmp_buf));
			snprintf(tmp_buf, 50, "unknown command: '%c'\n", rx_msg[0]);
			HAL_UART_Transmit(&huart2, (uint8_t *)&tmp_buf, strlen(tmp_buf), 0xFFFF);
			break;
	}
	return tmp;
}

void split_command(char * packet, int len, char * arg1, char* arg2)
{
    int index = 0, prev_index = 1;

    index = getposition(packet, 0, ',');

    if(index > 0)   //if found a ','
    {
        memcpy(arg1, packet + prev_index, index-1);
        prev_index = index + 1;


        index = getposition(packet + prev_index, 0, '\r');
        if(index >= 0)   //if theres \r
        {
            memcpy(arg2, packet + prev_index, index);
        }
        else
        {
            index = getposition(packet + prev_index, 0, '\n');
            if(index >= 0)   //if theres \n and its not in the first char
            {
                memcpy(arg2, packet + prev_index, index);
            }
            else //this means no \r and no \n
            {
                memcpy(arg2, packet + prev_index, len - prev_index);
            }
        }
    }
    else    //if theres no ','
    {
        index = getposition(packet, 0, '\r');
        if(index >= 1)   //if theres \r and its not in the first char
        {
            memcpy(arg1, packet + prev_index, index-1);
        }
        else
        {
            index = getposition(packet, 0, '\n');
            if(index >= 1)   //if theres \n and its not in the first char
            {
                memcpy(arg1, packet + prev_index, index-1);
            }
            else //this means no \r and no \n
            {
                memcpy(arg1, packet + prev_index, len - prev_index);
            }
        }
    }
}

int getposition(const char *array, size_t size, char c)
{
    char *ret = strchr(array, c);
    if(ret != NULL)
        return ret - array;
    else
        return -1;
}
