/*
 * decode_packet.h
 *
 *  Created on: Dec 23, 2021
 *      Author: dans
 */

#ifndef INC_DECODE_PACKET_H_
#define INC_DECODE_PACKET_H_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>



int decode_packet(char * rx_msg, int length);
int getposition(const char *array, size_t size, char c);
void split_command(char * packet, int len, char * arg1, char* arg2);

#endif /* INC_DECODE_PACKET_H_ */
