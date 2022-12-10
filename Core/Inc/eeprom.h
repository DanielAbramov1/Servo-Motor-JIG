/**
 * File: eeprom.h
 *
 * Created by: Uri Shani (uri@kaufman-rd.com)
 *
 * v. 0.1.0
 * 08.04.2021
 */

#ifndef INC_EEPROM_H
#define INC_EEPROM_H

#ifdef __cplusplus
extern "C"{
#endif

#include "stm32f0xx_hal.h"
#include <string.h>
#include <stdbool.h>

extern bool _production;

extern I2C_HandleTypeDef hi2c1;

char wmsg[6];
char rmsg[6];

#define EEPROMDEVADDR 0xA0
#define ONEPAGE 32
#define PROD_ADDR 0x0000
#define CARRIAGE_OFFSET_ADDR 0x00A0
#define CAPSULE_OFFSET_ADDR 0x00B0
#define DOOR_OFFSET_ADDR 0x00C0
#define FAULT_CODE 0x00D0

int write_char_to_eeprom_address(uint16_t DevAddress, uint16_t addr, char *data, uint8_t num_of_bytes);
int read_char_from_eeprom_address(uint16_t DevAddress, uint16_t addr, char *data, uint8_t num_of_bytes);
int8_t offset_from_eeprom(uint16_t address, char servo, int8_t current_offset);

void eeprom_test(void);

bool eeprom_check_prod(void);

#ifdef __cplusplus
}
#endif

#endif /* EEPROM_H */
