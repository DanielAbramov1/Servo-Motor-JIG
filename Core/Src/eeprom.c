/**
 * File: eeprom.h
 *
 * Created by: Uri Shani (uri@kaufman-rd.com)
 *
 * v. 0.1.0
 * 08.04.2021
 */

#include "eeprom.h"

extern char buf[];
extern char _offset;

extern char wmsg[6] = "PROD\r\n";
extern char rmsg[6] = {0};

const char servos[] = {
		't',
		'c',
		'd'
};

extern UART_HandleTypeDef huart2;

// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ eeprom functions
/*
 * addr = address of first byte to read
 * num_of_bytes = the number of bytes to read starting at addr
 */
int read_char_from_eeprom_address(uint16_t DevAddress, uint16_t addr, char *data, uint8_t num_of_bytes)
{
	int returnValue = HAL_BUSY;
	uint8_t i2c_buf[2];
	i2c_buf[1] = (uint8_t) (addr & 0xFF); // Page 7 in datasheet, big-Endian
	i2c_buf[0] = (uint8_t) ((addr >> 8) & 0xFF00); // Page 7 in datasheet, big-Endian

	returnValue = HAL_I2C_Master_Transmit(&hi2c1, DevAddress, i2c_buf, 2, HAL_MAX_DELAY);
	if(returnValue != HAL_OK)
	{
		return returnValue;
		 //transmit error
	}

	else
	{
		returnValue = HAL_I2C_Master_Receive(&hi2c1, DevAddress, (uint8_t *)data, num_of_bytes, HAL_MAX_DELAY);
		return returnValue;
//		if(returnValue != HAL_OK)
//		{
//			return returnValue;
//			//receive error
//		}
//		else
//		{
//			return 0;
//			//all good - the needed data is in the i2c_buf
//		}
	}
}


int write_char_to_eeprom_address(uint16_t DevAddress, uint16_t addr, char *data, uint8_t num_of_bytes)
{
	uint8_t i2c_buf[34] = {0};
	i2c_buf[1] = (uint8_t) (addr & 0xFF); // Page 7 in datasheet, big-Endian
	i2c_buf[0] = (uint8_t) ((addr >> 8) & 0xFF00); // Page 7 in datasheet, big-Endian

	for(int i = 0; i < num_of_bytes; i++)
		i2c_buf[i+2] = data[i];

	HAL_StatusTypeDef returnValue = HAL_I2C_Master_Transmit(&hi2c1, DevAddress, i2c_buf, 2 + num_of_bytes, HAL_MAX_DELAY);
	return returnValue;
	//the 2 is because of the address length, then comes the data i want to store.
//	if(returnValue != HAL_OK)
//	{
////		asm("NOP");
//		;;
//	}
}

int8_t offset_from_eeprom(uint16_t address, char servo, int8_t current_offset)
{
	char success = read_char_from_eeprom_address(EEPROMDEVADDR, address, &_offset, 1);

	if(success == 0 && _offset <= 240 && _offset >= 20)
	{
//			sprintf(buf,"Carriage offset read from EEPROM, set to: %d.\r\n", _offset - 127);
		current_offset = _offset - 127;
		sprintf(buf,"%c offset read from EEPROM, set to: %d.\r\n", servos[servo-1], current_offset);
	}
	else
//		sprintf(buf,"Could not read %c offset from EEPROM, returned status: %d.\r\n", servos[servo-1], success);
		sprintf(buf,"%c offset from EEPROM failed, set to None\r\n", servos[servo-1]);

	HAL_UART_Transmit(&huart2, (uint8_t *)&buf, strlen(buf), 0xFFFF);

	_offset = 0;

	return current_offset;
}

void eeprom_test()
{
	  uint8_t msg[100];
//	  uint8_t X = EEPROMDEVADDR+1;
	  char wmsg[] = "PROD\r\n";
	  char rmsg[32] = {0};

	  if(HAL_I2C_IsDeviceReady(&hi2c1,EEPROMDEVADDR,2,1000) == HAL_OK)
	  {
		sprintf((char*)msg,"EEPROM is ready with device address %x\r\n",EEPROMDEVADDR);
		HAL_UART_Transmit(&huart2,msg,strlen(msg),1000);
	  }

	  eepromWriteIO(EEPROMDEVADDR, 0x0000, wmsg, sizeof(wmsg)+1);
	  eepromReadIO(EEPROMDEVADDR, 0x0000, rmsg, sizeof(wmsg)+1);
	  HAL_UART_Transmit(&huart2,(uint8_t*)rmsg,sizeof(rmsg),1000);
}


bool eeprom_check_prod()
{
//	char msg[sizeof(wmsg)+1];
	bool _production = false;

	if(HAL_I2C_IsDeviceReady(&hi2c1 ,EEPROMDEVADDR ,2 ,1000) == HAL_OK)
	{
		read_char_from_eeprom_address(EEPROMDEVADDR, PROD_ADDR, rmsg, sizeof(wmsg)+1);
//		HAL_UART_Transmit(&huart2, (uint8_t*)rmsg, sizeof(rmsg), 1000);
	}

	for (int i = sizeof(wmsg); i >= 0; i--)
	{
		_production = (rmsg[i] == wmsg[i]) ? true : false;
//		asm("NOP");
	}

    if (rmsg == wmsg)
    {
    	;;
    }

	return _production;
}
