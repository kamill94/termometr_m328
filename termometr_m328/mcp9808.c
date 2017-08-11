/*
 * mcp9808.c
 *
 *  Created on: 6 lip 2017
 *      Author: Kamil Blasiak
 */

#include <avr/io.h>
#include "mcp9808.h"

float MCP9808_GetTemp (uint8_t AddressByte)
{
	float temp;
	uint8_t UpperByte = 0;
	uint8_t LowerByte = 0;


	mcperr = i2c_start(AddressByte<<1 & 0xFF);
	if(mcperr)
	{
		i2c_stop();
	}
	else i2c_write(0x05);

	mcperr = i2c_start(AddressByte<<1 | 0x01);
	if(mcperr)
	{
		i2c_stop();
	}
	else
	{
		UpperByte = i2c_readAck();
		LowerByte = i2c_readNak();
		i2c_stop();
	}





	if ((UpperByte & 0x80) == 0x80)
	{
	}
	if ((UpperByte & 0x40) == 0x40)
	{
	}
	if ((UpperByte & 0x20) == 0x20)
	{
	}
	UpperByte = UpperByte & 0x1F;

	if ((UpperByte & 0x10) == 0x10)
		{
		UpperByte = UpperByte & 0x0F;
		temp = (256 - ((UpperByte * 16.0) + (LowerByte / 16.0)));
		}
	else
		{
		temp = ((UpperByte * 16.0) + (LowerByte / 16.0));
		}


	return temp;
}

void MCP9808_WriteData (uint8_t AddressByte, uint8_t Register, uint16_t Data)
{
	mcperr = i2c_start(AddressByte<<1 & 0xFF);
	i2c_write(Register);
	if(Data <= 0b11111111)
	{
		i2c_write(Data & 0xFF);
	}
	else
	{
		i2c_write(Data >> 8);
		i2c_write(Data & 0xFF);
	}
	i2c_stop();

}



