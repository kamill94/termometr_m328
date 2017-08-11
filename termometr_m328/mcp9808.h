/*
 * mcp9808.h
 *
 *  Created on: 6 lip 2017
 *      Author: Kamil Blasiak
 */

#define MCP9808_CONF_REG 0x01
#define MCP9808_RES_REG 0x08

#define AMO 0
#define APO 1
#define ASE 2
#define ACN 3
#define AST 4
#define ICL 5
#define WLO 6
#define CLO 7
#define SHDN 8
#define TLOWER 9
#define TUPPER 10

#define BR0 0
#define BR1 1


#include "i2cmaster.h"

unsigned char mcperr;

float MCP9808_GetTemp (uint8_t AddressByte);
void MCP9808_WriteData (uint8_t AddressByte, uint8_t Register, uint16_t Data);




