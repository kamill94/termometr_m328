/*
 * sim800.c
 *
 *  Created on: 19 sie 2017
 *      Author: Admin
 */


#include "sim800.h"
#include <avr/io.h>
#include "uart.h"
#include <util/delay.h>
#include "common.h"
#include "lcdpcf8574.h"
#include "stdlib.h"

extern CFG ram_cfg;
extern float temp;
extern char lcdbuffer [];


uint8_t sim800_func_count = 0;
uint8_t sim800_signal;


void sim800_init(void)
    {
	//_delay_ms(1000);
    _delay_ms(10000);
    uart_puts_p(PSTR("ATE0\r\n"));
    _delay_ms(200);
    uart_puts_p(PSTR("AT+CLIP=1\r\n"));
    _delay_ms(200);
    uart_puts_p(PSTR("AT+CMGD=1,4\r\n"));
    _delay_ms(200);
    uart_puts_p(PSTR("AT+CMGF=1\r\n"));
    _delay_ms(200);
    uart_puts_p(PSTR("AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r\n"));
    _delay_ms(200);
    uart_puts_p(PSTR("AT+SAPBR=3,1,\"APN\",\"internet\"\r\n"));
    _delay_ms(200);
    }

void sim800_reset(void)
{
	PORTC &= ~(1<<PC3);
	_delay_ms(100);
	PORTC |= (1<<PC3);

}

void sim800_call(void)
{
	uart_puts_p(PSTR("ATD"));
	uart_puts(ram_cfg.nrtel);
	uart_puts_p(PSTR(";\r\n"));
}

void sim800_disconnect (void)
{
	uart_puts_p(PSTR("ATH\r\n"));
}


void sim800_get_data (void)
{
	switch (sim800_func_count)
	{
	case 0:
		uart_puts_p(PSTR("AT+CSQ\r\n"));
		break;
	case 1:
		break;
	}

	sim800_func_count++;
	if(sim800_func_count > 1) sim800_func_count = 0;
}

void sim800_send_sms (void)
{
	_delay_ms(300);
	uart_puts_p(PSTR("\r\n"));
	uart_putc(13);
	_delay_ms(100);
	uart_puts_p(PSTR("AT+CMGS=\""));
	uart_puts(ram_cfg.nrtel);
	uart_putc('\"');
	uart_putc(0x0D);
	_delay_ms(700);
	dtostrf(temp,3,1,lcdbuffer);
	uart_puts_p(PSTR("Temperatura: "));
	uart_puts(lcdbuffer);
	uart_puts(" C");
	uart_putc(0x1A);
	_delay_ms(2000);

}

void sim800_send_data_to_web (void)
{
	_delay_ms(300);
	uart_puts_p(PSTR("\r\n"));
	lcd_gotoxy(11,0);
	lcd_puts_p(PSTR(" GPRS"));
	uart_puts_p(PSTR("AT+SAPBR=1,1\r\n"));
	_delay_ms(5000);
	uart_puts_p(PSTR("AT+HTTPINIT\r\n"));
	_delay_ms(100);
	uart_puts_p(PSTR("AT+HTTPSSL=1\r\n"));
	_delay_ms(100);
	uart_puts_p(PSTR("AT+HTTPPARA=\"CID\",1\r\n"));
	_delay_ms(100);
	//uart_puts_p(PSTR("AT+HTTPPARA=\"URL\",\"https://api.thingspeak.com/update?api_key=3UY74VC0NTPC5SMJ&field1="));
	uart_puts_p(PSTR("AT+HTTPPARA=\"URL\",\"https://api.thingspeak.com/update?api_key="));
	uart_puts(ram_cfg.api);
	uart_puts_p(PSTR("&field1="));
	dtostrf(temp,3,1,lcdbuffer);
	uart_puts(lcdbuffer);
	uart_puts_p(PSTR("\"\r\n"));
	_delay_ms(100);
	uart_puts_p(PSTR("AT+HTTPACTION=0\r\n"));
	_delay_ms(5000);
	uart_puts_p(PSTR("AT+HTTPREAD\r\n"));
	_delay_ms(1000);
	uart_puts_p(PSTR("AT+HTTPTERM\r\n"));
	_delay_ms(100);
	//uart_puts_p(PSTR("AT+SAPBR=0,1\r\n"));  //Rozlaczenie GPRS
	//_delay_ms(100);
}
