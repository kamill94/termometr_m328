/*
 * main.c
 *
 *  Created on: 10 lip 2017
 *      Author: Kamil Blasiak
 */


#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>
#include "lcdpcf8574.h"
#include "uart.h"
#include "mcp9808.h"
#include "sim800.h"
#include "common.h"


int main (void)
{
	pin_init();
	uart_init(UART_BAUD_SELECT(9600, F_CPU));
	sei();
	i2c_init();
	lcd_init(LCD_DISP_ON);
	lcd_led(0);

	check_and_read_cfg();

	if(!(PIND & (1<<PD2)))
	{
		read_default_settings();
	}

	sim800_reset();
	sim800_init();
	timer0_init();
	adc_init();


	temp = MCP9808_GetTemp(MCP9808_ADDRESS);

	while(1)
	{
		if(NULL != uart_get_str(uart_receive_buffer))
		{
			uart_datatype = parsedata(uart_receive_buffer, uart_parsed_string);
		}

		if((timer0_second % 2) == 0 && f_temp)
			{
			temp = MCP9808_GetTemp(MCP9808_ADDRESS);
			sim800_get_data();
			f_temp = 0;
			}

		if(f_lcd_ref)
			{
			ADCSRA |= (1<<ADSC); // uruchomienie konwersji ADC
			battery = (((ADC*1.1)/1024)*4.33333333);

			if(battery < 3.70) battery_low();

			lcd_refresh();
			f_lcd_ref = 0;
			}


		if(f_send_web)
		{
			f_send_web = 0;
			sim800_send_data_to_web();
		}

		alarms();
		pin_check();

		if(uart_datatype == 1)
		{
			sim800_disconnect();

			if(strstr(uart_parsed_string,ram_cfg.nrtel))
			{
				timer0_lcd = LCD_TIME;
				lcd_led(0);
				lcd_clrscr();
				lcd_gotoxy(0,0);
				lcd_puts_p(PSTR("SMS"));
				_delay_ms(500);
				sim800_send_sms();
			}
			uart_parsed_string[0] = '\0';
			uart_receive_buffer[0] = '\0';

		}


	}

	return 0;
}

