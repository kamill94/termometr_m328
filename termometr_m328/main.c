/*
 * main.c
 *
 *  Created on: 10 lip 2017
 *      Author: Kamil Blasiak
 */

#define MCP9808_ADDRESS 0x18
#define CALL_TIME 20
#define DELAY_TIME 60
#define DEFAULT_TEMP_ALARM 20
#define ALARM_STOP_TIME 2700
#define SEND_DATA_TO_WEB 1
#define LCD_TIME 30

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>
#include "lcdpcf8574.h"
#include "uart.h"
#include "mcp9808.h"

uint8_t timer0_skaler = 0;
uint8_t timer0_second = 0;
uint8_t timer0_minute = 0;
uint8_t timer0_lcd = LCD_TIME;

float temp;

char lcdbuffer [16];

char uart_receive_buffer [129];
char uart_parsed_string [33];
uint8_t uart_datatype;

uint8_t sim800_func_count = 0;
int sim800_bat;
uint8_t sim800_signal;

uint8_t f_temp = 0;
uint8_t f_lcd_ref = 0;
uint8_t f_alarm = 0;
uint8_t f_alarm_stop = 0;
uint8_t f_connect = 0;
uint8_t f_disconnect = 0;
uint8_t f_delay = 0;
uint8_t f_lcdon = 0;
uint8_t f_send_web = 0;

uint8_t call_time = CALL_TIME;
uint8_t delay_time = DELAY_TIME;
signed int alarm_stop_time = ALARM_STOP_TIME;

void timer0_init(void);
void lcd_refresh (void);
void sim800_init(void);
void sim800_reset(void);
void alarms (void);
void pin_init (void);
void pin_check (void);
uint8_t parsedata(char *data, char *out);
void sim800_get_data (void);
void sim800_disconnect (void);
void sim800_send_sms (void);
void sim800_send_data_to_web (void);

ISR (TIMER0_OVF_vect)
{
	timer0_skaler++;
	if(timer0_skaler>=125)
	{
		timer0_skaler = 0;
		timer0_second++;
		f_temp=1;
		f_lcd_ref=1;

		if(timer0_lcd>0) timer0_lcd--;
		if(timer0_lcd == 0)
		{
			f_lcdon = 1;
		}
		else
		{
			f_lcdon = 0;
		}



		if(f_alarm)
		{

			if(f_connect)
			{
				call_time--;
			}

			if(call_time==0)
			{
				call_time = CALL_TIME;
				f_disconnect=1;
			}

			if(f_delay)
			{
				delay_time--;
			}

			if(delay_time==0)
			{
				delay_time = DELAY_TIME;
				f_delay=0;
			}

		}

		if(f_alarm_stop)
			{
				alarm_stop_time--;
			}

			if(alarm_stop_time<0)
			{
				alarm_stop_time = ALARM_STOP_TIME;
				f_alarm_stop = 0;
			}



		if(timer0_second>59)
			{
				timer0_second=0;
				timer0_minute++;

				if(!(timer0_minute % 2)) f_send_web = 1;
			}

	}



	TCNT0=131;
}

int main (void)
{
	pin_init();
	uart_init(UART_BAUD_SELECT(9600, F_CPU));
	sei();
	i2c_init();
	lcd_init(LCD_DISP_ON);
	lcd_led(0);
	sim800_reset();
	sim800_init();
	timer0_init();

	temp = MCP9808_GetTemp(MCP9808_ADDRESS);

	while(1)
	{

		uart_get_str(uart_receive_buffer);
		uart_datatype = parsedata(uart_receive_buffer, uart_parsed_string);

		if((timer0_second % 2) == 0 && f_temp)
			{
			temp = MCP9808_GetTemp(MCP9808_ADDRESS);
			sim800_get_data();
			f_temp = 0;
			}

		if(f_lcd_ref)
			{
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

			if(strstr(uart_parsed_string,"792682279"))
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

void timer0_init(void)
    {
    TCCR0B |= (1 << CS00) | (1 << CS02); //preskaler 1024
    TIMSK0 |= 1 << TOIE0;  //przerwanie
    TCNT0 = 131; //wartosc poczatkowa
    }

void lcd_refresh (void)
{
	lcd_led(f_lcdon);
	if(f_alarm == 0)
	{
		dtostrf(temp,3,1,lcdbuffer); // Wyswietlanie temperatury
		lcd_gotoxy(0,0);
		lcd_puts_p(PSTR("T:"));
		if(mcperr) lcd_puts_p(PSTR("BLAD"));
		else lcd_puts(lcdbuffer);


		if(!f_alarm_stop)  // Wyswietlanie pozostalego czasu blokady alarmu
			{
				lcd_puts("        ");
				dtostrf(timer0_second,-3,0,lcdbuffer);
				lcd_gotoxy(14,0);
				lcd_puts(lcdbuffer);
			}
			else
			{
				lcd_puts("     ");
				lcd_gotoxy(11,0);
				dtostrf((alarm_stop_time/60),2,0,lcdbuffer);
				lcd_puts(lcdbuffer);
				lcd_putc(':');
				dtostrf((alarm_stop_time%60),-2,0,lcdbuffer);
				if((alarm_stop_time%60) < 10) lcd_putc('0');
				lcd_puts(lcdbuffer);

			}


		lcd_gotoxy(0,1);       // Wyswietlanie stanu baterii i sygnalu sieci
		itoa(sim800_bat, lcdbuffer, 10);
		lcd_puts_p(PSTR("B:"));
		lcd_puts(lcdbuffer);
		lcd_puts_p(PSTR("mV"));

		lcd_gotoxy(11,1);
		itoa((sim800_signal*100/30), lcdbuffer, 10);
		lcd_puts("S:");
		lcd_puts(lcdbuffer);
		lcd_puts_p(PSTR("%"));


	}
	if(f_alarm == 1)
	{
		lcd_gotoxy(0,0);
		lcd_puts_p(PSTR("ALARM"));
		lcd_puts_p(PSTR("           "));
		lcd_gotoxy(0,1);
		if(mcperr) lcd_puts_p(PSTR("BLAD CZUJNIKA   "));
		else lcd_puts_p(PSTR("                "));

	}




//	lcd_gotoxy(0,1);
//	lcd_puts(uart_parsed_string);
//	lcd_puts("   ");
}

void sim800_init(void)
    {
	//_delay_ms(1000);
    _delay_ms(10000);
    uart_puts_p(PSTR("ATE0\r\n"));
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
	uart_puts_p(PSTR("ATD792682279;\r\n"));
}

void sim800_disconnect (void)
{
	uart_puts_p(PSTR("ATH\r\n"));
}

void alarms (void)
{
	if(((temp < DEFAULT_TEMP_ALARM) || mcperr) && !f_alarm && !f_alarm_stop)
		{
		f_alarm = 1;
		}

	if(f_alarm && f_connect != 1 && !f_delay)
		{
			sim800_call();
			f_connect = 1;
			timer0_lcd = LCD_TIME;
		}

	if(f_alarm && f_disconnect)
		{
		f_disconnect = 0;
		f_connect = 0;
		f_delay = 1;
		sim800_disconnect();
		}
}

void pin_init (void)
{
	DDRD &= ~(1<<PD2);
	PORTD |= 1<<PD2;

	DDRD &= ~(1<<PD3);
	PORTD |= 1<<PD3;

	DDRC |= (1<<PC3);
	PORTD |= (1<<PC3);
}

void pin_check (void)
{
	if(f_alarm && (!(PIND & (1<<PD2))))
	{
		lcd_clrscr();
		f_connect = 0;
		f_disconnect = 0;
		f_alarm_stop = 1;
		f_alarm = 0;
		sim800_disconnect();
	}

	if(!(PIND & (1<<PD2)))
		{
		timer0_lcd = LCD_TIME;
		lcd_led(0);
		}
}

uint8_t parsedata(char *data, char *out)

    {
    uint8_t datatype = 0;
    if (strstr(data, "+CLIP"))  // Odczyt dzwoni¹cego numeru telefonu
	{
	strcpy(out, data + 8);
	out[9] = 0;
	datatype = 1;
	}

    else if (strstr(data, "+CMTI")) // Odebrana wiadomosc SMS
	{
	datatype = 2;
	}

    else if (strstr(data, "NO CARRIER")) // Brak odpowiedzi
	{
	datatype = 3;
	}

    else if (strstr(data, "OK")) // OK
	{
	datatype = 4;
	}

    else if (strstr(data, "BUSY")) // Numer zajety
	{
	datatype = 5;
	}

    else if (strstr(data, "+CBC")) // Odczyt napiecia baterii
    	{
    	strcpy(out, data + 11);
    	out[4] = 0;
    	datatype = 6;
    	sim800_bat = atoi(out);
    	}

    else if (strstr(data, "+CSQ")) // Odczyt sily sygnalu sieci GSM
       	{
       	strcpy(out, data + 6);
       	out[2] = 0;
       	datatype = 7;
       	sim800_signal = atoi(out);
       	}

    else
	{
	datatype = 10;
	}


    return datatype;
    }

void sim800_get_data (void)
{
	switch (sim800_func_count)
	{
	case 0:
		uart_puts_p(PSTR("AT+CBC\r\n"));
		break;
	case 1:
		uart_puts_p(PSTR("AT+CSQ\r\n"));
		break;
	}

	sim800_func_count++;
	if(sim800_func_count > 1) sim800_func_count = 0;
}

void sim800_send_sms (void)
{
	uart_putc(13);
	uart_puts_p(PSTR("AT+CMGS=\"+48792682279\""));
	uart_putc(0x0D);
	_delay_ms(500);
	dtostrf(temp,3,1,lcdbuffer);
	uart_puts_p(PSTR("Temperatura: "));
	uart_puts(lcdbuffer);
	uart_puts(" C");
	uart_putc(0x1A);
	_delay_ms(2000);

}

void sim800_send_data_to_web (void)
{
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
	uart_puts_p(PSTR("AT+HTTPPARA=\"URL\",\"https://api.thingspeak.com/update?api_key=3UY74VC0NTPC5SMJ&field1="));
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
	//uart_puts_p(PSTR("AT+SAPBR=0,1\r\n"));
	//_delay_ms(100);
}


