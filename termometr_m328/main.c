/*
 * main.c
 *
 *  Created on: 10 lip 2017
 *      Author: Kamil Blasiak
 */

#define MCP9808_ADDRESS 0x18 //Adres czujnika temperatury
#define CALL_TIME 20
#define DELAY_TIME 60
#define DEFAULT_TEMP_ALARM 20
#define ALARM_STOP_TIME 2700
#define SEND_DATA_TO_WEB 1
#define LCD_TIME 30

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

uint8_t timer0_skaler = 0;
uint8_t timer0_second = 0;
uint8_t timer0_minute = 0;
uint8_t timer0_lcd = LCD_TIME;

float temp;
float battery;

char lcdbuffer [16];

uint8_t longpress = 0;

char uart_receive_buffer [129];
char uart_parsed_string [33];
uint8_t uart_datatype;

uint8_t sim800_func_count = 0;
uint8_t sim800_signal;


uint8_t f_temp = 0;  //flagi
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

typedef struct {
    char nrtel[10];
    char api[17];
} CFG;

CFG eem_cfg EEMEM;  // definicja struktury w eeprom
CFG ram_cfg;  // definicja struktury w ram
CFG const pgm_cfg PROGMEM = {"792682279","3UY74VC0NTPC5SMJ"};


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
void adc_init (void);
void settings (void);
void copy_eem_ram (void);
void copy_pgm_ram (void);
void copy_ram_eem (void);
void check_and_read_cfg (void);
void read_default_settings (void);
void battery_low (void);
char * uart_get_str(char *buf);


ISR (TIMER0_OVF_vect)
{
	timer0_skaler++;
	if(timer0_skaler>=125)
	{
		timer0_skaler = 0;
		timer0_second++;
		f_temp=1;
		f_lcd_ref=1;
		timeout--;

		if( (!(PIND & (1<<PD2))) || (!(PIND & (1<<PD2))) ) longpress++;
		else longpress = 0;

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
				if(timer0_minute>239) timer0_minute = 0;

				if(!(timer0_minute % 20)) f_send_web = 1;
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

		lcd_gotoxy(9,0);
		if(!(PIND & (1<<PD4)))
		{
			lcd_puts("BAT");
		}
		else lcd_puts("ZAS");

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
		dtostrf(battery,3,2,lcdbuffer);
		lcd_puts_p(PSTR("B:"));
		lcd_puts(lcdbuffer);
		lcd_puts_p(PSTR("V"));

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

}

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

	DDRD &= ~(1<<PD4);
	PORTD &= ~(1<<PD4);

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


	if(!f_alarm && (!(PIND & (1<<PD2))))
	{
		if(longpress>=5) settings();
	}

	if(!(PIND & (1<<PD2)))
		{
		timer0_lcd = LCD_TIME;
		lcd_led(0);
		}
	else longpress = 0;
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

   /* else if (strstr(data, "+CBC")) // Odczyt napiecia baterii
    	{
    	strcpy(out, data + 11);
    	out[4] = 0;
    	datatype = 6;
    	sim800_bat = atoi(out);
    	} */

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

void adc_init (void)
{
    ADCSRA = (1<<ADEN) //ADEN=1 w³¹czenie przetwornika ADC)
             |(1<<ADPS0) // ustawienie preskalera na 128
             |(1<<ADPS1)
             |(1<<ADPS2)
    		 |(1<<ADATE);

    ADMUX  =    (1<<REFS1) | (1<<REFS0); // REFS1:0: wybór napiêcia odniesienia ADC
                               //na wewnêtrzne Ÿród³o 1,1V
                               //z zewnêtrznym kondensatorem na pinie AREF
    ADCSRA |= (1<<ADSC);
}

void settings (void)
{
	uint8_t option = 1;
	uint8_t f_nextmenu = 1;
	uint8_t write = 0;
	uint8_t char_num = 0;

	lcd_clrscr();
	lcd_gotoxy(0,0);
	lcd_puts_p(PSTR("USTAWIENIA"));
	_delay_ms(2000);

	while(write == 0)
	{

		switch (option)
		{
		case 1:

			if(f_nextmenu == 1)
			{
				lcd_gotoxy(0,0);
				lcd_putc(163);
				lcd_puts_p(PSTR("R. TEL        "));
				lcd_gotoxy(0,1);
				lcd_puts(ram_cfg.nrtel);
				f_nextmenu = 0;
			}

			if(!(PIND & (1<<PD3)))
			{
				_delay_ms(100);
				ram_cfg.nrtel[char_num]++;
				if(ram_cfg.nrtel[char_num]>'9') ram_cfg.nrtel[char_num] = '0';
				lcd_gotoxy(0,1);
				lcd_puts(ram_cfg.nrtel);
			}

			if(!(PIND & (1<<PD2)))
			{
				char_num++;
				lcd_gotoxy(0,0);
				strcpy(lcdbuffer,"NR. TEL        ");
				lcdbuffer[char_num] = 163;
				lcd_puts(lcdbuffer);
				_delay_ms(100);
			}


			if(char_num>8)
			{
				option++;
				f_nextmenu = 1;
				char_num = 0;
				ram_cfg.nrtel[9]='\0';
			}
			break;

		case 2:

			if(f_nextmenu == 1)
			{
			lcd_gotoxy(0,0);
			lcd_putc(163);
			lcd_puts_p(PSTR("HINGSPEAK API "));
			lcd_gotoxy(0,1);
			lcd_puts(ram_cfg.api);
			f_nextmenu = 0;
			}

			if(!(PIND & (1<<PD3)))
			{
				_delay_ms(50);
				ram_cfg.api[char_num]++;
				if(ram_cfg.api[char_num] == 58) ram_cfg.api[char_num] = 65;
				if(ram_cfg.api[char_num]>'Z') ram_cfg.api[char_num] = '0';
				lcd_gotoxy(0,1);
				lcd_puts(ram_cfg.api);
			}

			if(!(PIND & (1<<PD2)))
			{
				char_num++;
				lcd_gotoxy(0,0);
				strcpy(lcdbuffer,"THINGSPEAK API ");
				lcdbuffer[char_num] = 163;
				lcd_puts(lcdbuffer);
				_delay_ms(100);
			}


			if(char_num>15)
			{
				char_num = 0;
				write = 1;
				ram_cfg.api[16]='\0';
			}
			break;


		}

		if(write)
			{
			lcd_clrscr();
			lcd_puts_p(PSTR("Zapis ustawien..."));
			_delay_ms(1000);
			copy_ram_eem();
			lcd_clrscr();
			f_lcdon = 0;
			timer0_lcd = LCD_TIME;
			}

	}

}

void copy_ram_eem (void)
{
	eeprom_write_block(&ram_cfg,&eem_cfg,sizeof(CFG));
}

void copy_eem_ram (void)
{
	eeprom_read_block(&ram_cfg,&eem_cfg,sizeof(CFG));
}

void copy_pgm_ram (void)
{
	memcpy_P(&ram_cfg,&pgm_cfg,sizeof(CFG));
}

void check_and_read_cfg (void)
{
	copy_eem_ram();

	if(ram_cfg.nrtel[0] == 0xff)
	{
		read_default_settings();
	}
}

void read_default_settings (void)
{
	lcd_clrscr();
	lcd_puts("Reset ustaw...");
	_delay_ms(500);
	copy_pgm_ram();
	copy_ram_eem();

	lcd_gotoxy(0,1);
	lcd_puts(ram_cfg.api);
	_delay_ms(500);
	lcd_clrscr();
}

void battery_low (void)
{
	lcd_clrscr();
	while((!(PIND & (1<<PD4))))
	{
		lcd_gotoxy(0,0);
		lcd_puts_p(PSTR("Bateria slaba"));
		lcd_gotoxy(0,1);
		lcd_puts_p(PSTR("Podlacz zasilacz"));
	}
	lcd_clrscr();
}

char * uart_get_str(char *buf)
{
  static int index;
  int c;

  while( (c=uart_getc()) != UART_NO_DATA )
  {
    if((c=='\n') || (c=='\r'))
   {
       buf[index]='\0';
       index = 0;
       return buf;
   }
   buf[index++] = c;
   }

  return NULL;
}


