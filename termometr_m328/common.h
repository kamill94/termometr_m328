/*
 * common.h
 *
 *  Created on: 19 sie 2017
 *      Author: Admin
 */

#ifndef COMMON_H_
#define COMMON_H_

#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>

#define MCP9808_ADDRESS 0x18 //Adres czujnika temperatury
#define CALL_TIME 20
#define DELAY_TIME 60
#define DEFAULT_TEMP_ALARM 20
#define ALARM_STOP_TIME 2700
#define SEND_DATA_TO_WEB 1
#define LCD_TIME 30

typedef struct {
    char nrtel[10];
    char api[17];
} CFG;

extern uint8_t sim800_signal;
extern float temp;
extern char uart_receive_buffer [129];
extern char uart_parsed_string [33];
extern uint8_t uart_datatype;
extern uint8_t timer0_second;
extern uint8_t f_temp;
extern uint8_t f_lcd_ref;
extern float battery;
extern uint8_t f_send_web;
extern CFG ram_cfg;
extern CFG eem_cfg EEMEM;
extern CFG const pgm_cfg PROGMEM;
extern uint8_t timer0_lcd;
extern uint8_t f_lcdon;
extern uint8_t f_alarm;
extern char lcdbuffer [];
extern uint8_t f_alarm_stop;
extern signed int alarm_stop_time;
extern uint8_t f_alarm;
extern uint8_t f_connect;
extern uint8_t f_delay;
extern uint8_t f_disconnect;
extern uint8_t longpress;




void timer0_init(void);
void lcd_refresh (void);
void alarms (void);
void pin_init (void);
void pin_check (void);
uint8_t parsedata(char *data, char *out);
void adc_init (void);
void settings (void);
void copy_eem_ram (void);
void copy_pgm_ram (void);
void copy_ram_eem (void);
void check_and_read_cfg (void);
void read_default_settings (void);
void battery_low (void);
char * uart_get_str(char *buf);




#endif /* COMMON_H_ */
