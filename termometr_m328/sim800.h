/*
 * sim800.h
 *
 *  Created on: 19 sie 2017
 *      Author: Admin
 */

#ifndef SIM800_H_
#define SIM800_H_


void sim800_init(void);
void sim800_reset(void);
void sim800_get_data (void);
void sim800_call(void);
void sim800_disconnect (void);
void sim800_send_sms (void);
void sim800_send_data_to_web (void);



#endif /* SIM800_H_ */
