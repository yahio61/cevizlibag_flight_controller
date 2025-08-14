/*
 * pin_timing.h
 *
 *  Created on: Aug 9, 2025
 *      Author: yahya
 */

#ifndef INC_PIN_TIMING_H_
#define INC_PIN_TIMING_H_

typedef struct pin_timing
{
	GPIO_TypeDef 	*GPIO_port;
	uint16_t		GPIO_pin;
	uint32_t		last_time;
}pin_timing_t;

void time_pin_update(void);
void time_pin(pin_timing_t pin);
void reset_pin_time(pin_timing_t pin);

#endif /* INC_PIN_TIMING_H_ */
