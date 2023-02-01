/*
 *  SwitecX25 Arduino Library
 *  Guy Carpenter, Clearwater Software - 2012
 *
 *  Licensed under the BSD2 license, see license.txt for details.
 *
 *  All text above must be included in any redistribution.
 */

#ifndef SwitecX25_h
#define SwitecX25_h

#include <stdint.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal_gpio.h"
#include <stdlib.h>

#define STATE_COUNT 6

#define STEPS (315*3)

// During zeroing we will step the motor CCW
// with a fixed step period defined by RESET_STEP_MICROSEC
#define RESET_STEP_MICROSEC 800


typedef struct SwitecX25 {
	uint32_t steps;
	uint16_t pins[4];
	unsigned char currentState;    // 6 steps
	unsigned int currentStep;      // step we are currently at
	unsigned int targetStep;       // target we are moving to
	uint32_t time0;           // time when we entered this state
	unsigned int microDelay;       // microsecs until next state
	unsigned int maxVel;           // fastest vel allowed
	unsigned int vel;              // steps travelled under acceleration
	signed char dir;                      // direction -1,0,1
	uint8_t stopped;               // true if stopped
	GPIO_TypeDef* GPIOx;
}SwitecX25;

void stepUp(SwitecX25 *x25);
void stepDown(SwitecX25 *x25);
void zero(SwitecX25 *x25);
void update(SwitecX25 *x25);
void updateBlocking(SwitecX25 *x25);
void setPosition(SwitecX25 *x25, unsigned int pos);
struct SwitecX25* init_stepper(uint16_t pin1,
		uint16_t pin2, uint16_t pin3, uint16_t pin4,
		GPIO_TypeDef *gpio_port);

#endif
