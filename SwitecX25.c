/*
 *  SwitecX25 Arduino Library
 *  Guy Carpenter, Clearwater Software - 2012
 *
 *  Licensed under the BSD2 license, see license.txt for details.
 *
 *  All text above must be included in any redistribution.
 */

#include "SwitecX25.h"

// experimentation suggests that 400uS is about the step limit
// with my hand-made needles made by cutting up aluminium from
// floppy disk sliders.  A lighter needle will go faster.

// State  3 2 1 0   Value
// 0      1 0 0 1   0x9
// 1      0 0 0 1   0x1
// 2      0 1 1 1   0x7
// 3      0 1 1 0   0x6
// 4      1 1 1 0   0xE
// 5      1 0 0 0   0x8
static unsigned char stateMap[] = {0x9, 0x1, 0x7, 0x6, 0xE, 0x8};

// This table defines the acceleration curve as a list of (step, delay) pairs.
// 1st value is the cumulative step count since starting from rest, 2nd value is delay in microseconds.
// 1st value in each subsequent row must be > 1st value in previous row
// The delay in the last row determines the maximum angular velocity.
unsigned short defaultAccelTable[][2] = { { 20, 3000 }, { 50, 1500 }, {
		100, 1000 }, { 150, 800 }, { 300, 600 } };


#define DEFAULT_ACCEL_TABLE_SIZE (sizeof(defaultAccelTable)/sizeof(*defaultAccelTable))

uint32_t micros(){
	return HAL_GetTick()* 1000;
}

struct SwitecX25* init_stepper(uint16_t pin1,
		uint16_t pin2, uint16_t pin3, uint16_t pin4,
		GPIO_TypeDef *gpio_port){
	  struct SwitecX25 *x25 = malloc(sizeof(struct SwitecX25));
	  x25->currentState = 0;
	  x25->steps = STEPS;
	  x25->pins[0] = pin1;
	  x25->pins[1] = pin2;
	  x25->pins[2] = pin3;
	  x25->pins[3] = pin4;
	  x25->GPIOx = gpio_port;

	  x25->dir = 0;
	  x25->vel = 0;
	  x25->stopped = 1;
	  x25->currentStep = 0;
	  x25->targetStep = 0;
	  x25->maxVel = defaultAccelTable[DEFAULT_ACCEL_TABLE_SIZE-1][0]; // last value in table.
	  return x25;

}

void writeIO(SwitecX25 *x25) {
	unsigned char mask = stateMap[x25->currentState];
	for (int i = 0; i < 4; i++) {
		HAL_GPIO_WritePin(x25->GPIOx, x25->pins[i], mask & 0x1);
		mask >>= 1;
	}
}

void stepUp(SwitecX25 *x25) {
	if (x25->currentStep < x25->steps) {
		x25->currentStep++;
		x25->currentState = (x25->currentState + 1) % STATE_COUNT;
		writeIO(x25);
	}
}

void stepDown(SwitecX25 *x25) {
	if (x25->currentStep > 0) {
		x25->currentStep--;
		x25->currentState = (x25->currentState + 5) % STATE_COUNT;
		writeIO(x25);
	}
}

void zero(SwitecX25 *x25) {
	x25->currentStep = x25->steps - 1;
	for (unsigned int i = 0; i < x25->steps; i++) {
		stepDown(x25);
		HAL_Delay(5);
	}
	x25->currentStep = 0;
	x25->targetStep = 0;
	x25->vel = 0;
	x25->dir = 0;
}

// This function determines the speed and accel
// characteristics of the motor.  Ultimately it 
// steps the motor once (up or down) and computes
// the delay until the next step.  Because it gets
// called once per step per motor, the calcuations
// here need to be as light-weight as possible, so
// we are avoiding floating-point arithmetic.
//
// To model acceleration we maintain vel, which indirectly represents
// velocity as the number of motor steps travelled under acceleration
// since starting.  This value is used to look up the corresponding
// delay in accelTable.  So from a standing start, vel is incremented
// once each step until it reaches maxVel.  Under deceleration 
// vel is decremented once each step until it reaches zero.

void advance(SwitecX25 *x25) {
	// detect stopped state
	if (x25->currentStep == x25->targetStep && x25->vel == 0) {
		x25->stopped = 1;
		x25->dir = 0;
		x25->time0 = micros();
		return;
	}

	// if stopped, determine direction
	if (x25->vel == 0) {
		x25->dir = x25->currentStep < x25->targetStep ? 1 : -1;
		// do not set to 0 or it could go negative in case 2 below
		x25->vel = 1;
	}

	if (x25->dir > 0) {
		stepUp(x25);
	} else {
		stepDown(x25);
	}

	// determine delta, number of steps in current direction to target.
	// may be negative if we are headed away from target
	int delta = x25->dir > 0 ? x25->targetStep - x25->currentStep : x25->currentStep - x25->targetStep;

	if (delta > 0) {
		// case 1 : moving towards target (maybe under accel or decel)
		if (delta < x25->vel) {
			// time to declerate
			x25->vel--;
		} else if (x25->vel < x25->maxVel) {
			// accelerating
			x25->vel++;
		} else {
			// at full speed - stay there
		}
	} else {
		// case 2 : at or moving away from target (slow down!)
		x25->vel--;
	}

	// vel now defines delay
	unsigned char i = 0;
	// this is why vel must not be greater than the last vel in the table.
	while (defaultAccelTable[i][0] < x25->vel) {
		i++;
	}
	x25->microDelay = defaultAccelTable[i][1];
	x25->time0 = micros();
}

void setPosition(SwitecX25 *x25, unsigned int pos) {
	// pos is unsigned so don't need to check for <0
	if (pos >= x25->steps)
		pos = x25->steps - 1;
	x25->targetStep = pos;
	if (x25->stopped) {
		// reset the timer to avoid possible time overflow giving spurious deltas
		x25->stopped = 0;
		x25->time0 = micros();
		x25->microDelay = 0;
	}
}

void update(SwitecX25 *x25) {
	if (!x25->stopped) {
		unsigned long delta = micros() - x25->time0;
		if (delta >= x25->microDelay) {
			advance(x25);
		}
	}
}

//This updateMethod is blocking, it will give you smoother movements, but your application will wait for it to finish
void updateBlocking(SwitecX25 *x25) {
	while (!x25->stopped) {
		unsigned long delta = micros() - x25->time0;
		if (delta >= x25->microDelay) {
			advance(x25);
		}
	}
}

