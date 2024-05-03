#include "millis.h"
#include "stm32l0xx_hal.h"

// Millisecond timer, inspired by the Arduino library
// Uses Timer0 interrupts

volatile uint32_t g_Millis=0;		//Global var. for counting milliseconds.
									//Don't touch without disabling interrupts.

void initMillis() {
	return;
}

// Return ellapsed time since startup in [ms]
uint32_t millis(){
	return HAL_GetTick();
}

