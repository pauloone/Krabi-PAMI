#ifndef MILLIS_H
#define MILLIS_H   1
#include "main.h"

// Millisecond timer, inspired by the Arduino linrary
// Uses Timer0 interrupts

//--------------------------------------------------
// Macros to manage fractional ticks
//--------------------------------------------------

//--------------------------------------------------
// Public functions
//--------------------------------------------------
// sets-up Timer0 (prescale, reload, interrupts)
void initMillis();

//Return number of milliseconds since startup
uint32_t millis();

#endif
