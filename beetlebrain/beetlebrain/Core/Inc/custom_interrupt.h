/*
 * custom_interrupt.h
 *
 *  Created on: Apr 2, 2024
 *      Author: pauli
 */

#ifndef INC_CUSTOM_INTERRUPT_H_
#define INC_CUSTOM_INTERRUPT_H_

#include "stm32l0xx_hal.h"

extern EXTI_HandleTypeDef RUN_Handle;

void interrupt_init(void);
void RUN_interrupt(void);

void EXTI4_15_IRQHandler(void);

#endif /* INC_CUSTOM_INTERRUPT_H_ */
