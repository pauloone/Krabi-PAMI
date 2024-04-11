/*
 * custom_interrupt.c
 *
 *  Created on: Apr 2, 2024
 *      Author: pauli
 */


#include "custom_interrupt.h"

EXTI_HandleTypeDef RUN_Handle = {0};

void interrupt_init(void){
	EXTI_ConfigTypeDef hext_a7_config = {0};
	hext_a7_config.Line = EXTI_LINE_7;
	hext_a7_config.Mode = EXTI_MODE_INTERRUPT;
	hext_a7_config.Trigger = EXTI_TRIGGER_RISING_FALLING;
	hext_a7_config.GPIOSel = EXTI_GPIOA;
	HAL_EXTI_GetHandle(&RUN_Handle, EXTI_LINE_7);
	HAL_EXTI_SetConfigLine(&RUN_Handle, &hext_a7_config);
	HAL_EXTI_RegisterCallback(&RUN_Handle, HAL_EXTI_COMMON_CB_ID, &RUN_interrupt);
	HAL_NVIC_SetPriority(EXTI4_15_IRQn,3,0);
	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

void RUN_interrupt(void){
	asm("NOP");
}

void EXTI4_15_IRQHandler(void)
{
// Function called on 4_15 interrupt
// We call the HAL handler
	if(HAL_EXTI_GetPending(&RUN_Handle, EXTI_TRIGGER_RISING_FALLING))
	{
		HAL_EXTI_IRQHandler(&RUN_Handle);
	}
};
