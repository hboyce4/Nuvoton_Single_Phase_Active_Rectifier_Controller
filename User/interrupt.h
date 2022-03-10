/*
 * interrupt.h
 *
 *  Created on: Jan 6, 2021
 *      Author: Hugo Boyce
 */

#ifndef INTERRUPT_H_
#define INTERRUPT_H_

#include "NuMicro.h"
#include "analog.h"
#include "UART_over_DMA.h"

void PDMA_IRQHandler(void);
void TMR1_IRQHandler(void);
void EADC00_IRQHandler(void);
#ifndef PWM_DAC
void DAC_IRQHandler(void);
#endif
//void EADC00_IRQHandler(void);
//void UART1_IRQHandler(void);


#endif /* INTERRUPT_H_ */
