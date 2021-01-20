/*
 * user_sys.h
 *
 *  Created on: Jan 5, 2021
 *      Author: Hugo Boyce
 */

#ifndef USER_SYS_H_
#define USER_SYS_H_

/*---------------------------------------------------------------------------------------------------------*/
/* Includes           				                                                                      */
/*---------------------------------------------------------------------------------------------------------*/

#include <string.h>
#include <stdbool.h>
#include "NuMicro.h"
#include "main.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macros           				                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
#define HEARTBEAT_INTERVAL_MS 1000

#define UART1_TX_DMA_CHANNEL 10 /* PDMA->REQSEL8_11 has to be modified by hand if the channel in this macro is changed. I know this is bad.*/
#define UART_DMA_JOB_BUFF_SIZE 32 /* Must be a power of 2. Max = 256*/
#define DMA_PRIORITY 5
#define UART1_PRIORITY 6

#define PWM_CARRIER_FREQ 400000

/*---------------------------------------------------------------------------------------------------------*/
/* Type definitions           				                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
typedef struct
{
	volatile char * str;
	volatile uint16_t count;
} UART_DMA_Xfer_t;


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables to be made available externally                                                        */
/*---------------------------------------------------------------------------------------------------------*/



/*---------------------------------------------------------------------------------------------------------*/
/* Functions declaration                                                                              */
/*---------------------------------------------------------------------------------------------------------*/


void update_button_LED_states(void);
void delay_ms(uint32_t time); /*Generates a milliseconds delay. NOT ACCURATE. Use a hardware timer for accuracy*/
void init_UART1_DMA(void); /* Needs SysTick!! */
void start_UART1_DMA_Xfer(UART_DMA_Xfer_t);
int8_t push_UART1(char*);
int8_t pop_UART1(UART_DMA_Xfer_t*);

void start_PWModulator_carrier(void);

#endif /* USER_SYS_H_ */
