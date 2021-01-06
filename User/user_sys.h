/*
 * user_sys.h
 *
 *  Created on: Jan 5, 2021
 *      Author: Hugo Boyce
 */

#ifndef USER_SYS_H_
#define USER_SYS_H_

#include <string.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macros           				                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
#define UART1_TX_DMA_CHANNEL 10 /* PDMA->REQSEL8_11 has to be modified by hand if the channel in this macro is changed. I know this is bad.*/
#define UART_DMA_JOB_BUFF_SIZE 32 /* Max = 256*/


/*---------------------------------------------------------------------------------------------------------*/
/* Type definitions           				                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
typedef struct
{
	volatile char * str;
	volatile uint16_t count;
} UART_DMA_Xfer_t;




/*---------------------------------------------------------------------------------------------------------*/
/* External global variables                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
extern volatile uint64_t g_SysTickIntCnt;


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

struct UART1_DMA_Job_Buffer{
	volatile UART_DMA_Xfer_t Buff[UART_DMA_JOB_BUFF_SIZE];
	volatile uint8_t Head;
	volatile uint8_t Tail;
};


/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void update_button_states(void);
void delay_ms(uint32_t time); /*Generates a milliseconds delay. NOT ACCURATE. Use a hardware timer for accuracy*/
void init_UART1_DMA(void); /* Needs SysTick!! */
void start_UART1_DMA_Xfer(UART_DMA_Xfer_t);
int8_t push_UART1(char*);
void pop_UART1(UART_DMA_Xfer_t);

#endif /* USER_SYS_H_ */
