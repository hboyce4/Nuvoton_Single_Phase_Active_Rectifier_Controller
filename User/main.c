/*************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    A project template for M480 MCU.
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "main.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Includes           				                                                                       */
/*---------------------------------------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "NuMicro.h"

#include "init.h"
#include "user_sys.h"
#include "inverter_control.h"
#include "PLL.h"
#include "UI.h"
#include "analog.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint64_t g_SysTickIntCnt = 0;

volatile bool UI_new_frame_tick = false;


/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
//void PDMA_IRQHandler(void);
//void UART_PDMATest(void);



void SysTick_Handler(void)	// Every millisecond
{
    g_SysTickIntCnt++;

	static uint16_t UI_refresh_counter = 0;
	if(!UI_refresh_counter){
		UI_refresh_counter = UI_FRAME_INTERVAL_MS;
		UI_new_frame_tick = true;
	}
	UI_refresh_counter--;

	PA13 ^= 1; // Toggle amber LED

}


int main()
{

    SYS_Init();
    /* SysTick 1m second interrupts  */
    SysTick_Config(SystemCoreClock / 1000);
    NVIC_SetPriority(SysTick_IRQn, SYSTICK_INT_PRIORITY);

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    /* Open UART 2, for external communication, on pins 31 & 32*/
    UART_Open(UART2, 460800);
    init_UART2_DMA(); /*Needs SysTick*/

    printf("UART0 Debug Port.\n");/* Connect UART0 on PA0 and PA1 to PC, and open a terminal tool to access debug info*/
    printf("Initializing...\n");

    init_buttons_LEDs();

    init_ADC();
#ifndef PWM_DAC
    init_DAC();
#endif
    printf("Building sine table...\n");
    init_sin_table(sin_table,SIN_TABLE_SIZE);	// Initialise le tableau de référence pour la fonction sinus (Look-up table, LUT)

    init_inverter_control();

    printf("Starting loop routine.\n");
    while(1){

    	/********************** UI begin ********************************/

    	int8_t row_sel, col_sel;
    	if(UI_new_frame_tick){
    		UI_new_frame_tick = false;

        	PA->DOUT &= ~(BIT15);//Turn ON red LED
        	draw_UI(row_sel, col_sel);
        	PA->DOUT |= BIT15;//Turn OFF red LED

    	}
    	read_user_input(&row_sel,&col_sel);/* Since the chip has a 16 byte hardware FIFO, and we are only expecting
    	human input, we don't need interrupts nor DMA. Just run this routine every few milliseconds.*/

    	/*********************** UI end *********************************/


    }


}

/*** (C) COPYRIGHT 2021 Hugo Boyce ***/
