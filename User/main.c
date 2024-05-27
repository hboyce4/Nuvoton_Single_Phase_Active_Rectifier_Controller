/*************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    A project template for M480 MCU.
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "main.h"

// This firmware is very interrupt-driven. Therefore most of the "meat" is in interrupt.c

/*---------------------------------------------------------------------------------------------------------*/
/* Includes           				                                                                       */
/*---------------------------------------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "NuMicro.h"

#include "sys.h"
#include "init.h"
#include "interrupt.h"
#include "inverter_control.h"
#include "PLL.h"
#include "UI.h"
#include "analog.h"
#include "UART_over_DMA.h"
#include "persistent_data.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------------------------------------*/
/* Functions definitions                                                                      	   */
/*---------------------------------------------------------------------------------------------------------*/

int main()
{

    SYS_Init();
    /* SysTick 1m second interrupts  */
    SysTick_Config(SystemCoreClock / 1000);
    NVIC_SetPriority(SysTick_IRQn, SYSTICK_INT_PRIORITY);

    /* Init UART to 115200-8n1 to print message */
    UART_Open(UART0, 115200);

    PD_Init();

    /* Open UART 2, for external communication, on pins 31 & 32*/
    UART_Open(UART2, 460800);
    init_UART2_DMA(); /*Needs SysTick*/

    printf("UART0 Debug Port.\n");/* Connect UART0 on PA0 and PA1 to PC, and open a terminal tool to access debug info*/
    printf("Initializing...\n");

    init_general_IO();

    init_ADC();

    init_BPWM1_carrier_generation();// Initialize BPWM1 for carrier generation. Generate a square wave for the analog PWM modulator

    init_BPWM0_duty_capture(); // MUST COME AFTER BPWM1 init!!! It reads BPWM1's CNR. Initialize BPWM0 for duty cycle capture

#ifndef PWM_DAC
    init_DAC();
#endif
    printf("Building sine table...\n");
    init_sin_table(sin_table,SIN_TABLE_SIZE);	// Initialise le tableau de r�f�rence pour la fonction sinus (Look-up table, LUT)

    init_inverter_control();

    printf("Starting loop routine.\n");
    while(1){

    	/********************** UI begin ********************************/

    	int8_t row_sel, col_sel;
    	if(g_UI_new_frame_tick){
    		g_UI_new_frame_tick = false;

        	//PA->DOUT &= ~(BIT15);//Turn ON blue LED
        	draw_UI(row_sel, col_sel);
        	//PA->DOUT |= BIT15;//Turn OFF blue LED

    	}
    	read_user_input(&row_sel,&col_sel);/* Since the chip has a 16 byte hardware FIFO, and we are only expecting
    	human input, we don't need interrupts nor DMA. Just run this routine every few milliseconds.*/

    	/*********************** UI end *********************************/
    }

}

/* I don't know where to put this*/


/*** (C) COPYRIGHT 2021 Hugo Boyce ***/
