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



void SysTick_Handler(void)
{
    g_SysTickIntCnt++;

	update_button_LED_states();/* Copy button SW2 and SW3 states to LEDG and LEDY and flash LEDR */

	static uint16_t UI_refresh_counter = 0;
	if(!UI_refresh_counter){
		UI_refresh_counter = UI_FRAME_INTERVAL_MS;
		UI_new_frame_tick = true;
	}
	UI_refresh_counter--;

	PH->DOUT &= ~(BIT1);//Timing measurements
	convert_from_ADC();
	PLL_main();
	PH->DOUT |= BIT1;	//Timing measurements
}


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable External XTAL (4~24 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);
    /* Set PCLK0 to HCLK/2 (96MHz) and PCLK1 to HCLK/4(48MHz) */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV4);

    /* Enable IP clock */
    //CLK_EnableModuleClock(UART0_MODULE);
    //CLK_EnableModuleClock(TMR0_MODULE);
    //CLK_EnableModuleClock(EPWM0_MODULE);
    CLK->APBCLK0 |= CLK_APBCLK0_TMR0CKEN_Msk; // UART0 Clock Enable
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk; // UART0 Clock Enable
    CLK->APBCLK0 |= CLK_APBCLK0_UART1CKEN_Msk; // UART1 Clock Enable
    CLK->APBCLK0 |= CLK_APBCLK0_EADCCKEN_Msk; /* EADC0 Clock Enable*/

    CLK->APBCLK1 |= CLK_APBCLK1_EPWM1CKEN_Msk; /* EPWM 1 Clock enable*/

    CLK->AHBCLK  |= CLK_AHBCLK_PDMACKEN_Msk; // PDMA Clock Enable

    /* Select IP clock source from HXT */
    //CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));
    //CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HXT, 0);
    //CLK_SetModuleClock(EADC_MODULE, 0, CLK_CLKDIV0_EADC(1));

    /* Select TMR0 clock source as HXT (0x0 for HXT) */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_TMR0SEL_Msk) | (0x0 << CLK_CLKSEL1_TMR0SEL_Pos);
    /* Select UART0 clock source is HXT (0x0 for HXT) */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UART0SEL_Msk) | (0x0 << CLK_CLKSEL1_UART0SEL_Pos);
    /* Select UART1 clock source is HXT (0x0 for HXT) */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UART1SEL_Msk) | (0x0 << CLK_CLKSEL1_UART1SEL_Pos);
    /* Set EPWM1 clock source as PLL  (0x0 for PLL, 0x1 for PCLK0) */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & ~CLK_CLKSEL2_EPWM1SEL_Msk) | (0x0 << CLK_CLKSEL2_EPWM1SEL_Msk);

    /* EADC clock has only one source: PCLK1, so no need to select it.*/
    /* Set divider for EADC */
    CLK->CLKDIV0 &= ~(CLK_CLKDIV0_EADCDIV_Msk);/* Reset EADCDIV */
    CLK->CLKDIV0 |= 0 << CLK_CLKDIV0_EADCDIV_Pos; /* EADC divider is EADCDIV + 1 . So EADC Clock will be PCLK1/1 (48MHz/1 = 48MHz) */


    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();


    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Set PA multi-function pins for UART1 TXD and RXD*/
     SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk);
     SYS->GPA_MFPL |= (0x8 << SYS_GPA_MFPL_PA2MFP_Pos) | (0x8 << SYS_GPA_MFPL_PA3MFP_Pos);

     /* Set PB.14 as output from EPWM1_CH1 */
     SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB14MFP_Msk);
     SYS->GPB_MFPH |= SYS_GPB_MFPH_PB14MFP_EPWM1_CH1;

     /* EADC: Set PB.0 ~ PB.7 to input mode */
	 PB->MODE &= ~(GPIO_MODE_MODE0_Msk | GPIO_MODE_MODE1_Msk | GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk |
			 GPIO_MODE_MODE4_Msk | GPIO_MODE_MODE5_Msk | GPIO_MODE_MODE6_Msk | GPIO_MODE_MODE7_Msk);
	 /* EADC: Configure the GPB0 - GPB7 ADC analog input pins. (Multi Function Pins) */
	 SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB0MFP_Msk | SYS_GPB_MFPL_PB1MFP_Msk | SYS_GPB_MFPL_PB2MFP_Msk | SYS_GPB_MFPL_PB3MFP_Msk
			 |SYS_GPB_MFPL_PB4MFP_Msk | SYS_GPB_MFPL_PB5MFP_Msk | SYS_GPB_MFPL_PB6MFP_Msk | SYS_GPB_MFPL_PB7MFP_Msk);
	 SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB0MFP_EADC0_CH0 | SYS_GPB_MFPL_PB1MFP_EADC0_CH1 | SYS_GPB_MFPL_PB2MFP_EADC0_CH2 |
			 SYS_GPB_MFPL_PB3MFP_EADC0_CH3 | SYS_GPB_MFPL_PB4MFP_EADC0_CH4 | SYS_GPB_MFPL_PB5MFP_EADC0_CH5 | SYS_GPB_MFPL_PB6MFP_EADC0_CH6 |
			 SYS_GPB_MFPL_PB7MFP_EADC0_CH7);
	 /* EADC: Disable the GPB0 - GPB7 digital input path to avoid the leakage current. */
	 GPIO_DISABLE_DIGITAL_PATH(PB, BIT7|BIT6|BIT5|BIT4|BIT3|BIT2|BIT1|BIT0);


    /* Lock protected registers */
    SYS_LockReg();
}

/*
 * This is a template project for M480 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART0 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */

int main()
{

    SYS_Init();
    /* SysTick 1m second interrupts  */
    SysTick_Config(SystemCoreClock / 1000);

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    UART_Open(UART1, 460800);
    init_UART1_DMA(); /*Needs SysTick*/

    /*Test*/

    /* Configure PH.0, PH.1 and PH.2 as Output mode for LED blink */
    GPIO_SetMode(PH, BIT0|BIT1|BIT2, GPIO_MODE_OUTPUT); // LED outputs
    GPIO_SetMode(PG, BIT15, GPIO_MODE_INPUT); // Configure pin as input for Button 1
    GPIO_SetMode(PF, BIT11, GPIO_MODE_INPUT); // Configure pin as input for Button 2

    start_PWModulator_carrier(); /* Begin to output the carrier waveform for the analog hardware PWModulator on PB.14 (pin 133 on the M487JIDAE) */

    init_ADC();
    init_sin_table(sin_table,SIN_TABLE_SIZE);	// Initialise le tableau de référence pour la fonction sinus (Look-up table, LUT)

    /* Connect UART to PC, and open a terminal tool to receive following message */
    printf("Hello World\n");
    printf("\033[2J");


    while(1){

    	/* UI begin */
    	int8_t row_sel, col_sel;
    	if(UI_new_frame_tick){
    		UI_new_frame_tick = false;
    		//PH->DOUT &= ~(BIT1); //Timing measurements
    		draw_UI(row_sel, col_sel);
    		//PH->DOUT |= BIT1;	//Timing measurements
    	}
    	read_user_input(&row_sel,&col_sel);/* Since the chip has a 16 byte hardware FIFO, and we are only expecting
    	 human input, we don't need interrupts nor DMA. Just run this routine every few milliseconds.*/
    	/* UI end */


    }


}

/*** (C) COPYRIGHT 2021 Hugo Boyce ***/
