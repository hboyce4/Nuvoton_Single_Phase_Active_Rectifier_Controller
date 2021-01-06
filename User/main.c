/*************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    A project template for M480 MCU.
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#include "user_sys.h"


#define PLL_CLOCK           192000000



/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint64_t g_SysTickIntCnt = 0;



/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
//void PDMA_IRQHandler(void);
//void UART_PDMATest(void);





void SysTick_Handler(void)
{
    g_SysTickIntCnt++;

	update_button_states();/* Copy button SW2 and SW3 states to LEDG and LEDY */

    if(!(g_SysTickIntCnt%1000)) { /* Every 1000 ms*/

        PH0 ^= 1; // Access single pin (Toggle red LED)
        //PH->DOUT ^= BIT0|BIT1|BIT2; // Access digital output register (toggle 3 LEDs)
    }
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
    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable IP clock */
    //CLK_EnableModuleClock(UART0_MODULE);
    //CLK_EnableModuleClock(TMR0_MODULE);
    CLK->APBCLK0 |= CLK_APBCLK0_TMR0CKEN_Msk; // UART0 Clock Enable
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk; // UART0 Clock Enable
    CLK->APBCLK0 |= CLK_APBCLK0_UART1CKEN_Msk; // UART1 Clock Enable
    CLK->AHBCLK  |= CLK_AHBCLK_PDMACKEN_Msk; // PDMA Clock Enable

    /* Select IP clock source from HXT */
    //CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));
    //CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HXT, 0);

    /* Select TMR0 clock source as HXT (0x0 for HXT) */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_TMR0SEL_Msk) | (0x0 << CLK_CLKSEL1_TMR0SEL_Pos);
    /* Select UART0 clock source is HXT (0x0 for HXT) */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UART0SEL_Msk) | (0x0 << CLK_CLKSEL1_UART0SEL_Pos);
    /* Select UART1 clock source is HXT (0x0 for HXT) */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UART1SEL_Msk) | (0x0 << CLK_CLKSEL1_UART1SEL_Pos);


    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();


    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Set PA multi-function pins for UART1 TXD and RXD*/
     SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk);
     SYS->GPA_MFPL |= (0x8 << SYS_GPA_MFPL_PA2MFP_Pos) | (0x8 << SYS_GPA_MFPL_PA3MFP_Pos);

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

    UART_Open(UART1, 115200);
    init_UART1_DMA(); /*Needs SysTick*/



    /* Configure PH.0, PH.1 and PH.2 as Output mode for LED blink */
    GPIO_SetMode(PH, BIT0|BIT1|BIT2, GPIO_MODE_OUTPUT); // LED outputs
    GPIO_SetMode(PG, BIT15, GPIO_MODE_INPUT); // Configure pin as input for Button 1
    GPIO_SetMode(PF, BIT11, GPIO_MODE_INPUT); // Configure pin as input for Button 2



    /* Connect UART to PC, and open a terminal tool to receive following message */
    printf("Hello World\n");
    //printf("\033[2J");

    delay_ms(1000);

   printf("Hello World, delayed.\n");

   delay_ms(1000);

    static const char string[] = "This is a DMA transfer\n\r";
    UART_DMA_Xfer_t t1;
    t1.str = string;
    t1.count = strlen(string);
    start_UART1_DMA_Xfer(t1);

    delay_ms(1000);

    static const char string2[] = "This is a second DMA transfer\n\r";
    UART_DMA_Xfer_t t2;
    t2.str = string2;
    t2.count = strlen(string2);
    start_UART1_DMA_Xfer(t2);

    //PDMA->DSCT[UART1_TX_DMA_CHANNEL].SA = ((uint32_t)string2); 		/* Starting Source address is the beginning of the string we want to send */
    //PDMA->DSCT[UART1_TX_DMA_CHANNEL].CTL = 	(PDMA->DSCT[UART1_TX_DMA_CHANNEL].CTL & ~(PDMA_DSCT_CTL_TXCNT_Msk|PDMA_DSCT_CTL_OPMODE_Msk))|((string2_length-1) << PDMA_DSCT_CTL_TXCNT_Pos)|(0b01 << PDMA_DSCT_CTL_OPMODE_Pos); /* OR the string length in the register and set the operating mode from idle to basic*/

    //UART1->INTEN |= (0b1 << UART_INTEN_TXPDMAEN_Pos); 		/* Bit TXPDMAEN is set to one enable PDMA requests */
    /* The second and subsequent times, the UART1->INTEN write is not necessary*/


    delay_ms(1000);

    const static char clear_screen[] = "\x1B[2J";
    UART_DMA_Xfer_t t3;
    t3.str = clear_screen;
    t3.count = strlen(clear_screen);
    start_UART1_DMA_Xfer(t3);




    /* Got no where to go, just loop forever */
    while(1);


}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
