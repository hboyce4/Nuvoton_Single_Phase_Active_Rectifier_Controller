/*
 * init.c
 *
 *  Created on: Feb 3, 2022
 *      Author: Hugo Boyce
 */

#include "init.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------------------------------------*/
/* Function definitions                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/

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
    /* Set PCLK0 to HCLK/2 (96MHz) and PCLK1 to HCLK/2(96MHz) */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable IP clock */
    //CLK_EnableModuleClock(UART0_MODULE);
    //CLK_EnableModuleClock(TMR0_MODULE);
    //CLK_EnableModuleClock(EPWM0_MODULE);
    //CLK_EnableModuleClock(TMR1_MODULE);
    CLK->APBCLK0 |= CLK_APBCLK0_TMR0CKEN_Msk; // TMR0 Clock Enable
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk; // UART0 Clock Enable
    CLK->APBCLK0 |= CLK_APBCLK0_UART2CKEN_Msk; // UART2 Clock Enable
    CLK->APBCLK0 |= CLK_APBCLK0_EADCCKEN_Msk; /* EADC0 Clock Enable*/
    CLK->APBCLK0 |= CLK_APBCLK0_TMR1CKEN_Msk; /* TMR1 Clock Enable*/

#ifdef PWM_DAC // If PWM DAC is used
    CLK->APBCLK1 |= CLK_APBCLK1_EPWM1CKEN_Msk; /* EPWM 1 Clock enable*/
#else
    CLK->APBCLK1 |= CLK_APBCLK1_DACCKEN_Msk;	/* DAC 0 & 1  Clock enable*/
#endif
    CLK->APBCLK1 |= CLK_APBCLK1_BPWM1CKEN_Msk; /* BPWM 1 Clock enable*/

    CLK->AHBCLK  |= CLK_AHBCLK_PDMACKEN_Msk; // PDMA Clock Enable

    /* Select IP clock source from HXT */
    //CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));
    //CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HXT, 0);
    //CLK_SetModuleClock(EADC_MODULE, 0, CLK_CLKDIV0_EADC(1));

    /* Select TMR0 clock source as HXT (0x0 for HXT) */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_TMR0SEL_Msk) | CLK_CLKSEL1_TMR0SEL_HXT;
    /* Select UART0 clock source is HXT (0x0 for HXT) */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UART0SEL_Msk) | CLK_CLKSEL1_UART0SEL_HXT;
    /* Select TMR1 clock source as HXT (0x0 for HXT) */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_TMR1SEL_Msk) | CLK_CLKSEL1_TMR1SEL_HXT;
#ifdef PWM_DAC // If PWM DAC is used
    /* Set EPWM1 clock source as PLL  (0x0 for PLL) */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & ~CLK_CLKSEL2_EPWM1SEL_Msk) | CLK_CLKSEL2_EPWM1SEL_PLL;
#endif
    CLK->CLKSEL2 = (CLK->CLKSEL2 & ~CLK_CLKSEL2_BPWM1SEL_Msk) | CLK_CLKSEL2_BPWM1SEL_PLL;
    //CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_PCLK0, 0);

    /* Select UART2 clock source is HXT (0x0 for HXT) */
    CLK->CLKSEL3 = (CLK->CLKSEL3 & ~CLK_CLKSEL3_UART2SEL_Msk) | CLK_CLKSEL3_UART2SEL_HXT;

    /* EADC clock has only one source: PCLK1, so no need to select it.*/
    /* Set divider for EADC */
    CLK->CLKDIV0 &= ~(CLK_CLKDIV0_EADCDIV_Msk);/* Reset EADCDIV */
    CLK->CLKDIV0 |= 1 << CLK_CLKDIV0_EADCDIV_Pos; /* EADC divider is EADCDIV + 1 . So EADC Clock will be PCLK1/2 (48MHz/1 = 48MHz) */


    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();


    /* Set GPA multi-function pins for UART0 RXD and TXD */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA0MFP_Msk | SYS_GPA_MFPL_PA1MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA0MFP_UART0_RXD | SYS_GPA_MFPL_PA1MFP_UART0_TXD);

    /* Set GPC multi-function pins for UART2 TXD and RXD on PC0(pin 32) and PC1(Pin 31) */
     SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC0MFP_Msk | SYS_GPC_MFPL_PC1MFP_Msk);
     SYS->GPC_MFPL |= (SYS_GPC_MFPL_PC0MFP_UART2_RXD | SYS_GPC_MFPL_PC1MFP_UART2_TXD);//(0x8 << SYS_GPC_MFPL_PC0MFP_Pos) | (0x8 << SYS_GPC_MFPL_PC1MFP_Pos);

     /* Set PA.6 as output from BPWM1_CH3 for hardware PWModulator carrier*/
     SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA6MFP_Msk);
     SYS->GPA_MFPL |= SYS_GPA_MFPL_PA6MFP_BPWM1_CH3;
     PA->SLEWCTL |= (GPIO_SLEWCTL_HIGH << 2*6); /*Set PA15 to "High" slew rate.*/
     	/* For some reason "High" mode seems faster than "Fast" mode. Most likely it's just my probing setup (signal reflections and such) */

#ifdef PWM_DAC // If PWM DAC is used
     SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk|SYS_GPB_MFPH_PB13MFP_Msk); /* Setup DAC pins as PWM outputs*/
     SYS->GPB_MFPH |= SYS_GPB_MFPH_PB12MFP_EPWM1_CH3|SYS_GPB_MFPH_PB13MFP_EPWM1_CH2;
     PB->SLEWCTL |= (GPIO_SLEWCTL_HIGH << 2*12)|(GPIO_SLEWCTL_HIGH << 2*13); /*Set PB12 & PB13 to "High" slew rate.*/
     /* For some reason "High" mode seems faster than "Fast" mode. Most likely it's just my probing setup (signal reflections and such) */
#endif



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

#ifndef PWM_DAC // If PWM DAC isn't used
	 /* DAC: Set pins as input to prevent writing to them */
	 PB->MODE &= ~(/*GPIO_MODE_MODE12_Msk |*/ GPIO_MODE_MODE13_Msk);
	 /* DAC: Set PB multi-function pins for DAC voltage output */
	 SYS->GPB_MFPH |= /*SYS_GPB_MFPH_PB12MFP_DAC0_OUT |*/ SYS_GPB_MFPH_PB13MFP_DAC1_OUT;
	 /* DAC: Disable digital input path of analog pin DAC0_OUT and DAC1_OUT to prevent leakage */
	 GPIO_DISABLE_DIGITAL_PATH(PB, /*BIT12 |*/ BIT13);
#endif

    /* Lock protected registers */
    SYS_LockReg();
}
