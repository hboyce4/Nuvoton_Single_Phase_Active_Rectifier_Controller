/*
 * init.c
 *
 *  Created on: Feb 3, 2022
 *      Author: Hugo Boyce
 */

#include "init.h"
#include "analog.h"
#include "timers.h"

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
    CLK->APBCLK1 |= CLK_APBCLK1_BPWM0CKEN_Msk; /* BPWM 0 Clock enable (For duty cycle capture */


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


    CLK_SetModuleClock(BPWM0_MODULE, CLK_CLKSEL2_BPWM0SEL_PLL, 0); /* For duty cycle capture. Make this uniform with the others in the future. */

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


	 //SYS->GPF_MFPL &= ~SYS_GPF_MFPL_PF4MFP_Msk; /* Clear the bits of PF4 multi function port selection */
	 //SYS->GPF_MFPL |= SYS_GPF_MFPL_PF4MFP_BPWM0_CH5;/* Set PF4 as BPWM0 channel 5*/

	 SYS->GPA_MFPH &= ~SYS_GPA_MFPH_PA10MFP_Msk; // Set multifuntion pin for BPWM0_CH1 for duty cycle capture
	 SYS->GPA_MFPH |= SYS_GPA_MFPH_PA10MFP_BPWM0_CH1;

    /* Lock protected registers */
    SYS_LockReg();
}


void init_general_IO(void){

	// LEDs
	/* Configure PA.12, PA.13 PA.14 and PA.15 as Output mode for LED blink on M481 boards */
	GPIO_SetMode(PA, BIT12|BIT13|BIT14|BIT15, GPIO_MODE_OUTPUT); // LED outputs

	// Relay and precharge optos
	// Since they're active-low, we initialize them high
	PC->DOUT |= (BIT2|BIT3|BIT4|BIT5); //set opto outputs high/inactive
	GPIO_SetMode(PC, BIT2|BIT3|BIT4|BIT5, GPIO_MODE_OUTPUT); // Set optocoupler pins to outputs

	// Inverter control pins
	PA5 = true; // Compensator reset pin is active high. It should be in the reset state when the inverter is not running to prevent windup.
	PA4 = false; // Enable pulse is idle in low state. Rising-edge trigerred
	PA2 = false; // Current_ok_latch is set by setting the pin high
	GPIO_SetMode(PA, BIT2|BIT4|BIT5, GPIO_MODE_OUTPUT);
	GPIO_SetMode(PA, BIT3, GPIO_MODE_INPUT); // input for Current_ok_latch state

}

void init_ADC(void){


    /* Set input mode as single-end and enable the A/D converter */
    EADC_Open(EADC, EADC_CTL_DIFFEN_SINGLE_END);

//    /* Configure the sample 4 module for analog input channel 0 and enable ADINT0 trigger source */
//    EADC_ConfigSampleModule(EADC, 0, EADC_ADINT0_TRIGGER, 0);
//    /* Configure the sample 5 module for analog input channel 1 and enable ADINT0 trigger source */
//    EADC_ConfigSampleModule(EADC, 1, EADC_ADINT0_TRIGGER, 1);
//    /* Configure the sample 6 module for analog input channel 2 and enable ADINT0 trigger source */
//    EADC_ConfigSampleModule(EADC, 2, EADC_ADINT0_TRIGGER, 2);
//    /* Configure the sample 7 module for analog input channel 3 and enable ADINT0 trigger source */
//    EADC_ConfigSampleModule(EADC, 3, EADC_ADINT0_TRIGGER, 3);
//    /* Configure the sample 4 module for analog input channel 0 and enable ADINT0 trigger source */
//    EADC_ConfigSampleModule(EADC, 4, EADC_ADINT0_TRIGGER, 4);
//    /* Configure the sample 5 module for analog input channel 1 and enable ADINT0 trigger source */
//    EADC_ConfigSampleModule(EADC, 5, EADC_ADINT0_TRIGGER, 5);
//    /* Configure the sample 6 module for analog input channel 2 and enable ADINT0 trigger source */
//    EADC_ConfigSampleModule(EADC, 6, EADC_ADINT0_TRIGGER, 6);
//    /* Configure the sample 7 module for analog input channel 3 and enable ADINT0 trigger source */
//    EADC_ConfigSampleModule(EADC, 7, EADC_ADINT0_TRIGGER, 7);

    uint8_t i;
    for(i = EADC_FIRST_CHANNEL; i <= EADC_LAST_CHANNEL; i++){

    	/* Configure the sample i module for analog input channel i and enable ADINT0 trigger source */
    	EADC_ConfigSampleModule(EADC, i, EADC_ADINT0_TRIGGER, i);
    }


    /* Clear the A/D ADINT0 interrupt flag for safety */
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);

    /* Enable the last sample module interrupt */
    EADC_ENABLE_INT(EADC, BIT0);//Enable sample module  A/D ADINT0 interrupt.
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 0, (1 << EADC_LAST_CHANNEL));//Enable sample module 7 interrupt.
    NVIC_SetPriority(EADC00_IRQn, ADC_INT_PRIORITY);

    /* Reset the ADC indicator and trigger sample module 7 to start A/D conversion */
    //g_u32AdcIntFlag = 0;
    //g_u32COVNUMFlag = 0;
    EADC_START_CONV(EADC, (1 << EADC_LAST_CHANNEL));

    //__WFI();

    /* Disable the sample module 7 interrupt */
    //EADC_DISABLE_SAMPLE_MODULE_INT(EADC, 0, BIT7);
}

void init_DAC(void){

#ifdef PWM_DAC
	 /*Using EPWM1 CH2 & CH3 */
	/* Start PWM generation for the PWM DACs */
	/* set EPWM to up counter type(edge aligned) */
	EPWM1->CTL1 = (EPWM1->CTL1 & ~(EPWM_CTL1_CNTTYPE2_Msk))|(EPWM_UP_COUNTER << EPWM_CTL1_CNTTYPE2_Pos);
	EPWM1->CTL1 = (EPWM1->CTL1 & ~(EPWM_CTL1_CNTTYPE3_Msk))|(EPWM_UP_COUNTER << EPWM_CTL1_CNTTYPE3_Pos);
	/* set EPWM to auto-reload by clearing CNTMODE bits*/
	EPWM1->CTL1 = (EPWM1->CTL1 & ~(EPWM_CTL1_CNTMODE2_Msk));
	EPWM1->CTL1 = (EPWM1->CTL1 & ~(EPWM_CTL1_CNTMODE3_Msk));

	EPWM1->CLKPSC[1] = 0;/* CLKPSC[1] is EPWM_CLKPSC2_3 which is the prescaler for channels 2 & 3*/

	EPWM1->PERIOD[2] = (uint16_t)DAC_RES_COUNT; /* 192MHz/2048 = 93.75 kHz*/
	EPWM1->PERIOD[3] = (uint16_t)DAC_RES_COUNT;

	EPWM1->CMPDAT[2] = (uint16_t)(DAC_RES_COUNT/2);/* Some initial value*/
	EPWM1->CMPDAT[3] = (uint16_t)(DAC_RES_COUNT/2);

	EPWM1->WGCTL0 = (EPWM1->WGCTL0 & ~(EPWM_WGCTL0_PRDPCTL2_Msk | EPWM_WGCTL0_ZPCTL2_Msk)) | ((uint32_t)EPWM_OUTPUT_HIGH << EPWM_WGCTL0_ZPCTL2_Pos);
	EPWM1->WGCTL0 = (EPWM1->WGCTL0 & ~(EPWM_WGCTL0_PRDPCTL3_Msk | EPWM_WGCTL0_ZPCTL3_Msk)) | ((uint32_t)EPWM_OUTPUT_HIGH << EPWM_WGCTL0_ZPCTL3_Pos);

	EPWM1->WGCTL1 = (EPWM1->WGCTL1 & ~(EPWM_WGCTL1_CMPDCTL2_Msk | EPWM_WGCTL1_CMPUCTL2_Msk)) | ((uint32_t)EPWM_OUTPUT_LOW << EPWM_WGCTL1_CMPUCTL2_Pos);
	EPWM1->WGCTL1 = (EPWM1->WGCTL1 & ~(EPWM_WGCTL1_CMPDCTL3_Msk | EPWM_WGCTL1_CMPUCTL3_Msk)) | ((uint32_t)EPWM_OUTPUT_LOW << EPWM_WGCTL1_CMPUCTL3_Pos);
	//EPWM_ConfigOutputChannel(EPWM1, 5, PWM_CARRIER_FREQ, 50)


	EPWM1->POEN |= EPWM_POEN_POEN2_Msk; /* Enable CH2 (set 1 at bit position 2)*/
	EPWM1->POEN |= EPWM_POEN_POEN3_Msk; /* Enable CH3 (set 1 at bit position 3)*/

	EPWM1->CNTEN |= EPWM_CNTEN_CNTEN2_Msk;
	EPWM1->CNTEN |= EPWM_CNTEN_CNTEN3_Msk;
#else
	/* Set the software trigger DAC and enable D/A converter */
		//DAC_Open(DAC0, 0, DAC_SOFTWARE_TRIGGER);
		DAC_Open(DAC1, 0, DAC_SOFTWARE_TRIGGER);

		//DAC0->CTL |= BIT8; /* Set Bit 8 of the control register to disable the output buffer*/
		//DAC1->CTL |= BIT8; /* Set Bit 8 of the control register to disable the output buffer*/

		/* Enable DAC to work in group mode, once group mode enabled, DAC1 is configured by DAC0 registers */
		//DAC_ENABLE_GROUP_MODE(DAC0);

	    /* The DAC conversion settling time is 1us */
	    //DAC_SetDelayTime(DAC0, 1);
	    DAC_SetDelayTime(DAC1, 6);

	    /* Set DAC 12-bit holding data */
	    DAC_WRITE_DATA(DAC1, 0, 0x400);

	    /* Clear the DAC conversion complete finish flag for safe */
	    DAC_CLR_INT_FLAG(DAC1, 0);

	    /* Enable the DAC interrupt */
	    DAC_ENABLE_INT(DAC1, 0);
	    NVIC_EnableIRQ(DAC_IRQn);

	    /* Start A/D conversion */
	    DAC_START_CONV(DAC1);


#endif

}

void init_BPWM0_duty_capture(void){// BPWM0 for duty cycle capture

	BPWM_SET_PRESCALER(BPWM0, 1, 0);// Set presacler to (0+1). Set to 1 for highest resolution

	/* set BPWM to up count type(edge aligned) */
	BPWM0->CTL1 = BPWM_UP_COUNTER;


	uint32_t BPWM1_CNR;

	BPWM1_CNR = BPWM_GET_CNR(BPWM0, 1); // Normally 480 (192 Mhz cpu freq / 400 kHz PWM carrier freq = 480)

	BPWM_SET_CNR(BPWM0, 1, (BPWM1_CNR + BPWM0_MARGIN)); // Count up to for fastest detection of underrun. BPWM1 cycle lasts 480 counts (Carrier period). Add a small margin on top of that to quickly detect overrun.

	/* Enable Timer for BPWM0 channel 1 */
	BPWM_Start(BPWM0, BPWM_CH_1_MASK);

	/* Enable Capture Function for BPWM0 channel 1 */
	BPWM_EnableCapture(BPWM0, BPWM_CH_1_MASK);

	/* Enable rising capture reload */
	BPWM0->CAPCTL |= BPWM_CAPCTL_RCRLDEN1_Msk;

	/* Wait until BPWM0 channel 0 Timer start to count */
	while((BPWM0->CNT) == 0);

}

void init_BPWM1_carrier_generation(void){ // BPWM1 for PWM carrier generation

	 /*Using BPWM1, */
	/* Begin to output the carrier waveform for the analog hardware PWModulator on PA.6 (pin 16 on the M481LIDAE) */
	BPWM_ConfigOutputChannel(BPWM1, 3, PWM_CARRIER_FREQ, 50); /* Set prescaler to 1, CNT to 480. Use the defined carrier freq, with 50% duty cycle */
	BPWM1->POEN |= BPWM_POEN_POEN3_Msk; /* Enable CH5 (set 1 at bit position 5)*/
	BPWM1->CNTEN = BPWM_CNTEN_CNTEN0_Msk;

	init_DAC();

	delay_ms(10); /* Give the analog modulator some time to stabilize */

}

void init_inverter_control(void){

	/* Start timer 1. Triggers the control loop interrupt */
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, F_CALC); /* Set the interrupt frequency as F_CALC*/
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);
    NVIC_SetPriority(TMR1_IRQn, TMR1_INT_PRIORITY);
    TIMER_Start(TIMER1);

}
