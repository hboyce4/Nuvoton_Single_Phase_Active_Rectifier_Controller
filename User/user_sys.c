/*
 * user_sys.c
 *
 *  Created on: Jan 5, 2021
 *      Author: Hugo Boyce
 */

#include "user_sys.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

struct UART1_DMA_Job_Buffer_{
	volatile UART_DMA_Xfer_t Buff[UART_DMA_JOB_BUFF_SIZE];
	volatile uint8_t Head;
	volatile uint8_t Tail;
} UART1_DMA_Job_Buffer = {.Head = 0, .Tail = 0};


uint16_t ADC_raw_val_buff[(EADC_OVERSAMPLING_NUMBER+1)*EADC_TOTAL_CHANNELS];

/*---------------------------------------------------------------------------------------------------------*/
/* Function definitions                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/


void update_button_LED_states() {

	/* Copy button SW2 and SW3 states to LEDG and LEDY */
	uint32_t button_state = PG15;
	// Acquire PG15(BTN1) state in new var at bit 0
	button_state <<= 1; // Move PG15 state to bit 1
	button_state |= PF11;
	button_state <<= 1; //  Move BTN states to bit 1 and bit 2
	PH->DOUT &= ~(/*BIT1 |*/ BIT2);
	PH->DOUT |= (button_state & BIT2);


	static uint16_t heartbeat_counter = 0;

	if(!heartbeat_counter){
		heartbeat_counter = HEARTBEAT_INTERVAL_MS;
		PH0 ^= 1; // Access single pin (Toggle red LED)
        //PH->DOUT ^= BIT0|BIT1|BIT2; // Access digital output register (toggle 3 LEDs)
	}
	heartbeat_counter--;

}

void delay_ms(uint32_t delay){ /*Generates a millisecons delay. NOT ACCURATE. Use a hardware timer for accuracy*/

	uint64_t end_time = g_SysTickIntCnt + ((uint64_t)delay);

	while(g_SysTickIntCnt <= end_time){ /* As long as the end time is not reached*/
		/* Do nothing*/
	}
}

void init_UART1_DMA(void){

		static const char init_str[] = "UART1 over DMA channel started...\n\r";
		uint16_t init_str_len = strlen(init_str);

		PDMA->CHCTL = (0b1 << UART1_TX_DMA_CHANNEL);			/* The channel we want to use is enabled in the CHCTL register */

	    PDMA->REQSEL8_11 = (PDMA->REQSEL8_11 & ~PDMA_REQSEL8_11_REQSRC10_Msk)|(PDMA_UART1_TX << PDMA_REQSEL8_11_REQSRC10_Pos); /* connect channel to UART1_TX. Channel 0 is used */

	    PDMA->DSCT[UART1_TX_DMA_CHANNEL].CTL = (0b1 << PDMA_DSCT_CTL_TXTYPE_Pos)|	/* 0b1 for TX type = single transfer*/
												(0b11 << PDMA_DSCT_CTL_DAINC_Pos)	/* 0b11 for no increment of destination address (fixed address)*/
											;

	    PDMA->DSCT[UART1_TX_DMA_CHANNEL].CTL = 	(PDMA->DSCT[UART1_TX_DMA_CHANNEL].CTL & ~(PDMA_DSCT_CTL_TXCNT_Msk|PDMA_DSCT_CTL_OPMODE_Msk))|((init_str_len-1) << PDMA_DSCT_CTL_TXCNT_Pos)|(0b01 << PDMA_DSCT_CTL_OPMODE_Pos); /* OR the string length in the register and set the operating mode from idle to basic*/

	    PDMA->DSCT[UART1_TX_DMA_CHANNEL].DA = ((uint32_t)&(UART1->DAT)); 	/* Destination address is the UART0 data register */

	    PDMA->DSCT[UART1_TX_DMA_CHANNEL].SA = ((uint32_t)init_str); 		/* Starting Source address is the beginning of the string we want to send */

	    UART1->INTEN |= (UART_INTEN_TXPDMAEN_Msk /*| UART_INTEN_RDAIEN_Msk*/); 		/* Bit TXPDMAEN is set to one enable PDMA requests. RDAIEN is set to generate interrupt on character receive*/
	    /* The fist time the DMA is used, the UARTn->INTEN write must absolutely be done at the end */

	    delay_ms(10); /* Make sure the init string is transferred before anything else happens*/

	    PDMA_EnableInt(PDMA,UART1_TX_DMA_CHANNEL, PDMA_INT_TRANS_DONE);
	    NVIC_EnableIRQ(PDMA_IRQn);

	    NVIC_SetPriority(PDMA_IRQn, DMA_INTERRUPT_PRIORITY);



}

void start_UART1_DMA_Xfer(UART_DMA_Xfer_t Xfer){

	PDMA->DSCT[UART1_TX_DMA_CHANNEL].SA = ((uint32_t)Xfer.str); 		/* Starting Source address is the beginning of the string we want to send */
	PDMA->DSCT[UART1_TX_DMA_CHANNEL].CTL = 	(PDMA->DSCT[UART1_TX_DMA_CHANNEL].CTL & ~(PDMA_DSCT_CTL_TXCNT_Msk|PDMA_DSCT_CTL_OPMODE_Msk))|((Xfer.count-1) << PDMA_DSCT_CTL_TXCNT_Pos)|(0b01 << PDMA_DSCT_CTL_OPMODE_Pos); /* OR the string length in the register and set the operating mode from idle to basic*/

}

int8_t push_UART1(char* p_str){

	UART_DMA_Xfer_t temp;
	temp.str = p_str;
	temp.count = strlen(p_str);

	if(!(PDMA->DSCT[UART1_TX_DMA_CHANNEL].CTL & PDMA_DSCT_CTL_OPMODE_Msk)){ /* If the DMA channel is idle (free)*/

		start_UART1_DMA_Xfer(temp);/* Start the transfer right away */

	}else{ /* Else, the channel is busy so store the job in the buffer*/

		uint8_t Next = UART1_DMA_Job_Buffer.Head + 1;

		Next &= (UART_DMA_JOB_BUFF_SIZE - 1); /* Force overflow*/

		if (Next == UART1_DMA_Job_Buffer.Tail){	// If Buffer Full -HB

			printf("WARNING : UART1 job buffer overflow!!!");	// Error message
			delay_ms(100);				// Wait for it to be emptied

			return 12;				// Return "out if memory". Buffer full.
		}

		UART1_DMA_Job_Buffer.Buff[UART1_DMA_Job_Buffer.Head] = temp;
		UART1_DMA_Job_Buffer.Head = Next;

	}

	return 0;					// Return Success -HB
}

int8_t pop_UART1(UART_DMA_Xfer_t* p_Xfer){

	if (UART1_DMA_Job_Buffer.Head == UART1_DMA_Job_Buffer.Tail){// Buffer empty -HB

			return 2;	// No data to pop out of the buffer -HB
		}

		uint8_t Next = UART1_DMA_Job_Buffer.Tail + 1;

		Next &= (UART_DMA_JOB_BUFF_SIZE - 1);

		*p_Xfer = UART1_DMA_Job_Buffer.Buff[UART1_DMA_Job_Buffer.Tail];

		UART1_DMA_Job_Buffer.Tail = Next;

		return 0;	// return Success -HB

}

void start_PWModulator_carrier(void){
	/* Begin to output the carrier waveform for the analog hardware PWModulator on PB.14 (pin 133 on the M487JIDAE) */

	PB->SLEWCTL |= (GPIO_SLEWCTL_HIGH << 2*14); /*Set PB14 to "High" slew rate.*/
	/* For some reason "High" mode seems faster than "Fast" mode. Most likely it's just my probing setup (signal reflexions and such) */

	EPWM_ConfigOutputChannel(EPWM1, 1, PWM_CARRIER_FREQ, 50); /* Set prescaler to 1, CNT to 480 */

	EPWM1->POEN |= 0b10; /* Enable CH1 (set 1 at bit position 1)*/

	EPWM1->CNTEN |= 0b10; /* Start the counter (set 1 at bit position 1)*/

}

void init_ADC(void){


    /* Set input mode as single-end and enable the A/D converter */
    EADC_Open(EADC, EADC_CTL_DIFFEN_SINGLE_END);

    /* Configure the sample 4 module for analog input channel 0 and enable ADINT0 trigger source */
    EADC_ConfigSampleModule(EADC, 0, EADC_ADINT0_TRIGGER, 0);
    /* Configure the sample 5 module for analog input channel 1 and enable ADINT0 trigger source */
    EADC_ConfigSampleModule(EADC, 1, EADC_ADINT0_TRIGGER, 1);
    /* Configure the sample 6 module for analog input channel 2 and enable ADINT0 trigger source */
    EADC_ConfigSampleModule(EADC, 2, EADC_ADINT0_TRIGGER, 2);
    /* Configure the sample 7 module for analog input channel 3 and enable ADINT0 trigger source */
    EADC_ConfigSampleModule(EADC, 3, EADC_ADINT0_TRIGGER, 3);
    /* Configure the sample 4 module for analog input channel 0 and enable ADINT0 trigger source */
    EADC_ConfigSampleModule(EADC, 4, EADC_ADINT0_TRIGGER, 4);
    /* Configure the sample 5 module for analog input channel 1 and enable ADINT0 trigger source */
    EADC_ConfigSampleModule(EADC, 5, EADC_ADINT0_TRIGGER, 5);
    /* Configure the sample 6 module for analog input channel 2 and enable ADINT0 trigger source */
    EADC_ConfigSampleModule(EADC, 6, EADC_ADINT0_TRIGGER, 6);
    /* Configure the sample 7 module for analog input channel 3 and enable ADINT0 trigger source */
    EADC_ConfigSampleModule(EADC, 7, EADC_ADINT0_TRIGGER, 7);

    /* Clear the A/D ADINT0 interrupt flag for safety */
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);

    /* Enable the sample module 7 interrupt */
    EADC_ENABLE_INT(EADC, BIT0);//Enable sample module  A/D ADINT0 interrupt.
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 0, BIT7);//Enable sample module 7 interrupt.
    //NVIC_EnableIRQ(EADC00_IRQn);

    /* Reset the ADC indicator and trigger sample module 7 to start A/D conversion */
    //g_u32AdcIntFlag = 0;
    //g_u32COVNUMFlag = 0;
    EADC_START_CONV(EADC, BIT7);

    //__WFI();

    /* Disable the sample module 7 interrupt */
    //EADC_DISABLE_SAMPLE_MODULE_INT(EADC, 0, BIT7);
}

void run_ADC_cal(void){

	EADC->CALCTL |= EADC_CALCTL_CALSTART_Msk;
}
//void init_ADC_DMA(void){
//	   /* Configure PDMA peripheral mode form EADC to memory */
//
//			PDMA->CHCTL = (0b1 << EADC_DMA_CHANNEL);			/* The channel we want to use is enabled in the CHCTL register */
//
//		    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & ~PDMA_REQSEL0_3_REQSRC2_Msk)|(PDMA_EADC0_RX << PDMA_REQSEL0_3_REQSRC2_Pos); /* connect channel to EADC0_TX. Channel 2 is used */
//
//		    PDMA->DSCT[EADC_DMA_CHANNEL].CTL = PDMA_REQ_SINGLE|PDMA_SAR_FIX|PDMA_WIDTH_16;	/* 0b1 for TX type = single transfer*/
//														/* 0b11 for no increment of source address (fixed address)*/
//
//		    PDMA->DSCT[EADC_DMA_CHANNEL].CTL = 	(PDMA->DSCT[EADC_DMA_CHANNEL].CTL & ~(PDMA_DSCT_CTL_TXCNT_Msk|PDMA_DSCT_CTL_OPMODE_Msk))|((((EADC_OVERSAMPLING_NUMBER+1)*EADC_TOTAL_CHANNELS)-1) << PDMA_DSCT_CTL_TXCNT_Pos)|PDMA_OP_BASIC; /* OR the string length in the register and set the operating mode from idle to basic*/
//
//		    PDMA->DSCT[EADC_DMA_CHANNEL].DA = ((uint32_t)&ADC_raw_val_buff); 	/* Destination address is the storage struct */
//
//		    PDMA->DSCT[EADC_DMA_CHANNEL].SA = ((uint32_t)&(EADC->CURDAT)); 		/* The constant source address is the EADC CURDAT register */
//
//		    EADC->INTEN |= (UART_INTEN_TXPDMAEN_Msk /*| UART_INTEN_RDAIEN_Msk*/); 		/* Bit TXPDMAEN is set to one enable PDMA requests. RDAIEN is set to generate interrupt on character receive*/
//		    /* The fist time the DMA is used, the UARTn->INTEN write must absolutely be done at the end */
//
//		    //delay_ms(10); /* Make sure the init string is transferred before anything else happens*/
//
//		    EADC_ENABLE_SAMPLE_MODULE_PDMA();
//
//		    PDMA_EnableInt(PDMA,EADC_DMA_CHANNEL, PDMA_INT_TRANS_DONE);
//		    NVIC_EnableIRQ(PDMA_IRQn);
//
//		    NVIC_SetPriority(PDMA_IRQn, DMA_INTERRUPT_PRIORITY);
//
//
//}

//void start_ADC_acq(void){
//
//	/*Reload DMA*/
//	PDMA->DSCT[EADC_DMA_CHANNEL].CTL = 	(PDMA->DSCT[EADC_DMA_CHANNEL].CTL & ~(PDMA_DSCT_CTL_TXCNT_Msk|PDMA_DSCT_CTL_OPMODE_Msk))|
//			((((EADC_OVERSAMPLING_NUMBER+1)*EADC_TOTAL_CHANNELS)-1) << PDMA_DSCT_CTL_TXCNT_Pos)|PDMA_OP_BASIC; /* OR the string length in the register and set the operating mode from idle to basic*/
//
//	/*Enable ADINT0 by sample module 7 for self triggering in a loop*/
//	EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 0, BIT7);//Enable sample module 7 interrupt.
//
//	/*Trigger sample module 7 to */
//
//}
