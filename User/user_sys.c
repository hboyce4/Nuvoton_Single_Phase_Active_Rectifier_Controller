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



/*---------------------------------------------------------------------------------------------------------*/
/* Function definitions                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/

void init_buttons_LEDs(void){


#ifdef TIMING_DEBUG
	GPIO_SetMode(PH, 0xFF, GPIO_MODE_OUTPUT); // 8 first bits of the port as timing debug outputs
#else
	/* Configure PH.0, PH.1 and PH.2 as Output mode for LED blink on ETM-M487 board*/
	GPIO_SetMode(PH, BIT0|BIT1|BIT2, GPIO_MODE_OUTPUT); // LED outputs
	/* Configure PA.5, PA.6 and PA.7 as Output mode for LED blink on M481 boards */
	GPIO_SetMode(PA, BIT5|BIT6|BIT7, GPIO_MODE_OUTPUT); // LED outputs
#endif



    GPIO_SetMode(PG, BIT15, GPIO_MODE_INPUT); // Configure pin as input for Button 1
    GPIO_SetMode(PF, BIT11, GPIO_MODE_INPUT); // Configure pin as input for Button 2

}

void update_buttons_LEDs_state() {

	/* Copy button SW2 state to LEDG */
	uint32_t button_state = PG15;// Acquire PG15 (Button 1) state in new var at bit 0
	button_state <<= 1; // Move PG15 state to bit 1
	button_state |= PF11; /*Button 2*/
	button_state <<= 1; //  Move BTN states to bit 1 and bit 2
	//PH->DOUT &= ~(/*BIT1 |*/ BIT2);
	//PH->DOUT |= (button_state & BIT2);


	static uint16_t heartbeat_counter = 0;

	if(!heartbeat_counter){
		heartbeat_counter = HEARTBEAT_INTERVAL_MS;
#ifdef TIMING_DEBUG
		PH0 ^= 1; // Access single pin (Toggle red LED)
#endif
        //PH->DOUT ^= BIT0|BIT1|BIT2; // Access digital output register (toggle 3 LEDs)
	}
	heartbeat_counter--;

}

void delay_ms(uint32_t delay){ /*Generates a milliseconds delay. NOT ACCURATE. Use a hardware timer for accuracy*/

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

	    NVIC_SetPriority(PDMA_IRQn, PDMA_INT_PRIORITY);



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

//void start_PWModulator_carrier(void){ /* This uses EPWM1 */
//	/* Begin to output the carrier waveform for the analog hardware PWModulator on PB.14 (pin 133 on the M487JIDAE) */
//
//	PB->SLEWCTL |= (GPIO_SLEWCTL_HIGH << 2*14); /*Set PB14 to "High" slew rate.*/
//	/* For some reason "High" mode seems faster than "Fast" mode. Most likely it's just my probing setup (signal reflexions and such) */
//
//	EPWM_ConfigOutputChannel(EPWM1, 1, PWM_CARRIER_FREQ, 50); /* Set prescaler to 1, CNT to 480 */
//
//	EPWM1->POEN |= 0b10; /* Enable CH1 (set 1 at bit position 1)*/
//
//	EPWM1->CNTEN |= 0b10; /* Start the counter (set 1 at bit position 1)*/
//
//}

