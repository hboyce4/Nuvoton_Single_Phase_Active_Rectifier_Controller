/*
 * UART_over_DMA.c
 *
 *  Created on: Jan 5, 2021
 *      Author: Hugo Boyce
 */

#include "UART_over_DMA.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

struct UART2_DMA_Job_Buffer_{
	volatile UART_DMA_Xfer_t Buff[UART_DMA_JOB_BUFF_SIZE];
	volatile uint8_t Head;
	volatile uint8_t Tail;
} UART2_DMA_Job_Buffer = {.Head = 0, .Tail = 0};



/*---------------------------------------------------------------------------------------------------------*/
/* Function definitions                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/






void init_UART2_DMA(void){

		static const char init_str[] = "UART2 over DMA channel started...\n\r";
		uint16_t init_str_len = strlen(init_str);

		PDMA->CHCTL = (0b1 << UART2_TX_DMA_CHANNEL);			/* The channel we want to use is enabled in the CHCTL register */

	    PDMA->REQSEL8_11 = (PDMA->REQSEL8_11 & ~PDMA_REQSEL8_11_REQSRC10_Msk)|(PDMA_UART2_TX << PDMA_REQSEL8_11_REQSRC10_Pos); /* connect PDMA channel 10 to UART2_TX. Channel 10 is used */

	    PDMA->DSCT[UART2_TX_DMA_CHANNEL].CTL = (0b1 << PDMA_DSCT_CTL_TXTYPE_Pos)|	/* 0b1 for TX type = single transfer*/
												(0b11 << PDMA_DSCT_CTL_DAINC_Pos)	/* 0b11 for no increment of destination address (fixed address)*/
											;

	    PDMA->DSCT[UART2_TX_DMA_CHANNEL].CTL = 	(PDMA->DSCT[UART2_TX_DMA_CHANNEL].CTL & ~(PDMA_DSCT_CTL_TXCNT_Msk|PDMA_DSCT_CTL_OPMODE_Msk))|((init_str_len-1) << PDMA_DSCT_CTL_TXCNT_Pos)|(0b01 << PDMA_DSCT_CTL_OPMODE_Pos); /* OR the string length in the register and set the operating mode from idle to basic*/

	    PDMA->DSCT[UART2_TX_DMA_CHANNEL].DA = ((uint32_t)&(UART2->DAT)); 	/* Destination address is the UART0 data register */

	    PDMA->DSCT[UART2_TX_DMA_CHANNEL].SA = ((uint32_t)init_str); 		/* Starting Source address is the beginning of the string we want to send */

	    UART2->INTEN |= (UART_INTEN_TXPDMAEN_Msk /*| UART_INTEN_RDAIEN_Msk*/); 		/* Bit TXPDMAEN is set to one enable PDMA requests. RDAIEN is set to generate interrupt on character receive*/
	    /* The fist time the DMA is used, the UARTn->INTEN write must absolutely be done at the end */

	    delay_ms(10); /* Make sure the init string is transferred before anything else happens*/

	    PDMA_EnableInt(PDMA,UART2_TX_DMA_CHANNEL, PDMA_INT_TRANS_DONE);
	    NVIC_EnableIRQ(PDMA_IRQn);

	    NVIC_SetPriority(PDMA_IRQn, PDMA_INT_PRIORITY);



}

void start_UART2_DMA_Xfer(UART_DMA_Xfer_t Xfer){

	PDMA->DSCT[UART2_TX_DMA_CHANNEL].SA = ((uint32_t)Xfer.str); 		/* Starting Source address is the beginning of the string we want to send */
	PDMA->DSCT[UART2_TX_DMA_CHANNEL].CTL = 	(PDMA->DSCT[UART2_TX_DMA_CHANNEL].CTL & ~(PDMA_DSCT_CTL_TXCNT_Msk|PDMA_DSCT_CTL_OPMODE_Msk))|((Xfer.count-1) << PDMA_DSCT_CTL_TXCNT_Pos)|(0b01 << PDMA_DSCT_CTL_OPMODE_Pos); /* OR the string length in the register and set the operating mode from idle to basic*/

}

int8_t push_UART2(char* p_str){

	UART_DMA_Xfer_t temp;
	temp.str = p_str;
	temp.count = strlen(p_str);

	if(!(PDMA->DSCT[UART2_TX_DMA_CHANNEL].CTL & PDMA_DSCT_CTL_OPMODE_Msk)){ /* If the DMA channel is idle (free)*/

		start_UART2_DMA_Xfer(temp);/* Start the transfer right away */

	}else{ /* Else, the channel is busy so store the job in the buffer*/

		uint8_t Next = UART2_DMA_Job_Buffer.Head + 1;

		Next &= (UART_DMA_JOB_BUFF_SIZE - 1); /* Force overflow*/

		if (Next == UART2_DMA_Job_Buffer.Tail){	// If Buffer Full -HB

			printf("WARNING : UART2 job buffer overflow!!!");	// Error message
			delay_ms(100);				// Wait for it to be emptied

			return 12;				// Return "out if memory". Buffer full.
		}

		UART2_DMA_Job_Buffer.Buff[UART2_DMA_Job_Buffer.Head] = temp;
		UART2_DMA_Job_Buffer.Head = Next;

	}

	return 0;					// Return Success -HB
}

int8_t pop_UART2(UART_DMA_Xfer_t* p_Xfer){

	if (UART2_DMA_Job_Buffer.Head == UART2_DMA_Job_Buffer.Tail){// Buffer empty -HB

			return 2;	// No data to pop out of the buffer -HB
		}

		uint8_t Next = UART2_DMA_Job_Buffer.Tail + 1;

		Next &= (UART_DMA_JOB_BUFF_SIZE - 1);

		*p_Xfer = UART2_DMA_Job_Buffer.Buff[UART2_DMA_Job_Buffer.Tail];

		UART2_DMA_Job_Buffer.Tail = Next;

		return 0;	// return Success -HB

}

void UART_DMA_process_interrupt(void) {
	uint32_t status = PDMA_GET_INT_STATUS(PDMA);
	if (status & 0x2) {
		/* If the interrupt is "Transfer Done" */
		if (PDMA_GET_TD_STS(PDMA) & (1 << UART2_TX_DMA_CHANNEL)) {
			/* If the transfer that was completed is the UART2_TX_DMA_CHANNEL*/
			PDMA_CLR_TD_FLAG(PDMA, 1 << UART2_TX_DMA_CHANNEL);
			UART_DMA_Xfer_t temp;
			if (!pop_UART2(&temp)) {
				// If popping a transfer "job" from the buffer succeeds
				start_UART2_DMA_Xfer(temp); // Start the transfer
			}
			/* else, popping from the buffer didn't not succeed, it means there's no transfer jobs left */} else {
			printf("PDMA int. from unknown ch.!!\n");
			PDMA_CLR_TD_FLAG(PDMA, PDMA_GET_TD_STS(PDMA));
		}
	} else if (status & 0x1) {
		/* abort */
		printf("PDMA target abort int.!!\n");
		if (PDMA_GET_ABORT_STS(PDMA) & 0x4)
			PDMA_CLR_ABORT_FLAG(PDMA, PDMA_GET_ABORT_STS(PDMA));
	} else if (status & 0x300) {
		/* channel 2 timeout */
		printf(" PDMA timeout int.!!\n");
	} else {
		printf("unknown PDMA int.!!\n");
	}
}
