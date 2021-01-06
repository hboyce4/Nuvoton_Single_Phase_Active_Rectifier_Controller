/*
 * user_sys.c
 *
 *  Created on: Jan 5, 2021
 *      Author: Hugo Boyce
 */

#include "user_sys.h"

void update_button_states() {
	/* Copy button SW2 and SW3 states to LEDG and LEDY */
	uint32_t button_state = PG15;
	// Acquire PG15(BTN1) state in new var at bit 0
	button_state <<= 1; // Move PG15 state to bit 1
	button_state |= PF11;
	button_state <<= 1; //  Move BTN states to bit 1 and bit 2
	PH->DOUT &= ~(BIT1 | BIT2);
	PH->DOUT |= button_state;
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

	    UART1->INTEN |= (0b1 << UART_INTEN_TXPDMAEN_Pos); 		/* Bit TXPDMAEN is set to one enable PDMA requests */
	    /* The fist time the DMA is used, the UARTn->INTEN write must absolutely be done at the end */

	    delay_ms(10); /* Make sure the init string is transferred before anything else happens*/

}

void start_UART1_DMA_Xfer(UART_DMA_Xfer_t Xfer){

	PDMA->DSCT[UART1_TX_DMA_CHANNEL].SA = ((uint32_t)Xfer.str); 		/* Starting Source address is the beginning of the string we want to send */
	PDMA->DSCT[UART1_TX_DMA_CHANNEL].CTL = 	(PDMA->DSCT[UART1_TX_DMA_CHANNEL].CTL & ~(PDMA_DSCT_CTL_TXCNT_Msk|PDMA_DSCT_CTL_OPMODE_Msk))|((Xfer.count-1) << PDMA_DSCT_CTL_TXCNT_Pos)|(0b01 << PDMA_DSCT_CTL_OPMODE_Pos); /* OR the string length in the register and set the operating mode from idle to basic*/

}

int8_t push_UART1(char*){
	int8_t ret_val = 0;





	return ret_val;
}

void pop_UART1(void){

	int8_t ret_val;

	start_UART1_DMA_Xfer(Xfer);

}
