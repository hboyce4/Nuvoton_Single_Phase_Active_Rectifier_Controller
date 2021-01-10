/*
 * UI.c
 *
 *  Created on: Jan 10, 2021
 *      Author: Hugo Boyce
 */

#include "UI.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/* Function definitions                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/

void draw_UI(void){

	/* Cursor home : \x1B[H */

    uint8_t line_counter = 0;

    /* Line 0 and 1*/
	const static char new_frame_and_header_str[] = "\x1B[0J\n\r****************MENU TEST****************\n\r";
	line_counter += 2;
	push_UART1((char*)new_frame_and_header_str);

	/* Line 2 */
	static char value1A[LINE_WIDTH];
	static char colour[ESCAPE_SEQUENCE_LENGTH];
	static char colour_default[ESCAPE_SEQUENCE_LENGTH] = "\x1B[97m";
	if(inverter_state_variables.V_DC_plus > 10){
		strcpy(colour,"\x1B[91m");/* Red*/
	}else{
	    strcpy(colour,"\x1B[97m");/* default*/
	}
	sprintf(value1A,"Value1A: %s%.2f V%s\n\r",colour,inverter_state_variables.V_DC_plus,colour_default);
	line_counter++;
	push_UART1((char*)value1A);


	inverter_state_variables.V_DC_plus += 0.51;


}
