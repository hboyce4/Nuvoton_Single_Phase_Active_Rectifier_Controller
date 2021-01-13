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

void draw_UI(void){/* 60-ish characters width*/

	/* Cursor home : \x1B[H */

    uint8_t line_counter = 0;
    static char colour_default[ESCAPE_SEQUENCE_LENGTH] = "\x1B[97m";

    /* Line 0 and 1*/
	const static char line_0_1_str[] = "\x1B[0J\n\r****ACTIVE RECTIFIER CONTROLLER V0.1****\n\r"; /*Escape sequence to clear from cursor to end of screen*/
	line_counter += 2;
	push_UART1((char*)line_0_1_str);
	/* End of line 0 and 1 */

	/* Line 2 */
	static char line_2_str[LINE_WIDTH];
	static char colour_V_DC_plus[ESCAPE_SEQUENCE_LENGTH];
	static char colour_V_DC_minus[ESCAPE_SEQUENCE_LENGTH];

	if(inverter_state_safety.OV_V_DC_plus){/* if overvoltage */
		strcpy(colour_V_DC_plus,"\x1B[91m");/* Red*/
	}else if(inverter_state_safety.UV_V_DC_plus){/* if undervoltage */
		strcpy(colour_V_DC_plus,"\x1B[96m");/* Cyan*/
	}else{
	    strcpy(colour_V_DC_plus,colour_default);/* default*/
	}

	if(inverter_state_safety.OV_V_DC_minus){/* if overvoltage */
		strcpy(colour_V_DC_minus,"\x1B[91m");/* Red*/
	}else if(inverter_state_safety.UV_V_DC_minus){/* if undervoltage */
		strcpy(colour_V_DC_minus,"\x1B[96m");/* Cyan*/
	}else{
		strcpy(colour_V_DC_minus,colour_default);/* default*/
	}

	sprintf(line_2_str,"V bus +: %s%2.2f V%s \tV bus -: %s%2.2f V%s\n\r",colour_V_DC_plus,inverter_state_variables.V_DC_plus,colour_default,
			colour_V_DC_minus,inverter_state_variables.V_DC_minus,colour_default);
	line_counter++;

	push_UART1((char*)line_2_str);
	/* End of line 2*/

	/* Line 3 */
	static char line_3_str[LINE_WIDTH];
	static char colour_V_DC_diff[ESCAPE_SEQUENCE_LENGTH];

	if(inverter_state_safety.OV_V_DC_diff){/* if the two halves of the bus are too unbalanced */
		strcpy(colour_V_DC_diff,"\x1B[91m");/* Red*/
	}else{
		strcpy(colour_V_DC_diff,colour_default);/* default*/
	}

	sprintf(line_3_str,"V bus diff: %s%2.2f V%s\tV bus total: %2.2f V\n\r",colour_V_DC_diff,inverter_state_variables.V_DC_diff,colour_default,inverter_state_variables.V_DC_total);
	line_counter++;

	push_UART1((char*)line_3_str);
	/* End of line 3 */


	/* Last line */
	static char last_line_str[LINE_WIDTH];
	sprintf(last_line_str,"\x1B[%uA",line_counter);
	push_UART1((char*)last_line_str);
	/* End of last line */


	inverter_state_variables.V_DC_plus += 0.51;


}
