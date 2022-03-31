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
/* Function definitions                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/


/* 363us in Debug, 219us in Release ( -O2 ) */
void draw_UI(int8_t row_sel, int8_t col_sel){/* 60-ish characters width*/

	/* Cursor home : \x1B[H */

    uint8_t p_line_counter = 0;

    /* Current state display */
	draw_UI_line_0_1(&p_line_counter);
	draw_UI_line_2(&p_line_counter);
	draw_UI_line_3(&p_line_counter);
	draw_UI_line_4(&p_line_counter);
	draw_UI_line_5(&p_line_counter);
	draw_UI_line_6(&p_line_counter);
	draw_UI_line_7(&p_line_counter);
	draw_UI_line_8(&p_line_counter);
	draw_UI_line_9(&p_line_counter);
	draw_UI_line_10(&p_line_counter);
	draw_UI_line_11(&p_line_counter);

	draw_UI_line_separator(&p_line_counter);
	/* Menu input*/
	draw_UI_line_A(&p_line_counter,row_sel, col_sel);
	draw_UI_line_B(&p_line_counter,row_sel, col_sel);

	draw_UI_line_separator(&p_line_counter);

	/* Last line */
	static char last_line_str[LINE_WIDTH];
	sprintf(last_line_str,"\x1B[%uA",p_line_counter);
	push_UART2((char*)last_line_str);
	/* End of last line */


	//inverter.V_DC_plus += 0.51; //debug


}

void read_user_input(int8_t* p_row_sel, int8_t* p_col_sel){

  	char user_char;
  	static uint8_t state = 0;
	uint32_t u32IntSts = UART2->INTSTS;

	if(u32IntSts & UART_INTSTS_RDAIF_Msk){/* If there is data to be read*/
		user_char = ((char)UART2->DAT); // read out data

		if (state == 0){/* If no characters have been received*/
			if(user_char == 0x1B){/* If the first character is escape*/
				state++;/* Go to the next state*/
			}

		} else if (state == 1){/* If the first character of an escape sequence (ESC) has been received*/

			if(user_char == '['){/* If the next received character is the 2nd character of an escape sequence*/
				state++;/* Go to the next state*/
			}else{
				state = 0;/* Else reset the state machine*/
			}
		} else if (state == 2){

			if(user_char == 'A'){/* Up */
				(*p_row_sel)--;/*Move the selection up*/
				if(*p_row_sel<0){/* If the selection goes past the highest row*/
					*p_row_sel = 0;/* Stay at the highest row, don't wrap around*/
				}
				state = 0;/* Finally, reset the state machine*/

			}else if (user_char == 'B'){ /* Down*/
				(*p_row_sel)++;/*Move the selection down*/
				if(*p_row_sel >= UI_MENU_NB_ROWS){/* If the selection goes past the lowest row*/
					*p_row_sel = (UI_MENU_NB_ROWS-1);/* Stay at the lowest row, don't wrap around*/
				}
				state = 0;/* Finally, reset the state machine*/

			}else if (user_char == 'C'){ /* Right */
				(*p_col_sel)++;/*Move the selection right*/
				if(*p_col_sel >= UI_MENU_NB_COLUMNS){/* If the selection goes past the lowest row*/
					*p_col_sel = (UI_MENU_NB_COLUMNS-1);/* Stay at the lowest row, don't wrap around*/
				}
				state = 0;/* Finally, reset the state machine*/

			}else if (user_char == 'D'){ /* Left*/
				(*p_col_sel)--;/*Move the selection left*/
				if(*p_col_sel<0){/* If the selection goes past the highest row*/
					*p_col_sel = 0;/* Stay at the highest row, don't wrap around*/
				}
				state = 0;/* Finally, reset the state machine*/

			}else{
				state = 0;/* Else reset the state machine*/
			}

		}

		if(user_char == '+'){/* If a '+' is received, the selected value is incremented*/
			increment_UI_value(*p_row_sel,*p_col_sel);
		}else if (user_char == '-'){/* If a '-' is received, the selected value is decremented*/
			decrement_UI_value(*p_row_sel,*p_col_sel);
		}
		//printf("\nReceived character '0x%x' \n", user_char); /* for debug */

	}
}

void increment_UI_value(int8_t row_sel, int8_t col_sel){

	if(row_sel == 0){
		if(col_sel == 0){
			inverter_setpoints.inverter_active = !inverter_setpoints.inverter_active;
		}else if (col_sel == 1){
			inverter_setpoints.V_DC_total_setpoint += FLOAT_INCREMENT;
		}
	} else if(row_sel == 1){
		if(col_sel == 0){
			inverter_setpoints.some_setpoint += FLOAT_INCREMENT;
		}else if (col_sel == 1){
			inverter_setpoints.V_DC_diff_setpoint += FLOAT_INCREMENT;
		}
	}
}

void decrement_UI_value(int8_t row_sel, int8_t col_sel){

	if(row_sel == 0){
		if(col_sel == 0){
			inverter_setpoints.inverter_active = !inverter_setpoints.inverter_active;
		}else if (col_sel == 1){
			inverter_setpoints.V_DC_total_setpoint -= FLOAT_INCREMENT;
		}
	} else if(row_sel == 1){
		if(col_sel == 0){
			inverter_setpoints.some_setpoint -= FLOAT_INCREMENT;
		}else if (col_sel == 1){
			inverter_setpoints.V_DC_diff_setpoint -= FLOAT_INCREMENT;
		}
	}
}


void draw_UI_line_0_1(uint8_t* p_line_counter) {
	/* Line 0 and 1*/
	static const char line_0_1_str[] =
			"\x1B[0J\n\r****ACTIVE RECTIFIER CONTROLLER V0.1****\n\r";
	/*Escape sequence to clear from cursor to end of screen*/
	*p_line_counter += 2;
	push_UART2((char*) line_0_1_str);
}

void draw_UI_line_2(uint8_t* p_line_counter) {

	static char line_2_str[LINE_WIDTH];
	static char colour_V_DC_plus[ESCAPE_SEQUENCE_LENGTH];
	static char colour_V_DC_minus[ESCAPE_SEQUENCE_LENGTH];
	if (inverter_safety.OV_V_DC_plus) {
		/* if overvoltage */
		strcpy(colour_V_DC_plus, "\x1B[91m"); /* Red*/
	} else if (inverter_safety.UV_V_DC_plus) {
		/* if undervoltage */
		strcpy(colour_V_DC_plus, "\x1B[96m"); /* Cyan*/
	} else {
		strcpy(colour_V_DC_plus, COLOUR_DEFAULT); /* default*/
	}

	if (inverter_safety.OV_V_DC_minus) {
		/* if overvoltage */
		strcpy(colour_V_DC_minus, "\x1B[91m"); /* Red*/
	} else if (inverter_safety.UV_V_DC_minus) {
		/* if undervoltage */
		strcpy(colour_V_DC_minus, "\x1B[96m"); /* Cyan*/
	} else {
		strcpy(colour_V_DC_minus, COLOUR_DEFAULT); /* default*/
	}

	sprintf(line_2_str, "V bus +: %s%2.2f V%s \tV bus -: %s%2.2f V\n\r",
			colour_V_DC_plus, inverter.V_DC_plus,
			COLOUR_DEFAULT, colour_V_DC_minus,
			inverter.V_DC_minus);
	(*p_line_counter)++;
	push_UART2((char*) line_2_str);
}

void draw_UI_line_3(uint8_t* p_line_counter) {

	static char line_3_str[LINE_WIDTH];
	static char colour_V_DC_diff[ESCAPE_SEQUENCE_LENGTH];
	if (inverter_safety.OV_V_DC_diff) {
		/* if the two halves of the bus are too unbalanced */
		strcpy(colour_V_DC_diff, "\x1B[91m"); /* Red*/
	} else {
		strcpy(colour_V_DC_diff, COLOUR_DEFAULT); /* default*/
	}
	sprintf(line_3_str, "V bus diff: %s%2.2f V%s\tV bus total: %2.2f V\n\r",
			colour_V_DC_diff, inverter.V_DC_diff,
			COLOUR_DEFAULT, inverter.V_DC_total);
	(*p_line_counter)++;
	push_UART2((char*) line_3_str);
}

void draw_UI_line_4(uint8_t* p_line_counter) {

	static char line_4_str[LINE_WIDTH];
	sprintf(line_4_str, "I AC RMS: %2.2f A\tV AC RMS: %2.2f V\n\r",
			inverter.I_AC_RMS,
			inverter.V_AC_RMS);
	(*p_line_counter)++;
	push_UART2((char*) line_4_str);
}

void draw_UI_line_5(uint8_t* p_line_counter) {

	static char line_5_str[LINE_WIDTH];
	static char power_flow_dir_str[ESCAPE_SEQUENCE_LENGTH];
	if (inverter.P_AC_RMS > 0) {
		/* If power is positive */
		strcpy(power_flow_dir_str, "->"); /* DC towards AC */
	} else {
		/* Else power is negative */
		strcpy(power_flow_dir_str, "<-"); /* AC towards DC */
	}
	sprintf(line_5_str, "P AC RMS: %2.2f W\tPower flow: DC%sAC\n\r",
			inverter.P_AC_RMS, power_flow_dir_str);
	(*p_line_counter)++;
	push_UART2((char*) line_5_str);
}

void draw_UI_line_6(uint8_t* p_line_counter) {

	static char line_6_str[LINE_WIDTH];
	static char xformer_temp_color_str[ESCAPE_SEQUENCE_LENGTH];
	static char inverter_temp_color_str[ESCAPE_SEQUENCE_LENGTH];
	if (inverter_safety.OT_Transformer) {
		/* If transformer is over temperature */
		strcpy(xformer_temp_color_str, "\x1B[91m"); /* Red */
	} else if (inverter_safety.HT_Transformer) {
		/* Else if the transformer has a high temperature*/
		strcpy(xformer_temp_color_str, "\x1B[93m"); /* Yellow */
	} else {
		/* Else the temperature is OK*/
		strcpy(xformer_temp_color_str, COLOUR_DEFAULT); /* default*/
	}

	if (inverter_safety.OT_Inverter) {
		/* If transformer is over temperature */
		strcpy(inverter_temp_color_str, "\x1B[91m"); /* Red */
	} else if (inverter_safety.HT_Inverter) {
		/* Else if the transformer has a high temperature*/
		strcpy(inverter_temp_color_str, "\x1B[93m"); /* Yellow */
	} else {
		/* Else the temperature is OK*/
		strcpy(inverter_temp_color_str, COLOUR_DEFAULT); /* default*/
	}

	sprintf(line_6_str,
			"Xformer temp: %s%2.1fC%s\tInverter temp: %s%2.1fC%s \n\r",
			xformer_temp_color_str, inverter.T_transformer,
			COLOUR_DEFAULT, inverter_temp_color_str,
			inverter.T_inverter, COLOUR_DEFAULT);
	(*p_line_counter)++;
	push_UART2((char*) line_6_str);
}

void draw_UI_line_7(uint8_t* p_line_counter) {

	static char line_7_str[LINE_WIDTH];
	sprintf(line_7_str, "AC Contactor:");
	if (inverter_safety.AC_contactor_state == CLOSED) {
		strcat(line_7_str, " Closed");
	} else if (inverter_safety.AC_contactor_state == PRECHARGE) {
		strcat(line_7_str, "\x1B[93mPrecharge"); /*Yellow Precharge*/
		strcat(line_7_str, COLOUR_DEFAULT);
	} else if (inverter_safety.AC_contactor_state == OPEN) {
		strcat(line_7_str, " Open");
	}

	strcat(line_7_str, "\tDC Contactor:");
	if (inverter_safety.DC_contactor_state == CLOSED) {
		strcat(line_7_str, " Closed");
	} else if (inverter_safety.DC_contactor_state == PRECHARGE) {
		strcat(line_7_str, "\x1B[93mPrecharge"); /*Yellow Precharge*/
		strcat(line_7_str, COLOUR_DEFAULT);
	} else if (inverter_safety.DC_contactor_state == OPEN) {
		strcat(line_7_str, " Open");
	}

	strcat(line_7_str, "\n\r");
	(*p_line_counter)++;
	push_UART2((char*) line_7_str);
}

void draw_UI_line_8(uint8_t* p_line_counter) {

	static char line_8_str[LINE_WIDTH];
	sprintf(line_8_str, "PLL sync: ");
	if (PLL.sync) {
		strcat(line_8_str, "\x1B[92mYES"); /* Green YES*/
		strcat(line_8_str, COLOUR_DEFAULT);
	} else {
		strcat(line_8_str, "\x1B[93mNO"); /* Yellow NO*/
		strcat(line_8_str, COLOUR_DEFAULT);
	}
	strcat(line_8_str, "\t\ti sync: ");
	if (inverter_safety.i_sync) {
		strcat(line_8_str, "\x1B[92mYES"); /* Green YES*/
		strcat(line_8_str, COLOUR_DEFAULT);
	} else {
		strcat(line_8_str, "\x1B[91mNO"); /* Red NO*/
		strcat(line_8_str, COLOUR_DEFAULT);
	}
	strcat(line_8_str, "\n\r");
	(*p_line_counter)++;
	push_UART2((char*) line_8_str);
}

void draw_UI_line_9(uint8_t* p_line_counter) {

	static char line_9_str[LINE_WIDTH];

	sprintf(line_9_str,"PLL freq: %2.2f Hz\n\r",PLL.freq_Hz);
	(*p_line_counter)++;

	push_UART2((char*) line_9_str);
}

void draw_UI_line_10(uint8_t* p_line_counter) {

	static char line_10_str[LINE_WIDTH];

	sprintf(line_10_str,"Inst. v AC: %2.2f V\n\r",inverter.v_AC);
	(*p_line_counter)++;

	push_UART2((char*) line_10_str);
}

void draw_UI_line_11(uint8_t* p_line_counter) {

	static char line_11_str[LINE_WIDTH];

	sprintf(line_11_str,"Inst. i AC (PV): %2.2f V\n\r",inverter.i_PV);
	(*p_line_counter)++;

	push_UART2((char*) line_11_str);
}


void draw_UI_line_separator(uint8_t* p_line_counter) {

	static char line_separator_str[LINE_WIDTH];
	sprintf(line_separator_str,"****************************************\n\r");
	(*p_line_counter)++;
	push_UART2((char*) line_separator_str);
}

void draw_UI_line_A(uint8_t* p_line_counter, int8_t row_sel, int8_t col_sel) {

	static char line_A_str[LINE_WIDTH];
	char colour_on_off_str[ESCAPE_SEQUENCE_LENGTH];
	char on_off_str[ESCAPE_SEQUENCE_LENGTH];
	char colour_v_setpoint_str[ESCAPE_SEQUENCE_LENGTH];

	/* Column 0 */
	if(row_sel == 0 && col_sel == 0){
		strcpy(colour_on_off_str, COLOUR_SELECTED);
	}else{
		strcpy(colour_on_off_str, COLOUR_NOT_SELECTED);
	}

	if(inverter_setpoints.inverter_active){
		strcpy(on_off_str, "ON");
	}else{
		strcpy(on_off_str, "OFF");
	}

	/* Column 1 */
	if(row_sel == 0 && col_sel == 1){
		strcpy(colour_v_setpoint_str, COLOUR_SELECTED);
	}else{
		strcpy(colour_v_setpoint_str, COLOUR_NOT_SELECTED);
	}


	sprintf(line_A_str,"Inverter: %s%s%s\t\tV DC set: %s%2.2f V%s\n\r",colour_on_off_str,on_off_str,COLOUR_DEFAULT,
			colour_v_setpoint_str,inverter_setpoints.V_DC_total_setpoint,COLOUR_DEFAULT);
	(*p_line_counter)++;

	push_UART2((char*) line_A_str);

}

void draw_UI_line_B(uint8_t* p_line_counter, int8_t row_sel, int8_t col_sel){

	static char line_B_str[LINE_WIDTH];
	char colour_V_diff_str[ESCAPE_SEQUENCE_LENGTH];
	//char on_off_str[ESCAPE_SEQUENCE_LENGTH];
	char colour_other_str[ESCAPE_SEQUENCE_LENGTH];

	/* Column 0 */
	if(row_sel == 1 && col_sel == 0){
		strcpy(colour_other_str, COLOUR_SELECTED);
	}else{
		strcpy(colour_other_str, COLOUR_NOT_SELECTED);
	}

	/* Column 1 */
	if(row_sel == 1 && col_sel == 1){
		strcpy(colour_V_diff_str, COLOUR_SELECTED);
	}else{
		strcpy(colour_V_diff_str, COLOUR_NOT_SELECTED);
	}

	sprintf(line_B_str,"Other: %s%2.2f%s\t\tV DC diff set: %s%2.2f V%s\n\r",colour_other_str,inverter_setpoints.some_setpoint,COLOUR_DEFAULT,
			colour_V_diff_str,inverter_setpoints.V_DC_diff_setpoint,COLOUR_DEFAULT);
	(*p_line_counter)++;

	push_UART2((char*) line_B_str);

}

