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

void draw_UI(void){/* 60-ish characters width*/

	/* Cursor home : \x1B[H */

    uint8_t line_counter = 0;
    uint8_t row_sel = 0;
    uint8_t col_sel = 0;

    //static char COLOUR_DEFAULT[2*ESCAPE_SEQUENCE_LENGTH] = "\x1B[97m\x1B[40m";
    //static char colour_selected[2*ESCAPE_SEQUENCE_LENGTH] = "\x1B[30m\x1B[105m";
    //static char colour_not_selected[2*ESCAPE_SEQUENCE_LENGTH] = "\x1B[95m\x1B[40m";

    /* Current state display */
	draw_UI_line_0_1(&line_counter);
	draw_UI_line_2(&line_counter);
	draw_UI_line_3(&line_counter);
	draw_UI_line_4(&line_counter);
	draw_UI_line_5(&line_counter);
	draw_UI_line_6(&line_counter);
	draw_UI_line_7(&line_counter);
	draw_UI_line_8(&line_counter);
	draw_UI_line_9(&line_counter);
	draw_UI_line_separator(&line_counter);


	/* Menu input*/
	draw_UI_line_A(&line_counter,row_sel, col_sel);
	draw_UI_line_B(&line_counter,row_sel, col_sel);

	/* Last line */
	static char last_line_str[LINE_WIDTH];
	sprintf(last_line_str,"\x1B[%uA",line_counter);
	push_UART1((char*)last_line_str);
	/* End of last line */


	inverter_state_variables.V_DC_plus += 0.51;


}

void draw_UI_line_0_1(uint8_t* line_counter) {
	/* Line 0 and 1*/
	static const char line_0_1_str[] =
			"\x1B[0J\n\r****ACTIVE RECTIFIER CONTROLLER V0.1****\n\r";
	/*Escape sequence to clear from cursor to end of screen*/
	*line_counter += 2;
	push_UART1((char*) line_0_1_str);
}

void draw_UI_line_2(uint8_t* line_counter) {

	static char line_2_str[LINE_WIDTH];
	static char colour_V_DC_plus[ESCAPE_SEQUENCE_LENGTH];
	static char colour_V_DC_minus[ESCAPE_SEQUENCE_LENGTH];
	if (inverter_state_safety.OV_V_DC_plus) {
		/* if overvoltage */
		strcpy(colour_V_DC_plus, "\x1B[91m"); /* Red*/
	} else if (inverter_state_safety.UV_V_DC_plus) {
		/* if undervoltage */
		strcpy(colour_V_DC_plus, "\x1B[96m"); /* Cyan*/
	} else {
		strcpy(colour_V_DC_plus, COLOUR_DEFAULT); /* default*/
	}

	if (inverter_state_safety.OV_V_DC_minus) {
		/* if overvoltage */
		strcpy(colour_V_DC_minus, "\x1B[91m"); /* Red*/
	} else if (inverter_state_safety.UV_V_DC_minus) {
		/* if undervoltage */
		strcpy(colour_V_DC_minus, "\x1B[96m"); /* Cyan*/
	} else {
		strcpy(colour_V_DC_minus, COLOUR_DEFAULT); /* default*/
	}

	sprintf(line_2_str, "V bus +: %s%2.2f V%s \tV bus -: %s%2.2f V\n\r",
			colour_V_DC_plus, inverter_state_variables.V_DC_plus,
			COLOUR_DEFAULT, colour_V_DC_minus,
			inverter_state_variables.V_DC_minus);
	(*line_counter)++;
	push_UART1((char*) line_2_str);
}

void draw_UI_line_3(uint8_t* line_counter) {

	static char line_3_str[LINE_WIDTH];
	static char colour_V_DC_diff[ESCAPE_SEQUENCE_LENGTH];
	if (inverter_state_safety.OV_V_DC_diff) {
		/* if the two halves of the bus are too unbalanced */
		strcpy(colour_V_DC_diff, "\x1B[91m"); /* Red*/
	} else {
		strcpy(colour_V_DC_diff, COLOUR_DEFAULT); /* default*/
	}
	sprintf(line_3_str, "V bus diff: %s%2.2f V%s\tV bus total: %2.2f V\n\r",
			colour_V_DC_diff, inverter_state_variables.V_DC_diff,
			COLOUR_DEFAULT, inverter_state_variables.V_DC_total);
	(*line_counter)++;
	push_UART1((char*) line_3_str);
}

void draw_UI_line_4(uint8_t* line_counter) {

	static char line_4_str[LINE_WIDTH];
	sprintf(line_4_str, "I AC RMS: %2.2f A\tV AC RMS: %2.2f V\n\r",
			inverter_state_variables.I_AC_RMS,
			inverter_state_variables.V_AC_RMS);
	(*line_counter)++;
	push_UART1((char*) line_4_str);
}

void draw_UI_line_5(uint8_t* line_counter) {

	static char line_5_str[LINE_WIDTH];
	static char power_flow_dir_str[ESCAPE_SEQUENCE_LENGTH];
	if (inverter_state_variables.P_AC_RMS > 0) {
		/* If power is positive */
		strcpy(power_flow_dir_str, "->"); /* DC towards AC */
	} else {
		/* Else power is negative */
		strcpy(power_flow_dir_str, "<-"); /* AC towards DC */
	}
	sprintf(line_5_str, "P AC RMS: %2.2f W\tPower flow: DC%sAC\n\r",
			inverter_state_variables.P_AC_RMS, power_flow_dir_str);
	(*line_counter)++;
	push_UART1((char*) line_5_str);
}

void draw_UI_line_6(uint8_t* line_counter) {

	static char line_6_str[LINE_WIDTH];
	static char xformer_temp_color_str[ESCAPE_SEQUENCE_LENGTH];
	static char inverter_temp_color_str[ESCAPE_SEQUENCE_LENGTH];
	if (inverter_state_safety.OT_Transformer) {
		/* If transformer is over temperature */
		strcpy(xformer_temp_color_str, "\x1B[91m"); /* Red */
	} else if (inverter_state_safety.HT_Transformer) {
		/* Else if the transformer has a high temperature*/
		strcpy(xformer_temp_color_str, "\x1B[93m"); /* Yellow */
	} else {
		/* Else the temperature is OK*/
		strcpy(xformer_temp_color_str, COLOUR_DEFAULT); /* default*/
	}

	if (inverter_state_safety.OT_Inverter) {
		/* If transformer is over temperature */
		strcpy(inverter_temp_color_str, "\x1B[91m"); /* Red */
	} else if (inverter_state_safety.HT_Inverter) {
		/* Else if the transformer has a high temperature*/
		strcpy(inverter_temp_color_str, "\x1B[93m"); /* Yellow */
	} else {
		/* Else the temperature is OK*/
		strcpy(inverter_temp_color_str, COLOUR_DEFAULT); /* default*/
	}

	sprintf(line_6_str,
			"Xformer temp: %s%2.1fC%s\tInverter temp: %s%2.1fC%s \n\r",
			xformer_temp_color_str, inverter_state_variables.T_transformer,
			COLOUR_DEFAULT, inverter_temp_color_str,
			inverter_state_variables.T_inverter, COLOUR_DEFAULT);
	(*line_counter)++;
	push_UART1((char*) line_6_str);
}

void draw_UI_line_7(uint8_t* line_counter) {

	static char line_7_str[LINE_WIDTH];
	sprintf(line_7_str, "AC Contactor:");
	if (inverter_state_safety.AC_contactor_state == CLOSED) {
		strcat(line_7_str, " Closed");
	} else if (inverter_state_safety.AC_contactor_state == PRECHARGE) {
		strcat(line_7_str, "\x1B[93mPrecharge"); /*Yellow Precharge*/
		strcat(line_7_str, COLOUR_DEFAULT);
	} else if (inverter_state_safety.AC_contactor_state == OPEN) {
		strcat(line_7_str, " Open");
	}

	strcat(line_7_str, "\tDC Contactor:");
	if (inverter_state_safety.DC_contactor_state == CLOSED) {
		strcat(line_7_str, " Closed");
	} else if (inverter_state_safety.DC_contactor_state == PRECHARGE) {
		strcat(line_7_str, "\x1B[93mPrecharge"); /*Yellow Precharge*/
		strcat(line_7_str, COLOUR_DEFAULT);
	} else if (inverter_state_safety.DC_contactor_state == OPEN) {
		strcat(line_7_str, " Open");
	}

	strcat(line_7_str, "\n\r");
	(*line_counter)++;
	push_UART1((char*) line_7_str);
}

void draw_UI_line_8(uint8_t* line_counter) {

	static char line_8_str[LINE_WIDTH];
	sprintf(line_8_str, "PLL sync: ");
	if (PLL_state_variables.PLL_sync) {
		strcat(line_8_str, "\x1B[92mYES"); /* Green YES*/
		strcat(line_8_str, COLOUR_DEFAULT);
	} else {
		strcat(line_8_str, "\x1B[91mNO"); /* Red NO*/
		strcat(line_8_str, COLOUR_DEFAULT);
	}
	strcat(line_8_str, "\t\ti sync: ");
	if (inverter_state_safety.i_sync) {
		strcat(line_8_str, "\x1B[92mYES"); /* Green YES*/
		strcat(line_8_str, COLOUR_DEFAULT);
	} else {
		strcat(line_8_str, "\x1B[91mNO"); /* Red NO*/
		strcat(line_8_str, COLOUR_DEFAULT);
	}
	strcat(line_8_str, "\n\r");
	(*line_counter)++;
	push_UART1((char*) line_8_str);
}

void draw_UI_line_9(uint8_t* line_counter) {

	static char line_9_str[LINE_WIDTH];

	sprintf(line_9_str,"PLL freq: %2.2f HZ\n\r",PLL_state_variables.PLL_freq_HZ);
	(*line_counter)++;

	push_UART1((char*) line_9_str);
}

void draw_UI_line_separator(uint8_t* line_counter) {

	static char line_separator_str[LINE_WIDTH];
	sprintf(line_separator_str,"****************************************\n\r");
	(*line_counter)++;
	push_UART1((char*) line_separator_str);
}

void draw_UI_line_A(uint8_t* line_counter, uint8_t row_sel, uint8_t col_sel) {

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
	(*line_counter)++;

	push_UART1((char*) line_A_str);

}

void draw_UI_line_B(uint8_t* line_counter, uint8_t row_sel, uint8_t col_sel) {

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
	(*line_counter)++;

	push_UART1((char*) line_B_str);

}

