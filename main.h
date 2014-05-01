/*
 * main.h
 *
 *  Created on: 16/03/2014
 *      Author: thiago
 */

#ifndef MAIN_H_
#define MAIN_H_

#define triacAux_on()		PORTD |=  (1<<2);	// Triac 1 is the starter
#define triacAux_off()		PORTD &= ~(1<<2);
#define triacStart_on()		PORTB |=  (1<<0);	// Triac 2 is aux
#define triacStart_off()	PORTB &= ~(1<<0);

#define echoPin 4 //Pino 13 recebe o pulso do echo
#define trigPin 3 //Pino 12 envia o pulso para gerar o echo

enum states01 {
	redTime,
	greenTime
};

volatile uint8_t flag01 = 0;
volatile uint8_t flag02 = 0;
volatile uint8_t flag03 = 0;
volatile uint8_t flag04 = 0;
volatile uint8_t flag05 = 0;
volatile uint8_t motorStatus=0;

uint8_t HourOn  = 21;
uint8_t MinOn   = 30;

uint8_t HourOff = 6;
uint8_t MinOff  = 0;

const uint8_t distMin = 130;
const uint8_t distMax = 230;

volatile uint8_t flagSummary=1;

enum states01 periodo = redTime;

#endif /* MAIN_H_ */
