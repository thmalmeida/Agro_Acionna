#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <Arduino.h>

#include <avr/wdt.h>

#include "RTC/Time.h"
#include "RTC/DS1307RTC.h"

const char *monthName[12] = {
	"Jan", "Feb", "Mar", "Apr", "May", "Jun",
	"Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

tmElements_t tm;
uint8_t opcode;

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

uint8_t flag_LL = 0;
uint8_t flag_MLL = 0;
uint8_t flag_HL = 0;

uint8_t flag_01 = 0;
uint8_t flag_02 = 0;
uint8_t flag_03 = 0;
uint8_t motorStatus = 0;

uint8_t HourOn  = 21;
uint8_t MinOn   = 30;

uint8_t HourOff = 6;
uint8_t MinOff  = 0;

volatile uint8_t flag_1s=1;

enum states01 periodo = redTime;

const uint8_t pkt_size = 10;
uint8_t pkt_Tx[pkt_size], pkt_Rx[pkt_size];
uint8_t k, rLength, j;
char aux[3], buffer[40], inChar, sInstr[20];
char sInstrBluetooth[30];

uint8_t enableSend = 0;
uint8_t enableDecode = 0;
uint8_t enableTranslate = 0;
uint8_t flagSync = 0;
uint8_t countSync = 0;
uint8_t flag_sendContinuously = 0;
uint8_t flag_reset = 0;

uint8_t j2 = 0;
uint8_t flag_frameStartBT = 0;
uint8_t enableTranslate_Bluetooth = 0;

int distance=0;

// Logs
const int nLog = 10;
uint8_t hourLog_ON[nLog], minuteLog_ON[nLog], secondLog_ON[nLog];
uint8_t hourLog_OFF[nLog], minuteLog_OFF[nLog], secondLog_OFF[nLog];
uint8_t dayLog_ON[nLog], monthLog_ON[nLog], YearLog_ON[nLog];
uint8_t dayLog_OFF[nLog], monthLog_OFF[nLog], YearLog_OFF[nLog];
int distanceLog_ON[nLog], distanceLog_OFF[nLog];

uint16_t levelSensorLL, levelSensorMLL, levelSensorMLH, levelSensorHL;
uint16_t levelSensorLL_d, levelSensorMLL_d, levelSensorMLH_d, levelSensorHL_d;

void print2digits(int number)
{
	if (number >= 0 && number < 10)
	{
		Serial.write('0');
	}
	Serial.print(number);
}
bool getTime(const char *str)
{
	int Hour, Min, Sec;

	if (sscanf(str, "%d:%d:%d", &Hour, &Min, &Sec) != 3) return false;
	tm.Hour = Hour;
	tm.Minute = Min;
	tm.Second = Sec;
	return true;
}
bool getDate(const char *str)
{
	char Month[12];
	int Day, Year;
	uint8_t monthIndex;

	if (sscanf(str, "%s %d %d", Month, &Day, &Year) != 3) return false;
	for (monthIndex = 0; monthIndex < 12; monthIndex++)
	{
		if (strcmp(Month, monthName[monthIndex]) == 0) break;
	}
	if (monthIndex >= 12) return false;
	tm.Day = Day;
	tm.Month = monthIndex + 1;
	tm.Year = CalendarYrToTm(Year);

	return true;
}
void clockSync_PC()
{
//	bool parse=false;
//	bool config=false;
//
//	// get the date and time the compiler was run
//	if (getDate(__DATE__) && getTime(__TIME__))
//	{
//		parse = true;
//		// and configure the RTC with this info
//		if (RTC.write(tm))
//		{
//			config = true;
//		}
//	}
//
}
void init_WDT()
{
//	cli();
//	wdt_reset();
//	/* Clear WDRF in MCUSR */
//	MCUSR &= ~(1<<WDRF);
//	/* Write logical one to WDCE and WDE */
//	/* Keep old prescaler setting to prevent unintentional time-out */
//	WDTCSR |= (1<<WDCE) | (1<<WDE);
//	/* Turn off WDT */
//	WDTCSR = 0x00;
//	sei();



	// Configuring to enable only Reset System if occurs 4 s timeout
//	WDTCSR <== WDIF WDIE WDP3 WDCE WDE WDP2 WDP1 WDP0
//	WDTCSR |=  (1<<WDCE) | (1<<WDE);	// Enable Watchdog Timer
//	WDTCSR &= ~(1<<WDIE);				// Disable interrupt
//
//	WDTCSR |=  (1<<WDP3);				// 512k (524288) Cycles, 4.0s
//	WDTCSR &= ~(1<<WDP2);
//	WDTCSR &= ~(1<<WDP1);
//	WDTCSR &= ~(1<<WDP0);

//	WDTCSR |=  (1<<WDCE);
//	WDTCSR = 0b00111000;

//	wdt_enable(WDTO_8S);
	// WDT enable

	wdt_enable(WDTO_8S);
}
void init_ADC()
{
//	ADCSRA ==> ADEN ADSC ADATE ADIF ADIE ADPS2 ADPS1 ADPS0
	ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);	// Set 128 division clock
	ADCSRA |= (1<<ADEN); 		// Enable module

//	ADCSRB ==>	–	ACME	–	–	MUX5	ADTS2	ADTS1	ADTS0
	ADCSRB &= ~(1<<ADTS2);		// Free running mode.
	ADCSRB &= ~(1<<ADTS1);
	ADCSRB &= ~(1<<ADTS0);

//	ADMUX ==> REFS1 REFS0 ADLAR MUX4 MUX3 MUX2 MUX1 MUX0
//	ADMUX |=  (1<<REFS0);		// Internal 1.1V reference
//	ADMUX |=  (1<<REFS1);

	ADMUX &=  ~(1<<REFS1);		//AVCC with external capacitor at AREF pin
	ADMUX |=   (1<<REFS0);

	ADMUX &= ~(1<<ADLAR);		// Right adjustment. To ADCL register.

//	ADMUX &= ~(1<<MUX3);		// Select ADC0
//	ADMUX &= ~(1<<MUX2);
//	ADMUX &= ~(1<<MUX1);
//	ADMUX &= ~(1<<MUX0);

//	DDRC &= ~(1<<PC0);
//	DDRC &= ~(1<<PC1);
//	DDRC &= ~(1<<PC2);

}
void init_Timer1_1Hz()
{
	// Timer 1 with 16 bit time counter. On a Fast PWM
	// TCCR1A <==	COM1A1	COM1A0	COM1B1	COM1B0	COM1C1	COM1C0	WGM11	WGM10
	TCCR1A = 0b00000010;

	// TCCR1B <==	ICNC1	ICES1	–		WGM13	WGM12	CS12	CS11	CS10
	TCCR1B = 0b00011101; // Start timer at Fcpu/1024

	// TIMSK1 <==	–		–		ICIE1	–		OCIE1C	OCIE1B	OCIE1A	TOIE1
//	TIMSK1 |= (1 << OCIE1A);
	TIMSK1 = 0b00000010;

	ICR1 = 15624; // To obtain 1Hz clock.
}
void init_IO()
{
//	DDRD &= ~(1 << 2);
//	DDRD &= ~(1 << 3);
//	DDRD &= ~(1 << 4);
//
//	DDRB &= ~(1 << 1);
//	DDRB &= ~(1 << 2);
//
//	DDRC &= ~(1 << 3);
//
//	DDRC = 0x00;
//	DDRD = 0x00;

	// Set triac1, triac2 and led connected pins as output
	DDRD |= (1 << 5); // Led!
	DDRB |= (1 << 0); // Motor

	pinMode(echoPin, INPUT); // define o pino 13 como entrada (recebe)
	pinMode(trigPin, OUTPUT); // define o pino 12 como saida (envia)
}
void init_pwm2()
{
	TCCR2A = 0b00000010; // TCCR2A ==>> COM2A1 COM2A0 COM2B1 COM2B0 - - WGM21 WGM20
	TCCR2B = 0b00000111; // TCCR2B ==>> FOC2A FOC2B - - WGM22 CS22 CS21 CS20
	TIMSK2 = 0b00000010; // TIMSK2 ==>> - - - - - OCIE2B OCIE2A TOIE2
	OCR2A = 251;

}
void stop_pwm2()
{
//	TCCR2A ==>> COM2A1 COM2A0 COM2B1 COM2B0 - - WGM21 WGM20
	TCCR2A = 0b00000011;

	// TCCR2B ==> FOC2A FOC2B - - WGM22 CS22 CS21 CS20
	TCCR2B = 0b00001000;
	OCR2A = 200;

	// TIMSK2 ==>> - - - - - OCIE2B OCIE2A TOIE2
	TIMSK2 = 0b00000010;

//     Turn Red Led off.
	PORTD &= ~(1 << PD2);
}

void motor_start()
{
	triacStart_on();
	motorStatus = 1;

	int i;
	for(i=(nLog-1);i>=0;i--)
	{
		hourLog_ON[i] = hourLog_ON[i-1];
		minuteLog_ON[i] = minuteLog_ON[i-1];
		secondLog_ON[i] = secondLog_ON[i-1];
		dayLog_ON[i] = dayLog_ON[i-1];
		monthLog_ON[i] = monthLog_ON[i-1];
		YearLog_ON[i] = YearLog_ON[i-1];
		distanceLog_ON[i] = distanceLog_ON[i-1];
	}

	hourLog_ON[0] = tm.Hour;
	minuteLog_ON[0] = tm.Minute;
	secondLog_ON[0] = tm.Second;
	dayLog_ON[0] = tm.Day;
	monthLog_ON[0] = tm.Month;
	YearLog_ON[0] = tm.Year;
	distanceLog_ON[0] = distance;

}
void motor_stop()
{
	triacStart_off();
	motorStatus = 0;

	int i;
	for(i=(nLog-1);i>=0;i--)
	{
		hourLog_OFF[i] = hourLog_OFF[i-1];
		minuteLog_OFF[i] = minuteLog_OFF[i-1];
		secondLog_OFF[i] = secondLog_OFF[i-1];
		dayLog_OFF[i] = dayLog_OFF[i-1];
		monthLog_OFF[i] = monthLog_OFF[i-1];
		YearLog_OFF[i] = YearLog_OFF[i-1];
		distanceLog_OFF[i] = distanceLog_OFF[i-1];
	}

	hourLog_OFF[0] = tm.Hour;
	minuteLog_OFF[0] = tm.Minute;
	secondLog_OFF[0] = tm.Second;
	dayLog_OFF[0] = tm.Day;
	monthLog_OFF[0] = tm.Month;
	YearLog_OFF[0] = tm.Year;
	distanceLog_OFF[0] = distance;

}

void sensorRead_Level()
{
	uint8_t low, high;
	uint16_t value;
	const uint16_t reference = 300;

	ADMUX &= ~(1<<MUX3);							// Select ADC0
	ADMUX &= ~(1<<MUX2);
	ADMUX &= ~(1<<MUX1);
	ADMUX &= ~(1<<MUX0);

	ADCSRA |= (1<<ADSC);				// Start conversion;
	while (bit_is_set(ADCSRA, ADSC));	// wait until conversion done;

	low  = ADCL;
	high = ADCH;
	value = (high << 8) | low;
	levelSensorLL_d = value;

	if(value<reference)
		levelSensorLL = 1;
	else
		levelSensorLL = 0;


	ADMUX &= ~(1<<MUX3);							// Select ADC1
	ADMUX &= ~(1<<MUX2);
	ADMUX &= ~(1<<MUX1);
	ADMUX |=  (1<<MUX0);

	ADCSRA |= (1<<ADSC);				// Start conversion;
	while (bit_is_set(ADCSRA, ADSC));	// wait until conversion done;

	low  = ADCL;
	high = ADCH;
	value = (high << 8) | low;
	levelSensorMLL_d = value;

	if(value<reference)
		levelSensorMLL = 1;
	else
		levelSensorMLL = 0;


	ADMUX &= ~(1<<MUX3);							// Select ADC2
	ADMUX &= ~(1<<MUX2);
	ADMUX |=  (1<<MUX1);
	ADMUX &= ~(1<<MUX0);

	ADCSRA |= (1<<ADSC);				// Start conversion;
	while (bit_is_set(ADCSRA, ADSC));	// wait until conversion done;

	low  = ADCL;
	high = ADCH;
	value = (high << 8) | low;
	levelSensorHL_d = value;

	if(value<reference)
		levelSensorHL = 1;
	else
		levelSensorHL = 0;
}

void periodVerify0()
{
	if (((tm.Hour == HourOn) && (tm.Minute >= MinOn)) || (tm.Hour > HourOn)
			|| (tm.Hour < HourOff)
			|| ((tm.Hour == HourOff) && (tm.Minute < MinOff)))
	{
		periodo = greenTime;
		flag_02 = 1;
		flag_03 = 0;
	}

	if (((tm.Hour == HourOff) && (tm.Minute >= MinOff))
			|| ((tm.Hour > HourOff) && (tm.Hour < HourOn))
			|| ((tm.Hour == HourOn) && (tm.Minute < MinOn)))
	{
		periodo = redTime;
		flag_02 = 0;
		flag_03 = 1;
	}
}
void periodVerify1()
{
	if (!flag_02)
	{
		if ((tm.Hour == HourOn) && (tm.Minute == MinOn))
		{
			periodo = greenTime;
			flag_02 = 1;
			flag_03 = 0;
		}
	}

	if (!flag_03)
	{
		if ((tm.Hour == HourOff) && (tm.Minute == MinOff))
		{
			periodo = redTime;
			flag_02 = 0;
			flag_03 = 1;
		}
	}
}
void motorDecision()
{
	switch (periodo)
	{
	case redTime:

		if (!flag_01)
		{
			motor_stop();
			flag_01 = 1;

			flag_MLL = 0;
			flag_LL = 0;
		}

		break;

	case greenTime:

		if(levelSensorMLL)
		{
			if(!flag_MLL)
			{
				flag_MLL = 1;
				flag_LL = 0;
				flag_01 = 0;
				motor_start();
			}
		}

		if(!levelSensorLL)
		{
			if(!flag_LL)
			{
				flag_LL = 1;
				flag_MLL = 0;
				flag_01 = 0;
				motor_stop();
			}
		}


//		if (distance)
//		{
//			// A distância do do sensor até a parte de cima da valvula é de 253cm.
//			if (distance <= distMin)
//			{
//				if (!flag01)
//				{
//					motor_start();
//				}
//			}
//
//			if (distance >= distMax)
//			{
//				if (!flag02)
//				{
//					motor_stop();
//					flag01 = 0;
//					flag02 = 1;
//
//					flag03 = 0;
//				}
//			}
//		}
		break;

	default:
		break;
	}
}

void summary_Print(uint8_t opt)
{
	switch (opt)
	{
		case 0:
			sprintf(buffer,"Time: %.2d:%.2d:%.2d, %.2d/%.2d/%d",tm.Hour, tm.Minute, tm.Second, tm.Day, tm.Month, tmYearToCalendar(tm.Year));
			Serial.print(buffer);

			sprintf(buffer," Uptime: %.2d:%.2d:%.2d, %d day(s), %d month(s), %d year(s)", hour(), minute(), second(), day()-1, month()-1, year()-1970);
			Serial.println(buffer);

			sprintf(buffer," Per.: %d, Motor: %d",periodo, motorStatus);
			Serial.println(buffer);
			break;

		case 1:
			int i;
			if(motorStatus)
			{
				for(i=(nLog-1);i>=0;i--)
				{
					sprintf(buffer,"Desligou_%.2d: %.2d:%.2d:%.2d, %.2d/%.2d/%d D= %.3d cm",(i+1),hourLog_OFF[i], minuteLog_OFF[i], secondLog_OFF[i], dayLog_OFF[i], monthLog_OFF[i], tmYearToCalendar(YearLog_OFF[i]), distanceLog_OFF[i]);
					Serial.println(buffer);

					sprintf(buffer,"Ligou_%.2d: %.2d:%.2d:%.2d, %.2d/%.2d/%d D= %.3d cm",(i+1),hourLog_ON[i], minuteLog_ON[i], secondLog_ON[i], dayLog_ON[i], monthLog_ON[i], tmYearToCalendar(YearLog_ON[i]), distanceLog_ON[i]);
					Serial.println(buffer);
				}
			}
			else
			{
				for(i=(nLog-1);i>=0;i--) //	for(i=0;i<nLog;i++)
				{
					sprintf(buffer,"Ligou_%.2d: %.2d:%.2d:%.2d, %.2d/%.2d/%d D= %.3d cm",(i+1),hourLog_ON[i], minuteLog_ON[i], secondLog_ON[i], dayLog_ON[i], monthLog_ON[i], tmYearToCalendar(YearLog_ON[i]), distanceLog_ON[i]);
					Serial.println(buffer);

					sprintf(buffer,"Desligou_%.2d: %.2d:%.2d:%.2d, %.2d/%.2d/%d D= %.3d cm",(i+1),hourLog_OFF[i], minuteLog_OFF[i], secondLog_OFF[i], dayLog_OFF[i], monthLog_OFF[i], tmYearToCalendar(YearLog_OFF[i]), distanceLog_OFF[i]);
					Serial.println(buffer);
				}
			}
			break;

		case 3:
			sprintf(buffer,"Motor: %d, Time: %.2d:%.2d:%.2d",motorStatus, tm.Hour, tm.Minute, tm.Second);
			Serial.println(buffer);

			break;

		case 4:
			sprintf(buffer,"LL:%d, MLL:%d, HL:%d ",levelSensorLL, levelSensorMLL, levelSensorHL);
			Serial.println(buffer);

			sprintf(buffer,"LL:%d, MLL:%d, HL:%d",levelSensorLL_d, levelSensorMLL_d, levelSensorHL_d);
			Serial.println(buffer);

			break;

		case 9:
			sprintf(buffer,"Error!");
			Serial.println(buffer);
			break;

		default:

			sprintf(buffer,"Comando nao implementado!");
			Serial.println(buffer);
			break;
	}
}

void refreshVariables()
{
	if (flag_1s)
	{
		if(flag_sendContinuously)
		{
			summary_Print(4);
		}

		flag_1s = 0;

		sensorRead_Level();

		if(flag_reset)
		{
			wdt_enable(WDTO_15MS);
			_delay_ms(100);
		}

//		distance = levelCheck();
//		if(flag_sendContinuously == 1)
//		{
//			Serial.println(distance);
//		}
		RTC.read(tm);
		periodVerify0();
	}
}

void handleMessage()
{
/*	0;				Verificar detalhes - Detalhes simples (tempo).
		00;				- Detalhes simples (tempo).


		09;				- Reseta o sistema.

	1123030;		Ajustar a hora para 12:30:30
	201042014;		ajustar a data para 01 de abril de 2014
	31;				ligar (31) ou desligar (30) o motor
*/

	// Tx - Transmitter
	if(enableDecode)
	{
		enableDecode = 0;

//		int i;
//		for(i=0;i<rLength;i++)
//		{
//			Serial1.println(sInstr[i]);
//		}
//		for(i=0;i<rLength;i++)
//		{
//			Serial1.println(sInstr[i],HEX);
//		}

		// Getting the opcode
		aux[0] = '0';
		aux[1] = sInstr[0];
		aux[2] = '\0';
		opcode = (uint8_t) atoi(aux);
//		Serial1.println("Got!");

		switch (opcode)
		{
			case 0:		// Check status
			{
				aux[0] = '0';
				aux[1] = sInstr[1];
				aux[2] = '\0';
				uint8_t statusCommand = 0;
				statusCommand = (uint8_t) atoi(aux);

				if(sInstr[2] == ';')
				{
					switch (statusCommand)
					{
						case 7:
							flag_sendContinuously = 1;
							break;

						case 8:
							flag_sendContinuously = 0;
							break;

						case 9:
							flag_reset = 1;
							break;

						default:
							summary_Print(statusCommand);
							break;
					}
				}
			}
				break;

//				if(sInstr[1] == ';')
//				{
//					summary_Print(0);
//				}
//				else
//				{
//					aux[0] = '0';
//					aux[1] = sInstr[1];
//					aux[2] = '\0';
//					uint8_t statusCommand = (uint8_t) atoi(aux);
//
//					summary_Print(statusCommand);
//				}
//
//				break;

// -----------------------------------------------------------------
			case 1:		// Set-up clock
			{
				// Getting the parameters
				aux[0] = sInstr[1];
				aux[1] = sInstr[2];
				aux[2] = '\0';
				tm.Hour = (uint8_t) atoi(aux);

				aux[0] = sInstr[3];
				aux[1] = sInstr[4];
				aux[2] = '\0';
				tm.Minute = (uint8_t) atoi(aux);

				aux[0] = sInstr[5];
				aux[1] = sInstr[6];
				aux[2] = '\0';
				tm.Second = (uint8_t) atoi(aux);

				RTC.write(tm);

				summary_Print(0);
			}
				break;

// -----------------------------------------------------------------
			case 2:		// Set-up date

				// Getting the parameters
				aux[0] = sInstr[1];
				aux[1] = sInstr[2];
				aux[2] = '\0';
				tm.Day = (uint8_t) atoi(aux);

				aux[0] = sInstr[3];
				aux[1] = sInstr[4];
				aux[2] = '\0';
				tm.Month = (uint8_t) atoi(aux);

				char aux2[5];
				aux2[0] = sInstr[5];
				aux2[1] = sInstr[6];
				aux2[2] = sInstr[7];
				aux2[3] = sInstr[8];
				aux2[4] = '\0';
				tm.Year = (uint8_t) (atoi(aux2)-1970);

				RTC.write(tm);

				summary_Print(0);

				break;

// -----------------------------------------------------------------
			case 3:		// Set motor ON/OFF

				uint8_t motorCommand;

				aux[0] = '0';
				aux[1] = sInstr[1];
				aux[2] = '\0';
				motorCommand = (uint8_t) atoi(aux);

				if (motorCommand&&(!motorStatus))
					motor_start();
				else
					motor_stop();

				summary_Print(3);

				break;

// -----------------------------------------------------------------
			default:
				summary_Print(10);
				break;
		}
		memset(sInstr,0,sizeof(sInstr));
	}
}

void handleMessage_OLD()
{
	// Packet Processing
	if (enableDecode)
	{
		opcode = pkt_Rx[0];
		enableDecode = 0;
		switch (opcode)
		{
			case 0:
				pkt_Tx[0] = 0;
				pkt_Tx[1] = tm.Hour;
				pkt_Tx[2] = tm.Minute;
				pkt_Tx[3] = tm.Second;
				pkt_Tx[4] = distance;
				pkt_Tx[5] = periodo;
		//			pkt_Tx[5] = motorStatus;
		//			pkt_Tx[6] = HourOn;
		//			pkt_Tx[7] = MinOn;
		//			pkt_Tx[8] = HourOff;
		//			pkt_Tx[9] = MinOff;
				enableSend = 1;

				break;

			case 1:
				setTime((int) pkt_Rx[1], (int) pkt_Rx[2], (int) pkt_Rx[3], 1, 1,
						2014);
				break;

			case 2:
				HourOn = pkt_Rx[1];
				MinOn = pkt_Rx[2];
				HourOff = pkt_Rx[3];
				MinOff = pkt_Rx[4];

				break;

			case 3:
				if (pkt_Rx[1])
					motor_start();
				else
					motor_stop();
				break;

			case 4:
				if(flag_sendContinuously)
				{

				}
				break;


			default:
				break;
		}
	}
}

void comm_Bluetooth_OLD()
{
	// Rx - Always listening
	while((Serial.available()>0))	// Reading from serial
	{
		inChar = Serial.read();
		sInstr[k] = inChar;
		k++;

		if(inChar==';')
		{
			rLength = k;
			k = 0;

			enableTranslate = 1;
		}
	}

	// Tx - Transmitter
	if(enableTranslate)
	{
		enableTranslate = 0;

		// Getting the opcode
		aux[0] = '0';
		aux[1] = sInstr[0];
		aux[2] = '\0';
		opcode = (uint8_t) atoi(aux);
		Serial.println("Got!");

		switch (opcode)
		{
			case 0:		// Check status

				int i;
				if(motorStatus)
				{
					for(i=(nLog-1);i>=0;i--)
					{
						sprintf(buffer,"Desligou_%.2d: %.2d:%.2d:%.2d, %.2d/%.2d/%d D= %.3d cm",(i+1),hourLog_OFF[i], minuteLog_OFF[i], secondLog_OFF[i], dayLog_OFF[i], monthLog_OFF[i], tmYearToCalendar(YearLog_OFF[i]), distanceLog_OFF[i]);
						Serial.println(buffer);

						sprintf(buffer,"Ligou_%.2d: %.2d:%.2d:%.2d, %.2d/%.2d/%d D= %.3d cm",(i+1),hourLog_ON[i], minuteLog_ON[i], secondLog_ON[i], dayLog_ON[i], monthLog_ON[i], tmYearToCalendar(YearLog_ON[i]), distanceLog_ON[i]);
						Serial.println(buffer);
					}
				}
				else
				{
					for(i=(nLog-1);i>=0;i--) //	for(i=0;i<nLog;i++)
					{
						sprintf(buffer,"Ligou_%.2d: %.2d:%.2d:%.2d, %.2d/%.2d/%d D= %.3d cm",(i+1),hourLog_ON[i], minuteLog_ON[i], secondLog_ON[i], dayLog_ON[i], monthLog_ON[i], tmYearToCalendar(YearLog_ON[i]), distanceLog_ON[i]);
						Serial.println(buffer);

						sprintf(buffer,"Desligou_%.2d: %.2d:%.2d:%.2d, %.2d/%.2d/%d D= %.3d cm",(i+1),hourLog_OFF[i], minuteLog_OFF[i], secondLog_OFF[i], dayLog_OFF[i], monthLog_OFF[i], tmYearToCalendar(YearLog_OFF[i]), distanceLog_OFF[i]);
						Serial.println(buffer);
					}
				}

				sprintf(buffer,"Uptime: %.2d:%.2d:%.2d, %d day(s), %d month(s), %d year(s)", hour(), minute(), second(), day()-1, month()-1, year()-1970);
				Serial.println(buffer);

				sprintf(buffer,"Time: %.2d:%.2d:%.2d, %.2d/%.2d/%d,  D= %.3d cm",tm.Hour, tm.Minute, tm.Second, tm.Day, tm.Month, tmYearToCalendar(tm.Year), distance);
				Serial.print(buffer);

				sprintf(buffer," Per.: %d, Motor: %d",periodo, motorStatus);
				Serial.println(buffer);


//				sprintf(buffer,"Ligou_01___: %.2d:%.2d:%.2d, %.2d/%.2d/%d D= %.3d cm",hourLog_ON[0], minuteLog_ON[0], secondLog_ON[0], dayLog_ON[0], monthLog_ON[0], tmYearToCalendar(YearLog_ON[0]), distanceLog_ON[0]);
//				Serial.println(buffer);
//
//				sprintf(buffer,"Ligou_02___: %.2d:%.2d:%.2d, %.2d/%.2d/%d D= %.3d cm",hourLog_ON[1], minuteLog_ON[1], secondLog_ON[1], dayLog_ON[1], monthLog_ON[1], tmYearToCalendar(YearLog_ON[1]), distanceLog_ON[1]);
//				Serial.println(buffer);
//
//				sprintf(buffer,"Desligou_01: %.2d:%.2d:%.2d, %.2d/%.2d/%d D= %.3d cm",hourLog_OFF[0], minuteLog_OFF[0], secondLog_OFF[0], dayLog_OFF[0], monthLog_OFF[0], tmYearToCalendar(YearLog_OFF[0]), distanceLog_OFF[0]);
//				Serial.println(buffer);
//
//				sprintf(buffer,"Desligou_02: %.2d:%.2d:%.2d, %.2d/%.2d/%d D= %.3d cm",hourLog_OFF[1], minuteLog_OFF[1], secondLog_OFF[1], dayLog_OFF[1], monthLog_OFF[1], tmYearToCalendar(YearLog_OFF[1]), distanceLog_OFF[1]);
//				Serial.println(buffer);

				break;

			case 1:		// Set-up time

				// Getting the parameters
				aux[0] = sInstr[1];
				aux[1] = sInstr[2];
				aux[2] = '\0';
				tm.Hour = (uint8_t) atoi(aux);

				aux[0] = sInstr[3];
				aux[1] = sInstr[4];
				aux[2] = '\0';
				tm.Minute = (uint8_t) atoi(aux);

				aux[0] = sInstr[5];
				aux[1] = sInstr[6];
				aux[2] = '\0';
				tm.Second = (uint8_t) atoi(aux);

				RTC.write(tm);

				break;

			case 2:		// Set-up start and stop time

				// Getting the parameters
				aux[0] = sInstr[1];
				aux[1] = sInstr[2];
				aux[2] = '\0';
				tm.Day = (uint8_t) atoi(aux);

				aux[0] = sInstr[3];
				aux[1] = sInstr[4];
				aux[2] = '\0';
				tm.Month = (uint8_t) atoi(aux);

				char aux2[5];
				aux2[0] = sInstr[5];
				aux2[1] = sInstr[6];
				aux2[2] = sInstr[7];
				aux2[3] = sInstr[8];
				aux2[4] = '\0';
				tm.Year = (uint8_t) (atoi(aux2)-1970);

				RTC.write(tm);

				break;

			case 3:		// Set motor ON/OFF

				uint8_t motorCommand;
				aux[0] = '0';
				aux[1] = sInstr[1];
				aux[2] = '\0';
				motorCommand = (uint8_t) atoi(aux);

				if (motorCommand)
					motor_start();
				else
					motor_stop();

				break;

			case 4:		// Set motor ON/OFF

				uint8_t sendContinuously;
				aux[0] = '0';
				aux[1] = sInstr[1];
				aux[2] = '\0';
				sendContinuously = (uint8_t) atoi(aux);

//				sprintf(buffer,"Ligou_%.2d: %.2d:%.2d:%.2d, %.2d/%.2d/%d D= %.3d cm",(i+1),hourLog_ON[i], minuteLog_ON[i], secondLog_ON[i], dayLog_ON[i], monthLog_ON[i], tmYearToCalendar(YearLog_ON[i]), distanceLog_ON[i]);
				Serial.println("Entrou");

				if(sendContinuously)
				{
					flag_sendContinuously = 1;
					Serial.println("Entrou = 1");
				}
				else
				{
					flag_sendContinuously = 0;
					Serial.println("Entrou = 0");
				}

				break;

			default:
				break;

		}
	}
}
void comm_Bluetooth()
{
	// Rx - Always listening
//	uint8_t j2 =0;
	while((Serial.available()>0))	// Reading from serial
	{
		inChar = Serial.read();

		if(inChar=='$')
		{
			j2 = 0;
			flag_frameStartBT = 1;
//			Serial.println("Frame Start!");
		}

		if(flag_frameStartBT)
			sInstrBluetooth[j2] = inChar;

//		sprintf(buffer,"J= %d",j2);
//		Serial.println(buffer);

		j2++;

		if(j2>=sizeof(sInstrBluetooth))
		{
			memset(sInstrBluetooth,0,sizeof(sInstrBluetooth));
			j2=0;
			Serial.println("ZEROU! sIntr BLuetooth Buffer!");
		}

		if(inChar==';')
		{
//			Serial.println("Encontrou ; !");
			if(flag_frameStartBT)
			{
//				Serial.println("Frame Stop!");
				flag_frameStartBT = 0;
				rLength = j2;
				j2 = 0;
				enableTranslate_Bluetooth = 1;
			}
		}
	}
//	flag_frameStart = 0;

	if(enableTranslate_Bluetooth)
	{
//		Serial.println("enableTranslate_Bluetooth");
		enableTranslate_Bluetooth = 0;

		char *pi0, *pf0;
		pi0 = strchr(sInstrBluetooth,'$');
		pf0 = strchr(sInstrBluetooth,';');

		if(pi0!=NULL)
		{
			uint8_t l0=0;
			l0 = pf0 - pi0;

			int i;
			for(i=1;i<=l0;i++)
			{
				sInstr[i-1] = pi0[i];
//				Serial.write(sInstr[i-1]);
			}
			memset(sInstrBluetooth,0,sizeof(sInstrBluetooth));
	//		Serial.println(sInstr);

			enableDecode = 1;
		}
		else
		{
			Serial.println("Error!!");
			Serial.write(pi0[0]);
			Serial.write(pf0[0]);
		}
	}
}

ISR(TIMER1_COMPA_vect)
{
	flag_1s = 1;
}

int main()
{
	// Initialize arduino hardware requirements.
	init();
	init_IO();
	init_Timer1_1Hz();
	init_ADC();
	init_WDT();

	Serial.begin(38400);
	Serial.println("- Acionna Water Bomb -");

	while (1)
	{
		// Refrash all variables to compare and take decisions;
		wdt_reset();
		refreshVariables();

		// Compare time of day and machine status;
		wdt_reset();
		motorDecision();

		// Bluetooth communication
		wdt_reset();
		comm_Bluetooth();

		// Message Processing
		wdt_reset();
		handleMessage();
	}
}
