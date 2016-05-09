#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>

#include <Arduino.h>

#include "RTC/Time.h"
#include "RTC/DS1307RTC.h"

//const char *monthName[12] = {
//	"Jan", "Feb", "Mar", "Apr", "May", "Jun",
//	"Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
//};

tmElements_t tm;
uint8_t opcode;

//#define triac_B_on()		PORTD |=  (1<<6);	// Triac 1 is the starter
//#define triac_B_off()		PORTD &= ~(1<<6);
#define triac_A_on()		PORTB |=  (1<<0);	// Triac 2 is aux
#define triac_A_off()		PORTB &= ~(1<<0);

#define driveMotor_ON()		triac_A_on();
#define driveMotor_OFF()	triac_A_off();

#define readPin_Rth			(~PIND & 0b10000000)
#define readPin_k1			bit_is_set(PINB, 0)
//#define read_buttonSignal	bit_is_clear(PIND, 3)

enum states01 {
	redTime,
	greenTime
};
enum states01 periodo = redTime;

// Addresses
const uint8_t addr_stateMode 	= 1;			// 1 bytes
const uint8_t addr_LevelRef 	= 2;			// 4 bytes
const uint8_t addr_standBy_min 	= 5;			// 2 bytes
const uint8_t addr_motorTimerE 	= 7;			// 1 byte
const uint8_t addr_PRessureRef 	= 8;			// 1 byte
const uint8_t addr_HourOnTM 	= 30;			// 9 bytes
const uint8_t addr_MinOnTM 		= 39;			// nTM byte(s)
const uint8_t addr_nTM 			= 48;			// 1 byte

// Wake up interrupts
uint8_t flag_WDRF = 0;			// Watchdog System Reset Flag
uint8_t flag_BORF = 0;			// Brown-out Reset Flag
uint8_t flag_EXTRF = 0;			// External Reset Flag
uint8_t flag_PORF = 0;			// Power-on Reset Flag

uint8_t flag_waitPowerOn = 0;	// Minutes before start motor after power line ocasionally down
uint8_t powerOn_min_Standy = 0;
uint8_t powerOn_min = 0;
uint8_t powerOn_sec = 0;

uint8_t powerOff_min = 0;
uint8_t powerOff_sec = 0;

uint8_t motorTimerE = 0;
uint8_t PRessureRef = 0;

int PRess;
int Pd = 0;

uint8_t flag_AwaysON = 0;
uint8_t flag_timeMatch = 0;
uint8_t flag_01 = 0;
uint8_t flag_02 = 0;
uint8_t flag_03 = 0;
uint8_t motorStatus = 0;
uint8_t flag_Th = 0;

uint16_t motor_timerON = 0;
uint16_t motor_timerOFF = 0;

uint8_t HourOn  = 21;
uint8_t MinOn   = 30;
uint8_t HourOff = 6;
uint8_t MinOff  = 0;

uint8_t nTM;
uint8_t HourOnTM[9];
uint8_t MinOnTM[9];

volatile uint8_t flag_1s = 1;

//const uint8_t pkt_size = 10;
//uint8_t pkt_Tx[pkt_size], pkt_Rx[pkt_size];
uint8_t k, rLength, j;
char aux[3], aux2[5], buffer[70], inChar, sInstr[20];
char sInstrBluetooth[30];

uint16_t levelRef_10bit = 0;

uint8_t stateMode = 0;
uint8_t enableSend = 0;
uint8_t enableDecode = 0;
uint8_t enableTranslate = 0;
uint8_t flagSync = 0;
uint8_t countSync = 0;
uint8_t flag_debug = 0;

uint8_t j2 = 0;
uint8_t flag_frameStartBT = 0;
uint8_t enableTranslate_Bluetooth = 0;

// Logs
const int nLog = 20;
uint8_t hourLog_ON[nLog], minuteLog_ON[nLog];
uint8_t hourLog_OFF[nLog], minuteLog_OFF[nLog];
uint8_t dayLog_ON[nLog], monthLog_ON[nLog];
uint8_t dayLog_OFF[nLog], monthLog_OFF[nLog];

uint16_t levelSensorLL, levelSensorML, levelSensorHL;
uint16_t levelSensorLL_d, levelSensorML_d, levelSensorHL_d;


uint16_t read_ADC(uint8_t channel)
{
	uint8_t low, high;

	switch(channel)
	{
		case 0:
			ADMUX &= ~(1<<MUX3);
			ADMUX &= ~(1<<MUX2);				// Select ADC0
			ADMUX &= ~(1<<MUX1);
			ADMUX &= ~(1<<MUX0);
			break;

		case 1:
			ADMUX &= ~(1<<MUX3);
			ADMUX &= ~(1<<MUX2);				// Select ADC1
			ADMUX &= ~(1<<MUX1);
			ADMUX |=  (1<<MUX0);
			break;

		case 2:
			ADMUX &= ~(1<<MUX3);
			ADMUX &= ~(1<<MUX2);				// Select ADC2
			ADMUX |=  (1<<MUX1);
			ADMUX &= ~(1<<MUX0);
			break;

		case 3:
			ADMUX &= ~(1<<MUX3);
			ADMUX &= ~(1<<MUX2);				// Select ADC3
			ADMUX |=  (1<<MUX1);
			ADMUX |=  (1<<MUX0);
			break;

		case 4:
			ADMUX &= ~(1<<MUX3);
			ADMUX |=  (1<<MUX2);				// Select ADC4
			ADMUX &= ~(1<<MUX1);
			ADMUX &= ~(1<<MUX0);
			break;

		case 5:
			ADMUX &= ~(1<<MUX3);
			ADMUX |=  (1<<MUX2);				// Select ADC5
			ADMUX &= ~(1<<MUX1);
			ADMUX |=  (1<<MUX0);
			break;

		case 6:
			ADMUX &= ~(1<<MUX3);
			ADMUX |=  (1<<MUX2);				// Select ADC5
			ADMUX |=  (1<<MUX1);
			ADMUX &= ~(1<<MUX0);
			break;

		case 7:
			ADMUX &= ~(1<<MUX3);
			ADMUX |=  (1<<MUX2);				// Select ADC5
			ADMUX |=  (1<<MUX1);
			ADMUX |=  (1<<MUX0);
			break;

		default:
			ADMUX &= ~(1<<MUX3);
			ADMUX &= ~(1<<MUX2);				// Select ADC0
			ADMUX &= ~(1<<MUX1);
			ADMUX &= ~(1<<MUX0);
			break;
	}


	ADCSRA |= (1<<ADSC);				// Start conversion;
	while (bit_is_set(ADCSRA, ADSC));	// wait until conversion done;

	//		Serial.println((ADCH << 8) | ADCL);
	low  = ADCL;
	high = ADCH;

	return ((high << 8) | low);
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

	wdt_enable(WDTO_4S);
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
//	ADMUX &=  ~(1<<REFS1);		// AREF, Internal Vref turned off
//	ADMUX &=  ~(1<<REFS0);

	ADMUX &=  ~(1<<REFS1);		// AVCC with external capacitor at AREF pin
	ADMUX |=   (1<<REFS0);

//	ADMUX |=   (1<<REFS1);		// Internal 1.1V Voltage Reference with external capacitor at AREF pin
//	ADMUX |=   (1<<REFS0);

	ADMUX &= ~(1<<ADLAR);		// Right adjustment. To ADCL register.

//	ADMUX &= ~(1<<MUX3);		// Select ADC0
//	ADMUX &= ~(1<<MUX2);
//	ADMUX &= ~(1<<MUX1);
//	ADMUX &= ~(1<<MUX0);

	DDRC &= ~(1<<PC0);
	DDRC &= ~(1<<PC1);
	DDRC &= ~(1<<PC2);
	DDRC &= ~(1<<PC3);
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


	DDRD &= ~(1 << 7); // Thermal swith as input!

	// Set triac1, triac2 and led connected pins as output
//	DDRD |= (1 << 5); // Led!
	DDRB |= (1 << 0); // Triac A (Motor)
	DDRD |= (1 << 6); // Triac B (Motor)
}
void motor_start()
{
	if(flag_waitPowerOn)
	{
		int i;
		for(i=(nLog-1);i>0;i--)
		{
			hourLog_ON[i] = hourLog_ON[i-1];
			minuteLog_ON[i] = minuteLog_ON[i-1];
	//		secondLog_ON[i] = secondLog_ON[i-1];
			dayLog_ON[i] = dayLog_ON[i-1];
			monthLog_ON[i] = monthLog_ON[i-1];
	//		YearLog_ON[i] = YearLog_ON[i-1];
		}

		hourLog_ON[0] = tm.Hour;
		minuteLog_ON[0] = tm.Minute;
	//	secondLog_ON[0] = tm.Second;
		dayLog_ON[0] = tm.Day;
		monthLog_ON[0] = tm.Month;
	//	YearLog_ON[0] = tm.Year;

	//	flag_LL = 0;
	//	flag_ML = 1;
		motor_timerON = 0;

		driveMotor_ON();

		motorStatus = readPin_k1;
	}
}
void motor_stop()
{
	int i;
	for(i=(nLog-1);i>0;i--)
	{
		hourLog_OFF[i] = hourLog_OFF[i-1];
		minuteLog_OFF[i] = minuteLog_OFF[i-1];
//		secondLog_OFF[i] = secondLog_OFF[i-1];
		dayLog_OFF[i] = dayLog_OFF[i-1];
		monthLog_OFF[i] = monthLog_OFF[i-1];
//		YearLog_OFF[i] = YearLog_OFF[i-1];
	}

	hourLog_OFF[0] = tm.Hour;
	minuteLog_OFF[0] = tm.Minute;
//	secondLog_OFF[0] = tm.Second;
	dayLog_OFF[0] = tm.Day;
	monthLog_OFF[0] = tm.Month;
//	YearLog_OFF[0] = tm.Year;

//	flag_LL = 1;
//	flag_ML = 0;

	motor_timerOFF = 0;

	flag_waitPowerOn = 0;
	powerOn_min = powerOn_min_Standy;

	driveMotor_OFF();

	motorStatus = readPin_k1;
}
double get_Pressure()
{
	/*
	Sensor details

    Thread size : G 1/4" (BSP)
    Sensor material:  Carbon steel alloy
    Working voltage: 5 VDC
    Output voltage: 0.5 to 4.5 VDC
    Working Current: <= 10 mA
    Working pressure range: 0 to  1.2 MPa
    Maxi pressure: 2.4 MPa
    Working temperature range: 0 to 100 graus C
    Accuracy: ± 1.0%
    Response time: <= 2.0 ms
    Package include: 1 pc pressure sensor
    Wires : Red---Power (+5V)  Black---Power (0V) - blue ---Pulse singal output


    4.5 V___	   922___	1.2 MPa___	 12 Bar___	 120 m.c.a.___
	  	  |				|			|			|				|
	 	  |				|			|			|				|
	 	  |				|			|			|				|
	  out_|			Pd__|		  __|			|			Pa__|
	 	  |				|			|			|				|
	 	  |				|			|			|				|
	 	  |				|			|			|				|
		 _|_		   _|_		   _|_		   _|_			   _|_
	0.5 V			103			0 MPa		0 Bar		0 m.c.a.

	(out-0.5)/(4.5-0.5) = 1024

	(out-0.0)/(5-0) = (x-0)/(1024-0)

	(Pd - 103)/(922-103) = (Pa - 0)/(120 - 0)
	Pa = 120.0*Pd/(1024.0);

	(xs - 0) = temp - (0)
	(255 - 0)  +50 - (0)

	Direct Conversion
	xs = 255*(temp+0)/51
	tempNow_XS = (uint8_t) 255.0*(tempNow+0.0)/51.0;

	Inverse Conversion
	temp = (TempMax*xs/255) - TempMin
	tempNow = (uint8_t) ((sTempMax*tempNow_XS)/255.0 - sTempMin);
    */
//	const double PRessMax = 68.9475729;	// Sensor max pressure [m.c.a.];
//	const double PRessMax = 120.658253;
	const double PRessMax = 120.0;

	Pd = read_ADC(7);
	return (PRessMax)*(Pd-102.4)/(921.6-102.4);
//	(Pd - 103)/(922-103) = (Pa - 0)/(120 - 0);
}
void check_levelSensors()
{
	// Select ADC0 - LL sensor
	levelSensorLL_d = read_ADC(0);

	if(levelSensorLL_d < levelRef_10bit)
		levelSensorLL = 1;
	else
		levelSensorLL = 0;


	// Select ADC1 - ML sensor
	levelSensorML_d = read_ADC(1);

	if(levelSensorML_d < levelRef_10bit)
		levelSensorML = 1;
	else
		levelSensorML = 0;


	// Select ADC2 - HL sensor
	levelSensorHL_d = read_ADC(2);

	if(levelSensorHL_d < levelRef_10bit)
		levelSensorHL = 1;
	else
		levelSensorHL = 0;
}
void check_period()
{
	// Season time verify
	if(((tm.Hour == HourOn) && (tm.Minute == MinOn)) || (tm.Hour > HourOn) || (tm.Hour < HourOff) || ((tm.Hour == HourOff) && (tm.Minute < MinOff)))
	{
		periodo = greenTime;

		if(flag_01)
		{
			flag_01 = 0;
//			flag_timeMatch = 1;
		}
	}

	if (((tm.Hour == HourOff) && (tm.Minute >= MinOff))	|| ((tm.Hour > HourOff) && (tm.Hour < HourOn))	|| ((tm.Hour == HourOn) && (tm.Minute < MinOn)))
	{
		periodo = redTime;

//		flag_timeMatch = 0;
		flag_01 = 1;
	}
}
void check_timeMatch()
{
	uint8_t i, nTM_var=0;

	// matching time verify
	if(!motorStatus)
	{
		if(stateMode)
		{
			switch (stateMode)
			{
			case 1:
				nTM_var = 1;
				break;

			case 2:
				nTM_var = nTM;
				break;
			}

			for(i=0;i<nTM_var;i++)
			{
				if((tm.Hour == HourOnTM[i]) && (tm.Minute == MinOnTM[i]))
				{
					flag_timeMatch = 1;
				}
			}
		}
	}

}
void check_thermalSafe()
{
	if(motorStatus)
	{
		if(readPin_Rth)
		{
			uint16_t countThermal = 50000;
			Serial.println("Th0");
			while(readPin_Rth && countThermal)
			{
				countThermal--;
			}
			Serial.println("Th1");
			if(!countThermal)
			{
				flag_Th = 1;

				motor_stop();
				stateMode = 0;
				eeprom_write_byte(( uint8_t *)(addr_stateMode), stateMode);
			}
			else
				flag_Th = 0;

		}
		else
		{
			flag_Th = 0;
		}
	}
}
void check_gpio()
{
//	motorStatus =
	motorStatus = readPin_k1;
}
void motorControl_byVariables()
{
	if(flag_timeMatch)
	{
		flag_timeMatch = 0;

		if(!motorStatus)
		{
			motor_start();
		}
	}

	if(stateMode == 1)
	{
		if(levelSensorHL)
		{
			if(!motorStatus)
			{
				motor_start();
			}
		}
	}

	if((PRess >= PRessureRef) || (!levelSensorLL))
	{
		if(motorStatus)
		{
			motor_stop();
		}
	}

	if(motorTimerE)
	{
		if(motorStatus)
		{
			if(motor_timerON/60 >= motorTimerE)
			{
				motor_stop();
			}
		}
	}
}
void motorPeriodDecision(uint16_t highSensor)
{
	switch (periodo)
	{
	case redTime:
		if(motorStatus)
		{
			motor_stop();
		}
		break;

	case greenTime:
		motorControl_byVariables();
		break;
	}
}
void process_Mode()
{
	switch (stateMode)
	{
		case 0:	// System Down!
			if((!levelSensorLL) || (PRess >= PRessureRef))
			{
				if(motorStatus)
				{
					motor_stop();

					flag_waitPowerOn = 0;
					powerOn_min = powerOn_min_Standy;
				}
			}
			break;

		case 1:	// Night Working;
			motorPeriodDecision(levelSensorHL);
			break;

		case 2:
			motorControl_byVariables();
			break;

		default:
			stateMode = 0;
			Serial.println("Standby");
			break;
	}
}
void summary_Print(uint8_t opt)
{
	switch (opt)
	{
		case 0:
			sprintf(buffer,"Time:%.2d:%.2d:%.2d %.2d/%.2d/%d",tm.Hour, tm.Minute, tm.Second, tm.Day, tm.Month, tmYearToCalendar(tm.Year));
			Serial.print(buffer);

			sprintf(buffer," UP:%.2d:%.2d:%.2d, d:%d m:%d", hour(), minute(), second(), day()-1, month()-1);
			Serial.println(buffer);

			sprintf(buffer," P:%d k1:%d",periodo, motorStatus);
			Serial.println(buffer);

			switch (stateMode)
			{
				case 0:
					strcpy(buffer," Modo:Desligado");
					break;

				case 1:
					sprintf(buffer," Modo:Liga Noite");
					break;

				case 2:
					if(nTM == 1)
					{
						sprintf(buffer," Modo: Liga %dx as %2.d:%.2d", nTM, HourOnTM[0], MinOnTM[0]);
					}
					else
					{
						sprintf(buffer," Modo: Liga %dx/dia",nTM);
					}
					break;

				default:
					strcpy(buffer,"sMode Err");
					break;
			}
			Serial.println(buffer);

			break;

		case 1:
			int i;
			if(motorStatus)
			{
				for(i=(nLog-1);i>=0;i--)
				{
					sprintf(buffer,"Desligou_%.2d: %.2d:%.2d, %.2d/%.2d  ",(i+1),hourLog_OFF[i], minuteLog_OFF[i], dayLog_OFF[i], monthLog_OFF[i]);
					Serial.println(buffer);

					sprintf(buffer,"Ligou_%.2d: %.2d:%.2d, %.2d/%.2d  ",(i+1),hourLog_ON[i], minuteLog_ON[i], dayLog_ON[i], monthLog_ON[i]);
					Serial.println(buffer);
				}
			}
			else
			{
				for(i=(nLog-1);i>=0;i--) //	for(i=0;i<nLog;i++)
				{
					sprintf(buffer,"Ligou_%.2d: %.2d:%.2d, %.2d/%.2d  ",(i+1),hourLog_ON[i], minuteLog_ON[i], dayLog_ON[i], monthLog_ON[i]);
					Serial.println(buffer);

					sprintf(buffer,"Desligou_%.2d: %.2d:%.2d, %.2d/%.2d  ",(i+1),hourLog_OFF[i], minuteLog_OFF[i], dayLog_OFF[i], monthLog_OFF[i]);
					Serial.println(buffer);
				}
			}
			break;

		case 2:
			sprintf(buffer,"f:%d t1:%.2d:%.2d c:%dmin t2:%d s:%dmin", flag_waitPowerOn, powerOn_min, powerOn_sec, powerOn_min_Standy, motor_timerON/60, motorTimerE);
			Serial.println(buffer);
			break;

		case 3:
			sprintf(buffer,"Motor:%d Fth:%d Rth:%d Pr:%d Pref:%d ", motorStatus, flag_Th, readPin_Rth, PRess, PRessureRef);
			Serial.println(buffer);
			break;

		case 4:
			sprintf(buffer,"LL:%d ML:%d HL:%d ",levelSensorLL, levelSensorML, levelSensorHL);
			Serial.println(buffer);

			sprintf(buffer,"LL:%d ML:%d HL:%d",levelSensorLL_d, levelSensorML_d, levelSensorHL_d);
			Serial.println(buffer);
			break;

		case 5:
			for(i=0;i<nTM;i++)
			{
				sprintf(buffer,"h%d: %.2d:%.2d",i+1, HourOnTM[i], MinOnTM[i]);
				Serial.println(buffer);
			}
			break;

		case 6:
			sprintf(buffer,"timeON:%d ",motor_timerON/60);
			Serial.println(buffer);
			sprintf(buffer,"timeOFF:%d ",motor_timerOFF/60);
			Serial.println(buffer);
			break;

		case 7:
			sprintf(buffer,"P:%d Fth:%d Rth:%d  ", PRess, flag_Th, readPin_Rth);
			Serial.println(buffer);

			sprintf(buffer,"LL:%d ML:%d HL:%d  ",levelSensorLL, levelSensorML, levelSensorHL);
			Serial.println(buffer);

			sprintf(buffer,"LL:%d ML:%d HL:%d  ",levelSensorLL_d, levelSensorML_d, levelSensorHL_d);
			Serial.println(buffer);
			break;
			
		case 8:
			sprintf(buffer,"WDRF:%d BORF:%d EXTRF:%d PORF:%d", flag_WDRF, flag_BORF, flag_EXTRF, flag_PORF);
			Serial.println(buffer);
			break;

		case 9:
			sprintf(buffer,"Err");
			Serial.println(buffer);
			break;

		default:
			sprintf(buffer,"not implemented");
			Serial.println(buffer);
			break;
	}
}
void refreshTimerVar()
{
	if(motorStatus)
	{
		motor_timerON++;
	}
	else
	{
		motor_timerOFF++;
	}

	if(!flag_waitPowerOn)
	{
		if(powerOn_sec == 0)
		{
			if(powerOn_min == 0)
			{
				flag_waitPowerOn = 1;
			}
			else
			{
				powerOn_sec = 59;
				powerOn_min--;
			}
		}
		else
		{
			powerOn_sec--;
		}
	}
}
void refreshVariables()
{
	if (flag_1s)
	{
		flag_1s = 0;

		refreshTimerVar();

		PRess = get_Pressure();			// Get pressure in 1 second interval

		check_gpio();
		check_thermalSafe();
		check_levelSensors();

		RTC.read(tm);
		check_period();					// Period verify
		check_timeMatch();

		if(flag_debug)
		{
			summary_Print(7);
		}
	}
}
void refreshStoredData()
{
	stateMode = eeprom_read_byte((uint8_t *)(addr_stateMode));

	powerOn_min_Standy = eeprom_read_byte((uint8_t *)(addr_standBy_min));
	powerOn_min = powerOn_min_Standy;

	uint8_t lbyte, hbyte;
	hbyte = eeprom_read_byte((uint8_t *)(addr_LevelRef+1));
	lbyte = eeprom_read_byte((uint8_t *)(addr_LevelRef));
	levelRef_10bit = ((hbyte << 8) | lbyte);

	motorTimerE = eeprom_read_byte((uint8_t *)(addr_motorTimerE));
	PRessureRef = eeprom_read_byte((uint8_t *)(addr_PRessureRef));

	nTM = eeprom_read_byte((uint8_t *)(addr_nTM));
	uint8_t i;
	for(i=0;i<9;i++)
	{
		HourOnTM[i] = eeprom_read_byte((uint8_t *)(addr_HourOnTM+i));
		MinOnTM[i] = eeprom_read_byte((uint8_t *)(addr_MinOnTM+i));
	}

}
void handleMessage()
{
/*
$0X;				Verificar detalhes - Detalhes simples (tempo).
	$00;			- Detalhes simples (tempo).
	$01;			- Verifica histórico de quando ligou e desligou;
	$02;			- Mostra tempo que falta para ligar;
		$02:c;		- Zera o tempo;
		$02:c:30;	- Ajusta novo tempo para 30 min;
		$02:s:90;	- Tempo máximo ligado para 90 min. Para não utilizar, colocar zero;
	$03;			- Verifica detalhes do motor, pressão e sensor termico;
	$04;			- Verifica detalhes do nível de água no poço e referência 10 bits;
		$04:0;		- Interrompe o envio continuo das variáveis de pressão e nível;
		$04:1;		- Envia continuamente valores de pressão e nível;
	$04:dddd;		- Referência para os níveis de água;
		$04:0900d;	- Adiciona nova referência para os sensores de nível. Valor de 0 a 1023;
	$05;			- Mostra os horários que liga no modo $62;
	$06;			- Tempo ligado e tempo desligado;
	$07:x;			- ADC reference change.
		$07:0;		- AREF
		$07:1;		- AVCC with external cap at AREF pin
		$07:2;		- Internal 1.1 Voltage reference.
	$08;			- Motivo do reboot.
	$09;			- Reinicia o sistema.

$1HHMMSS;			- Ajusta o horário do sistema;
	$1123040;		- Ajusta a hora para 12:30:40

$2DDMMAAAA;			- Ajusta a data do sistema no formato dia/mês/ano(4 dígitos);
	$201042014;		- Ajusta a data para 01 de abril de 2014;

$3X;				- Acionamento do motor;
	$31;			- liga o motor;
	$30;			- desliga o motor;

$5:n:X; ou $5:hX:HHMM;
	$5:n:9;			- Configura para acionar 9 vezes. Necessário configurar 9 horários;
	$5:n:9;			- Configura o sistema para acionar uma única vez às 21:30 horas;
	$5:h1:2130;		- Configura o primeiro horário como 21:30 horas;
	$5:h8:0437;		- Configura o oitavo horário como 04:37 horas;

$6X;				- Modos de funcionamento;
	$60; 			- Sistema Desligado (nunca ligará);
	$61;			- Liga somente à noite. Sensor superior;
	$62;			- Liga nos determinados horários estipulados;
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
		uint8_t statusCommand = 0;

		switch (opcode)
		{
// -----------------------------------------------------------------
			case 0:		// Check status
			{
				aux[0] = '0';
				aux[1] = sInstr[1];
				aux[2] = '\0';
				statusCommand = (uint8_t) atoi(aux);

				switch (statusCommand)
				{
					// ----------
					// $02:c;  -> clear time counter;
					// $02:c:mm;  -> set time counter ref;
					case 2:
					{
						if(sInstr[2]==':' && sInstr[3]=='c')
						{
							if(sInstr[4] == ';')
							{
								flag_waitPowerOn = 1;
								powerOn_min = 0;
								powerOn_sec = 0;
							}
							else if(sInstr[4] ==':' && sInstr[7] == ';')
							{
								aux[0] = sInstr[5];
								aux[1] = sInstr[6];
								aux[2] = '\0';
								powerOn_min_Standy = (uint8_t) atoi(aux);

								eeprom_write_byte((uint8_t *)(addr_standBy_min), powerOn_min_Standy);

//								Serial.print("powerOn min:");
//								Serial.println(powerOn_min_Standy);
							}
						}
						else if(sInstr[3] == 's' && sInstr[4] == ':' && sInstr[7] == ';')
						{
							aux[0] = sInstr[5];
							aux[1] = sInstr[6];
							aux[2] = '\0';
							motorTimerE = (uint8_t) atoi(aux);

							eeprom_write_byte((uint8_t *)(addr_motorTimerE), motorTimerE);

//							Serial.print("timeE:");
//							Serial.println(motorTimerE);
						}
						summary_Print(statusCommand);
					}
					break;
					// ------------------------------
					case 3:
						if(sInstr[2]==':' && sInstr[3] == 's' && sInstr[4] == ':' && sInstr[7] == ';')
						{
							aux[0] = sInstr[5];
							aux[1] = sInstr[6];
							aux[2] = '\0';
							PRessureRef = (uint8_t) atoi(aux);
							eeprom_write_byte((uint8_t *)(addr_PRessureRef), PRessureRef);

							Serial.print("PRessRef:");
							Serial.println(PRessureRef);
						}
						else
							summary_Print(statusCommand);
						break;
					// ------------------------------
					case 4: // $04:1;
						if(sInstr[2]==':' && sInstr[4]==';')
						{
							aux[0] = '0';
							aux[1] = sInstr[3];
							aux[2] = '\0';
							uint8_t debugCommand;
							debugCommand = (uint8_t) atoi(aux);
							flag_debug = debugCommand;
						}//$04:s:0900;
						else if(sInstr[2]==':' && sInstr[3]=='s' && sInstr[4]==':' && sInstr[9]==';')
						{
							aux2[0] = sInstr[5];
							aux2[1] = sInstr[6];
							aux2[2] = sInstr[7];
							aux2[3] = sInstr[8];
							aux2[4] = '\0';

							uint16_t wordReference = 0;
							wordReference = (uint16_t) atoi(aux2);

							levelRef_10bit = wordReference;

							uint8_t lbyteRef = 0, hbyteRef = 0;
							lbyteRef = wordReference;
							hbyteRef = (wordReference >> 8);

							eeprom_write_byte((uint8_t *)(addr_LevelRef+1), hbyteRef);
							eeprom_write_byte((uint8_t *)(addr_LevelRef), lbyteRef);
						}
						summary_Print(statusCommand);
						Serial.print("Ref: ");
						Serial.println(levelRef_10bit);
						break;
					// ------------------------------
					case 7:
						if(sInstr[2]==':' && sInstr[4]==';')
						{
							aux[0] = '0';
							aux[1] = sInstr[3];
							aux[2] = '\0';
							uint8_t adcCommand = (uint8_t) atoi(aux);

							switch (adcCommand)
							{
								case 0:
									ADMUX &=  ~(1<<REFS1);		// AREF, Internal Vref turned off
									ADMUX &=  ~(1<<REFS0);
									Serial.println("AREF");
									break;

								case 1:
									ADMUX &=  ~(1<<REFS1);		// AVCC with external capacitor at AREF pin
									ADMUX |=   (1<<REFS0);
									Serial.println("AVCC");
									break;

								case 2:
									ADMUX |=   (1<<REFS1);		// Internal 1.1V Voltage Reference with external capacitor at AREF pin
									ADMUX |=   (1<<REFS0);
									Serial.println("1.1V");
									break;
							}
						}
						break;
					// ------------------------------
					case 9:
						Serial.println("Rebooting...");
						wdt_enable(WDTO_15MS);
						_delay_ms(100);
//						flag_reset = 1;
						break;

					default:
						summary_Print(statusCommand);
						break;
				}
			}
			break;
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
			{
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
			}
			break;
// -----------------------------------------------------------------
			case 3:		// Set motor ON/OFF
			{
				uint8_t motorCommand;
				aux[0] = '0';
				aux[1] = sInstr[1];
				aux[2] = '\0';
				motorCommand = (uint8_t) atoi(aux);

				if (motorCommand && (!motorStatus))
					motor_start();
				else
					motor_stop();

				summary_Print(3);
			}
			break;
// -----------------------------------------------------------------

// -----------------------------------------------------------------
			case 5: // Command is $5:h1:2130;
			{
				if(sInstr[1]==':' && sInstr[2]=='h' && sInstr[4]==':' && sInstr[9]==';')
				{
					aux[0] = '0';
					aux[1] = sInstr[3];
					aux[2] = '\0';
					uint8_t indexV = (uint8_t) atoi(aux);

					aux[0] = sInstr[5];
					aux[1] = sInstr[6];
					aux[2] = '\0';
					HourOnTM[indexV-1] = (uint8_t) atoi(aux);
					eeprom_write_byte(( uint8_t *)(addr_HourOnTM+indexV-1), HourOnTM[indexV-1]);

					aux[0] = sInstr[7];
					aux[1] = sInstr[8];
					aux[2] = '\0';
					MinOnTM[indexV-1] = (uint8_t) atoi(aux);
					eeprom_write_byte(( uint8_t *)(addr_MinOnTM+indexV-1), MinOnTM[indexV-1]);

					summary_Print(5);
				}
				else if(sInstr[1]==':' && sInstr[2]=='n' && sInstr[3]==':' && sInstr[5]==';')
				{
					aux[0] = '0';
					aux[1] = sInstr[4];
					aux[2] = '\0';

					nTM = (uint8_t) atoi(aux);
					eeprom_write_byte(( uint8_t *)(addr_nTM), nTM);

					summary_Print(5);
				}
				else if(sInstr[1]==';')
				{
					summary_Print(5);
				}
			}
			break;
// ----------------------------------------------------------------
			case 6:		// Set working mode
			{
				aux[0] = '0';
				aux[1] = sInstr[1];
				aux[2] = '\0';
				stateMode = (uint8_t) atoi(aux);
				eeprom_write_byte(( uint8_t *)(addr_stateMode), stateMode);

				summary_Print(0);
			}
			break;
// -----------------------------------------------------------------
			default:
				summary_Print(10);
				break;
		}
		memset(sInstr,0,sizeof(sInstr));	// Clear all vector;
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
//			Serial.println("ZEROU! sIntr BLuetooth Buffer!");
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
//			Serial.println("Err");
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
	// PowerOFF / Reset verification
	cli();
	flag_WDRF 	= (WDRF & MCUSR);
	flag_BORF 	= (BORF & MCUSR);
	flag_EXTRF 	= (EXTRF & MCUSR);
	flag_PORF 	= (PORF & MCUSR);
	MCUSR = 0x00;
	sei();

	// Initialize arduino hardware requirements.	
	init();
	init_IO();
	init_Timer1_1Hz();
	init_ADC();
	init_WDT();

	Serial.begin(38400);
	Serial.println("-Acionna WP-");

	refreshStoredData();

//	if(flag_WDRF || flag_BORF || flag_EXTRF || flag_PORF)
//	{
//		printf(buffer,"WDRF: %d, BORF: %d, EXTRF: %d, PORF: %d", flag_WDRF, flag_BORF, flag_EXTRF, flag_PORF);
//		printf(buffer,"W:%d B:%d E:%d P:%d ", flag_WDRF, flag_BORF, flag_EXTRF, flag_PORF);
//		Serial.println(buffer);
//	}

	while (1)
	{
		// Refresh all variables to compare and take decisions;
		wdt_reset();
		refreshVariables();

		// Bluetooth communication
		wdt_reset();
		comm_Bluetooth();

		// Message Processing
		wdt_reset();
		handleMessage();

		// Compare time of day and machine status;
		wdt_reset();
		process_Mode();
	}
}







//void init_pwm2()
//{
//	TCCR2A = 0b00000010; // TCCR2A ==>> COM2A1 COM2A0 COM2B1 COM2B0 - - WGM21 WGM20
//	TCCR2B = 0b00000111; // TCCR2B ==>> FOC2A FOC2B - - WGM22 CS22 CS21 CS20
//	TIMSK2 = 0b00000010; // TIMSK2 ==>> - - - - - OCIE2B OCIE2A TOIE2
//	OCR2A = 251;
//
//}
//void stop_pwm2()
//{
////	TCCR2A ==>> COM2A1 COM2A0 COM2B1 COM2B0 - - WGM21 WGM20
//	TCCR2A = 0b00000011;
//
//	// TCCR2B ==> FOC2A FOC2B - - WGM22 CS22 CS21 CS20
//	TCCR2B = 0b00001000;
//	OCR2A = 200;
//
//	// TIMSK2 ==>> - - - - - OCIE2B OCIE2A TOIE2
//	TIMSK2 = 0b00000010;
//
////     Turn Red Led off.
//	PORTD &= ~(1 << PD2);
//}
//void print2digits(int number)
//{
//	if (number >= 0 && number < 10)
//	{
//		Serial.write('0');
//	}
//	Serial.print(number);
//}
//bool getTime(const char *str)
//{
//	int Hour, Min, Sec;
//
//	if (sscanf(str, "%d:%d:%d", &Hour, &Min, &Sec) != 3) return false;
//	tm.Hour = Hour;
//	tm.Minute = Min;
//	tm.Second = Sec;
//	return true;
//}
//bool getDate(const char *str)
//{
//	char Month[12];
//	int Day, Year;
//	uint8_t monthIndex;
//
//	if (sscanf(str, "%s %d %d", Month, &Day, &Year) != 3) return false;
//	for (monthIndex = 0; monthIndex < 12; monthIndex++)
//	{
//		if (strcmp(Month, monthName[monthIndex]) == 0) break;
//	}
//	if (monthIndex >= 12) return false;
//	tm.Day = Day;
//	tm.Month = monthIndex + 1;
//	tm.Year = CalendarYrToTm(Year);
//
//	return true;
//}
//void clockSync_PC()
//{
////	bool parse=false;
////	bool config=false;
////
////	// get the date and time the compiler was run
////	if (getDate(__DATE__) && getTime(__TIME__))
////	{
////		parse = true;
////		// and configure the RTC with this info
////		if (RTC.write(tm))
////		{
////			config = true;
////		}
////	}
////
//}
