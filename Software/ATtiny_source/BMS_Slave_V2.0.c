/**********************************************************************/
// Battery Management System (c) by WWW.SMART-ELECTRO.NET
// This source code is subject to the CreativeCommons Attribution 4.0 International (CC BY 4.0) license
//
// You can copy and redistribute the material in any medium or format.
// Remix, transform, and build upon the material for any purpose, even commercially as long as you
// don't forget to mentioned www.smart-electro.net in your work.
// More about license can be found at: http://http://creativecommons.org/licenses/by/4.0/deed.en
//
// THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
// KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
// PARTICULAR PURPOSE.
//
// Version: V2.0
// Author: Tadej Gortnar <tadej.gortnar@gmail.com>
// Created: Dec 2010
//
// Modified: Feb 2014 Simon.grabar <simon.grabar@gmail.com>
// -Updated header and comments
//
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>

// Voltage settings for different battery types
/*****************************************************/
// Programming voltage for ACD calibration [0.01V]
#define BMS_programm_voltage (332)

// maximum alowable voltage of cell.
#define Li_Max (400)

// optimum full voltage. At this value BMS will start PWM on bypass transistor
#define Li_Full (375) // mV

// minimum cell voltage
#define Li_Min  (280)

/*****************************************************/


#define SET |=
#define CLR &=~
#define TOG ^=

#define CMPINT_EN_MASK    ((1<<OCIE0A) | (0<<OCIE0B)) // INT on Comp-A

//#define SOFTUART_PRESCALE (8)
#define SOFTUART_PRESCALE (1)

#if (SOFTUART_PRESCALE==8)
#define PRESC_MASKA         (0)
#define PRESC_MASKB         (1<<CS01)
#elif (SOFTUART_PRESCALE==1)
#define PRESC_MASKA         (0)
#define PRESC_MASKB         (1<<CS00)
#else
#error "prescale unsupported"
#endif

// 1 Startbit, 8 Databits, 1 Stopbit = 10 Bits/Frame
// A timer interrupt must be set to interrupt at three times the required baud rate.
#define SOFTUART_BAUD_RATE 1200
#define F_CPU 1200000

//#define TIMERTOP (F_CPU/SOFTUART_PRESCALE/SOFTUART_BAUD_RATE/3 -1)
#define TIMERTOP (F_CPU/SOFTUART_PRESCALE/SOFTUART_BAUD_RATE/6 -1) // Corect PWM
#define TX_NUM_OF_BITS (10)
#define SU_TRUE 1
#define SU_FALSE 0

#define SOFTUART_TXPORT  PORTB
#define SOFTUART_TXDDR   DDRB
#define SOFTUART_TXBIT   PB2

volatile static unsigned char  flag_tx_ready;
volatile static unsigned char  timer_tx_ctr;
volatile static unsigned char  bits_left_in_tx;
volatile static unsigned short internal_tx_buffer; /* ! mt: was type uchar - this was wrong */

#define set_tx_pin_high() (SOFTUART_TXPORT SET (1<<SOFTUART_TXBIT))
#define set_tx_pin_low()  (SOFTUART_TXPORT CLR (1<<SOFTUART_TXBIT))


#define BATTERY_OVER   PB0
#define BATTERY_UNDER  PB4

#define set_over_pin()  (PORTB SET (1<<BATTERY_OVER))
#define clr_over_pin()  (PORTB CLR (1<<BATTERY_OVER))
#define set_under_pin() (PORTB SET (1<<BATTERY_UNDER))
#define clr_under_pin() (PORTB CLR (1<<BATTERY_UNDER))

//volatile static unsigned short ADC_result;


#define EE_COMM 0
// eeoprom calibration values
#define EE_OSZI 0 //clock calibration (calibrate from programmer)
#define EE_ADC_CALIBRATED 1
#define EE_ADC_HI 2
#define EE_ADC_LO 3

// MAIN data
volatile unsigned short cnt = 0;
volatile char ADC_done = 0;
volatile char ADC_go = 0;
unsigned char comm_mode = 0;

char tx_data[8];

// INT
ISR(TIM0_COMPA_vect)
{
	char tmp;
	// Transmitter Section
	if (flag_tx_ready)
	{
		tmp = timer_tx_ctr;
		if (--tmp <= 0)
		{
			if (internal_tx_buffer & 0x01)
			{
				set_tx_pin_high();
				//set_tx_pin_low();
			}
			else
			{
				set_tx_pin_low();
				//set_tx_pin_high();
			}
			internal_tx_buffer >>= 1;
			tmp = 3;
			if (--bits_left_in_tx <= 0)
			{
				flag_tx_ready = SU_FALSE;
			}
		}
		timer_tx_ctr = tmp;
	}

	// ADC
	if (ADC_go == 1)
	{
		ADCSRA SET (1<<ADSC);   // Start ADC
		ADC_go = 0;
	}

	// Timer
	cnt++;
}

ISR(TIM0_COMPB_vect)
{
	//  cnt--;
}

ISR(ADC_vect)
{
	ADC_done = 1;
	//  ADCSRA SET (1<<ADSC); Start ADC
}


// SOFT-UART TX
unsigned char softuart_can_transmit( void )
{
	return ( flag_tx_ready );
}

void softuart_putchar( const char ch )
{
	while ( flag_tx_ready ) {
		; // wait for transmitter ready
		// add watchdog-reset here if needed;
	}

	// invoke_UART_transmit
	timer_tx_ctr       = 3;
	bits_left_in_tx    = TX_NUM_OF_BITS;
	internal_tx_buffer = ( ch<<1 ) | 0x200;
	flag_tx_ready      = SU_TRUE;
}

void softuart_puts( const char *s )
{
	while ( *s ) {
		softuart_putchar( *s++ );
	}
}

void softuart_puts_p( const char *prg_s )
{
	char c;

	while ( ( c = pgm_read_byte( prg_s++ ) ) ) {
		softuart_putchar(c);
	}
}

// INIT
void adc_init()
{

	ADMUX = (1<<REFS0) | (0<<ADLAR) | (1<<MUX1) | (1<<MUX0);   // 1.1V=VREF, PB3 input MUX

	ADCSRA = (1<<ADEN) | (0<<ADSC) | (0<<ADATE) | (0<<ADIF) | (0<<ADIE) | (1<<ADPS2) | (0<<ADPS1) | (0<<ADPS0);

	//  DIDR0 = (1<<ADC0D) | (0<<ADC1D) | (0<<ADC2D ) | (0<<ADC3D); // ADC0 input buffer OFF
	DIDR0 = (0<<ADC0D) | (0<<ADC1D) | (0<<ADC2D ) | (1<<ADC3D);   // ADC0 input buffer OFF

	//  Initialise ADC, ignore first result
	ADCSRA SET (1<<ADSC);      // START ACDC
	while( ADCSRA & (1<<ADSC) ) {} // Pool ADC
	
	ADCSRB = (0<<ADTS2) | (1<<ADTS1) | (1<<ADTS0 );
	ADCSRA SET (1<<ADIE);      // Enable ADC interrupt
	
	// ADCSRA SET (1<<ADSC);     // START ACD
}

static void timer_init(void)
{
	unsigned char sreg_tmp;
	
	sreg_tmp = SREG;
	cli();
	CLKPR=0b10000011;			//system clock prescaler by 8
	OCR0A = TIMERTOP;          /* set top */
	OCR0B = 0;                 /* set PWM-B */

	TCCR0A = PRESC_MASKA | (1<<WGM00) | (1<<COM0B1) | (0<<COM0B0);
	TCCR0B = PRESC_MASKB | (1<<WGM02);

	TIMSK0 |= CMPINT_EN_MASK;

	TCNT0 = 0;                 /* reset counter */
	
	SREG = sreg_tmp;
}


// EEPROM
void EEPROM_write(unsigned char ucAddress, unsigned char ucData)
{

	while(EECR & (1<<EEWE));             // Wait for completion of previous write

	EECR = (0<<EEPM1)|(0>>EEPM0);        // Set Programming mode

	EEARL = ucAddress;                   // Set up address and data registers
	
	EEDR = ucData;
	
	EECR |= (1<<EEMWE);                  // Write logical one to EEMPE

	EECR |= (1<<EEWE);                   // Start eeprom write by setting EEPE
}

unsigned char EEPROM_read(unsigned char ucAddress)
{

	while(EECR & (1<<EEWE));             // Wait for completion of previous write

	EEARL = ucAddress;                   // Set up address register

	EECR |= (1<<EERE);                   // Start eeprom read by writing EERE

	return EEDR;                         // Return data from data register
}

// Pomozne funkcije

char to_hex(char bits4)
{
	char tp_bits;
	tp_bits = bits4 & 0x0F;
	if (tp_bits <= 9)
	{
		return(48 + tp_bits);
	}
	else
	{
		return(55 + tp_bits);
	}
}

// MAIN
int main(void)
{
	volatile unsigned char ad_H, ad_L;
	volatile unsigned short ADC_result;

	char Bat_status ='U'; 
	unsigned short miliVolt;
	wdt_enable(WDTO_2S);       // Set WDT for 2sek
	wdt_reset();               // Reset WDT.

	//DDRB = 0xFF;
	DDRB = 0x37;
	
	comm_mode = EEPROM_read(EE_COMM);
	set_over_pin();
	set_under_pin();

	flag_tx_ready = SU_FALSE;
	set_tx_pin_high();                   // mt: set to high to avoid garbage on init

	ADC_done = 0;
	ADC_go = 0;

	adc_init();
	timer_init();
	//read ADC calibration values from EEPROM if BMS was already calibrated
	if (EEPROM_read(EE_ADC_CALIBRATED)==1){
		ad_H = EEPROM_read(EE_ADC_HI); //ADC calibration value
		ad_L = EEPROM_read(EE_ADC_LO);
	}else{
		//take 10 measurements todo
		//for (i=0; i<10; i++){
		ADCSRA SET (1<<ADSC);             // START ACDC
		while( ADCSRA & (1<<ADSC) ) {}    // Pool ADC
		ad_L = ADCL;
		ad_H = ADCH;
		//miliVolt=(ad_H << 8) + ad_L;
		//}
		EEPROM_write(EE_ADC_HI, ad_H);
		EEPROM_write(EE_ADC_LO, ad_L);
		EEPROM_write(EE_ADC_CALIBRATED, 1);
	}
	
	miliVolt=(ad_H << 8) + ad_L;
	//miliVolt=6990;   //610
	miliVolt*=50; //multiplication before devision to minimize error
	miliVolt /= BMS_programm_voltage;
	sei();                               // ENABLE interupt

	while (1)
	{
		if (ADC_done == 1)                 // Preveri ali imamo novo meritev napetosti na bateriji
		{
			ADC_done = 0;  // Reset AD readout flag
			ad_L = ADCL;
			ad_H = ADCH;
			ADC_result = (ad_H << 8) + ad_L; // AD value			
			
			ADC_result *=50; //we multiplied miliVold variable to minimize devision error
			Bat_status = 'N'; //??????
			
						
			//softuart_putchar('.');
			//softuart_putchar(ad_L);
			//softuart_putchar(ad_H);
			Bat_status = 'N';

			// cell voltage at absolute maximum
			// - stop charging
			// - set signal on optocoupler
			// - set status on UART
			if (ADC_result > (miliVolt*Li_Max)){
				//softuart_putchar('q');
				Bat_status = 'q';
				clr_over_pin();  //optocoupler signaling
				set_under_pin();  //optocoupler signaling
				Bat_status = 'Q';
				if (OCR0B < TIMERTOP){
					OCR0B++; //increese pwm to maximum
				}
			}

			// Cell voltage more than full. Start balancing
			// - decrease charging curernt to 1A
			// - set signal on optocoupler
			// - set status on UART
			if ((ADC_result > (miliVolt*Li_Full)) && (ADC_result <= (miliVolt*Li_Max))){
				//softuart_putchar('O');
				Bat_status = 'O';
				clr_over_pin(); // optocoupler signaling
				set_under_pin();  //optocoupler signaling
				Bat_status = 'O';
				//increse PWM
				if (OCR0B < TIMERTOP)
				{
					OCR0B++;
				}
			}

			// Cell voltage below full.
			// - full charging
			// - set signal on optocoupler
			// - set status on UART
			if ((ADC_result <= (miliVolt*Li_Full)) && (ADC_result >= (miliVolt*Li_Min)))
			{
				//softuart_putchar('c');
				Bat_status = 'c';
				if (OCR0B > 0)
				{
					OCR0B--;
				}
			}
			

			//Battery undrewoltage. Set warning.
			if (ADC_result < (miliVolt*Li_Min))
			{
				//softuart_putchar('M');
				Bat_status = 'M';
				clr_under_pin();
				set_over_pin();  //optocoupler signaling
				Bat_status = 'M';
			}
			
			// send_adc();                      // Poslji meritev na UART
			tx_data[0] = '>';
			tx_data[1] = Bat_status;
			tx_data[2] = '-';
			tx_data[3] = to_hex(((ADC_result) / (miliVolt)) >> 8);
			tx_data[4] = to_hex(((ADC_result) / (miliVolt)) >> 4);
			tx_data[5] = to_hex(0);
			tx_data[6] = '\r';
			tx_data[7] = '\0';

			softuart_puts(tx_data);
		}
		if (cnt >= 360)                    // 1/20sek
		{
			//        PORTB SET (1<<PB4);
			cnt = 0;
			ADC_go = 1;                      // Naslednja prekinitev prozi ADC
		}
		wdt_reset();                       // Kuza pazi "sedi".
		
	}
}