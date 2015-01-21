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
//#include <avr/pgmspace.h>
#include <util/crc16.h>

// Voltage settings for different battery types
/*****************************************************/
// Programming voltage for ACD calibration [0.01V]
#define BMS_programm_voltage (4200)

// maximum alowable voltage of cell.
#define Li_Max (4250)

// optimum full voltage. At this value BMS will start PWM on bypass transistor
#define Li_Full (4200) // mV

// minimum cell voltage
#define Li_Min  (3000)

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
//#define TIMERTOP (F_CPU/SOFTUART_PRESCALE/SOFTUART_BAUD_RATE/6 -1) // Correct PWM
#define TIMERTOP (165)
#define TX_NUM_OF_BITS (10)
#define SU_TRUE 1
#define SU_FALSE 0

#define SOFTUART_TXPORT  PORTB
#define SOFTUART_TXDDR   DDRB
#define SOFTUART_TXBIT   PB4

volatile static unsigned char  flag_tx_ready;
volatile static unsigned char  timer_tx_ctr;
volatile static unsigned char  bits_left_in_tx;
volatile static unsigned short internal_tx_buffer; /* ! mt: was type uchar - this was wrong */


#define set_tx_pin_high() (SOFTUART_TXPORT SET (1<<SOFTUART_TXBIT))
#define set_tx_pin_low()  (SOFTUART_TXPORT CLR (1<<SOFTUART_TXBIT))



//#define EE_COMM 0
// eeoprom calibration values
#define EE_OSZI 0 //clock calibration (calibrate from programmer)
#define EE_ADC_CALIBRATED 1
#define EE_ADC_HI 2
#define EE_ADC_LO 3

// MAIN data
volatile unsigned short cnt1 = 0, cnt2 = 0;
volatile char ADC_done = 0,ADC_go=0;
//volatile char 100msFlag = 0;
//unsigned char comm_mode = 0;
 unsigned short int  RefVolt=0;
 unsigned char avarageCounter=0;
unsigned char PWM=0;
 unsigned short int  ADC_result=0;
unsigned short int  ADC_avgr[10];
//char tx_data[8];

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

	// Timer
	cnt1++;
	cnt2++;

}

/*ISR(TIM0_COMPB_vect)
{
	//  cnt--;
}*/

ISR(ADC_vect)
{
	ADC_done = 1;
}


// SOFT-UART TX
/*unsigned char softuart_can_transmit( void )
{
	return ( flag_tx_ready );
}*/

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
/*
void softuart_puts( const char *s )
{
	while ( *s ) {
		softuart_putchar( *s++ );
	}
}*/

/*void softuart_puts_p( const char *prg_s )
{
	char c;

	while ( ( c = pgm_read_byte( prg_s++ ) ) ) {
		softuart_putchar(c);
	}
}*/

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

/*char to_hex(char bits4)
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
}*/
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
	
	//read RC calibration
	OSCCAL = EEPROM_read(EE_OSZI);
}


//void sendData(unsigned short ADC_resultat,unsigned short RefVoltage,char PWM){
void sendData(){
//ADC_result=10240;
//RefVolt=10240;
//PWM=255;
		//crc
		uint8_t crc8=0;
		// send to UART
		softuart_putchar('>');
		crc8 = _crc_ibutton_update(crc8, '>');
			
		softuart_putchar((ADC_result & 0xFF00L) >> 8);   // Apply the lower middle byte of the data 
		crc8= _crc_ibutton_update(crc8, (ADC_result & 0xFF00L) >> 8);

		softuart_putchar((ADC_result & 0x00FFL));      // Apply the low byte of the data 
		crc8= _crc_ibutton_update(crc8, (ADC_result & 0x00FFL));

		softuart_putchar((RefVolt & 0xFF00L) >> 8);   // Apply the lower middle byte of the data
		crc8= _crc_ibutton_update(crc8, (RefVolt & 0xFF00L) >> 8);
		
		softuart_putchar((RefVolt & 0x00FFL));      // Apply the low byte of the data
		crc8= _crc_ibutton_update(crc8, (RefVolt & 0x00FFL));
			
		softuart_putchar(PWM);
		crc8= _crc_ibutton_update(crc8, PWM);
		
		softuart_putchar(crc8);
		softuart_putchar(0xEE);
		
}

// MAIN
int main(void)
{
	
	
			for (int i=0; i<10; i++){
				ADC_avgr[i]=111;
			}	
	wdt_enable(WDTO_2S);       // Set WDT for 2sek
	wdt_reset();               // Reset WDT.

	//DDRB = 0xFF;
	DDRB = 0x37;

	flag_tx_ready = SU_FALSE;
	set_tx_pin_high();                   // mt: set to high to avoid garbage on init

	ADC_done = 0;
	ADC_go = 0;

	adc_init();
	timer_init();
	
	//read ADC calibration values from EEPROM if BMS was already calibrated
	if (EEPROM_read(EE_ADC_CALIBRATED)==1){
		RefVolt=(EEPROM_read(EE_ADC_HI) << 8) + EEPROM_read(EE_ADC_LO);
	}else{
		//take 10 measurements
		for (int i=0; i<10; i++){
			ADCSRA SET (1<<ADSC);             // START ACDC
			while( ADCSRA & (1<<ADSC) ) {}    // Pool ADC
			RefVolt+=(ADCH << 8) + ADCL;
		}
		EEPROM_write(EE_ADC_HI, ((RefVolt & 0xFF00L) >> 8));
		EEPROM_write(EE_ADC_LO, ((RefVolt & 0x00FFL)));
		EEPROM_write(EE_ADC_CALIBRATED, 1);
	}

	sei();                               // ENABLE interupt

		
	while (1)
	{
		//10ms interrupt for ADC read. 
		if(cnt1>=36) cnt1=0;
		//for first 2.5ms we stop PWM
		if(cnt1<=9){
			OCR0B=0;//stop PWM
			ADC_go=0;
		}else 
		{
			if(ADC_go==0)
			{
				ADC_go=1;
				ADCSRA SET (1<<ADSC); //Start ADC
				//sendData();
			}
		}
		
		//if conversion is done we add ADC value to buffer and stop PWM
		if(ADC_done){
			ADC_done=0;
			//OCR0B=(char)PWM; //turn on PWM
			
			//add value into circular buffer
			ADC_avgr[avarageCounter]=(ADCH<< 8) + ADCL; // AD value
			//ADC_avgr[avarageCounter]=100; // AD value
			avarageCounter++;
			if(avarageCounter>=10) avarageCounter=0;
			ADC_result=(ADCH<< 8) + ADCL;
		}

		//100ms interrupt
		if (cnt2 >= 360)                    // 1/20sek
		{
			cnt2 = 0;
			//calculate ADC value from last 10 measurements
			/*ADC_result=0;
			for (int i=0; i<10; i++){
				ADC_result +=ADC_avgr[i];
				//ADC_result +=100;
			}*/
			
			// Cell voltage more than full. Start balancing
			if ((ADC_result > RefVolt)){
				//increse PWM
				if (PWM < TIMERTOP)
				{
					PWM++;
				}
			}
			// Cell voltage below full.
			// - full charging
			if ((ADC_result <= RefVolt))
			{
				if (PWM > 0)
				{
					PWM--;
				}
			}
			
			//send the data to UART
			//sendData(ADC_result, RefVolt, PWM);
			sendData();
			
		}			




		wdt_reset();                       // Kuza pazi "sedi".	
	}
}

		