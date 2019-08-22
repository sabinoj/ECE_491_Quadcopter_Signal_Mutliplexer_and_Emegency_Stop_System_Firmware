/*
 *	Engineer: John Sabino
 *	Start Date: 09/26/2018
 *	End Date: ??/??/????
 *	Micro-Controller: Atmega8L
 *	Power Voltage: 5V
 *	Crystal Frequency: 6.5536MHz
 *	Project Description:
 *		A signal multiplexer (sigmux) required to provide manual control over the
 *		quad-copter project for ECE 491, and provide an emergency stop system.
 *
 ****************************************/

/*Pin Definitions:
Pin:	|  Direction:     | Netname:			| Function:
PB0	   INPUT	    RC_Gear_Signal	  	  Input Capture Pin
PB1	   OUTPUT	    uC_Gear_Signal	  	  Output Compare 1A
PB2	   OUTPUT	    uC_Status_LED	  	  GPIO
PB3	   OUTPUT           MOSI                  	  SPI - MOSI
PB4        INPUT            MISO                  	  SPI - MISO
PB5        OUTPUT           SCK                   	  SPI - SCK
PB6        INPUT	    XTAL_N		  	  XTAL1
PB7	   OUTPUT           XTAL_P		          XTAL2

PC0        OUTPUT           MUX_CONTROL_GEAR              GPIO
PC1        OUTPUT           MUX_CONTROL_RUDDER            GPIO
PC2        OUTPUT           MUX_CONTROL_ELEVATOR  	  GPIO
PC3        OUTPUT           MUX_CONTROL_AILERON   	  GPIO
PC4        OUTPUT           MUX_CONTROL_THROTTLE  	  GPIO
PC5        OUTPUT           RELAY_CTRL            	  GPIO
PC6	   UNUSED           RESET                 	  /RESET

PD0        INPUT            Calibration_Pin	  	  GPIO
PD1        OUTPUT	    Manual_Mode_LED	  	  GPIO           
PD2        INPUT            Manual_Elevator_Pin   	  GPIO
PD3        INPUT            Manual_Throttle_Pin   	  GPIO
PD4        OUTPUT           Autonomous_Mode_LED   	  GPIO
PD5        OUTPUT           Emergency_Stop_Mode_LED 	  GPIO
PD6        INPUT            Manual_Aileron_Pin    	  GPIO
PD7        INPUT            Manual_Rudder_Pin     	  GPIO

ADC6	   UNUSED           GND                   	  ADC
ADC7	   UNUSED           GND                   	  ADC

NOTE: All LEDs for the different input pins, are actually built as pull-ups so the micro-controller does not
	need to control them.
*/

//Please review page 74 of the Atmega8L complete datasheet for external clocking source.
//Remember, an external source is prescaled by 2 before being used in the micro-controller.

//Preprocessor Flags:
#undef  ____CALIBRATION_MODE_ENABLED____


//Preprocessor Definitions:
#define F_CPU 			(6553600UL)		//External CPU Clock = 3.2768MHz now 6.5536MHz
#define TIMER_CLK		(51200UL)		//External Timer Clock = 51.2KHz = 3.2768MHz / 64
#define WDT_OSCILLATOR		(1000000UL)		//Internal Oscillator 1MHz.

//Positive Duty Cycle/Time:
#define ___FAILSAFE_MODE_MS___		(1.76)		//(15.0)		//15mS	 = 75%
#define ___ATITUDE_MODE_MS___		(1.50)		//(10.2)		//10.2mS = 51%
#define ___MANUAL_MODE_MS___		(1.16)		//(04.0)		//4mS    = 20%
#define ___PERIOD_MS___			(20.0)		//20mS   = 50Hz

//Used for the compare match for Gear Output:
#define ___FAILSAFE_MODE_DUTY___ 	((uint16_t)( 2 * ((___FAILSAFE_MODE_MS___ / ___PERIOD_MS___) * 1024.0)))
#define ___ATITUDE_MODE_DUTY___		((uint16_t)( 2 * ((___ATITUDE_MODE_MS___ / ___PERIOD_MS___) * 1024.0)))
#define ___MANUAL_MODE_DUTY___		((uint16_t)( 2 * ((___MANUAL_MODE_MS___ / ___PERIOD_MS___) * 1024.0)))


#define __MAN_AILERON_PIN__	(1 << PD6)
#define __MAN_ELEVATOR_PIN__	(1 << PD2)
#define __MAN_THROTTLE_PIN__	(1 << PD3)
#define __MAN_RUDDER_PIN__	(1 << PD7)
#define __CALIBRATION_PIN__	(1 << PD0)

#define __MUX_GEAR__		(PC0)
#define __MUX_RUDDER__		(PC1)
#define __MUX_ELEVATOR__	(PC2)
#define __MUX_AILERON__		(PC3)
#define __MUX_THROTTLE__	(PC4)
#define __RELAY_CONTROL__	(PC5)

#define __MANUAL_MODE_LED__	(PD1)
#define __AUTONOMOUS_MODE_LED__	(PD4)
#define __ESTOP_MODE_LED__	(PD5)

//Libraries:
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <avr/sleep.h>

//Macros:
#define ____UPPER_BOUND_TOLERANCE____(input)	((uint16_t)(1.10 * input))
#define ____LOWER_BOUND_TOLERANCE____(input)	((uint16_t)(0.90 * input))

//Global Variables:
uint16_t 		 Positive_Duty_Time  		= 0;
float 		 	 Positive_Duty_Cycle 		= 0.0;
volatile static uint8_t	 Manual_Mode_Mask 		= 0;
volatile static uint8_t  Duty_Ready_Flag 		= 0;
volatile static uint8_t  Fall_Desynchronized_Flag 	= 0;
uint8_t			 Auto_Count			= 0;
uint8_t			 Man_Count			= 0;
uint8_t			 Fail_Count			= 0;
//Interrupt Global Variables:
volatile static uint16_t Rise_Point 			= 0;
volatile static uint16_t Fall_Point 			= 0;


//Function Prototypes:
inline void Initialization(void);
inline void Check_Manual_Pins(void);
inline void Calculate_Duty_Cycle(void);
inline void Calibration_Check(void);
inline void Set_Emergency_Stop_Mode(void);
inline void Set_Manual_Mode(void);
inline void Set_Autonomous_Mode(void);

void main()
{
	PORTC = 0;
	_delay_ms(1000);
	//Initialize the pins, interrupts, and timer units.
	Initialization();
	
	//After initialization of the hardware, set the system into emergency stop mode.
	Set_Emergency_Stop_Mode();

	for(;;)
	{
		wdt_reset();			//Feed the watchdog.
		Check_Manual_Pins();		//Check the status of the input pins.
		Calculate_Duty_Cycle();		//Identify what mode to be in.
		wdt_reset();
	//	sleep_cpu();			//Enter into Idle mode for 6 CLK cycles.
		//_delay_ms(500);
	}//End infinite for loop
	
	//If we got here, something has gone horribly wrong.
}//End main


//Functions:
inline void Initialization ()
{
	//Disable interrupts until everything has been set up.
	cli();


	//Set data directionality of the pins.
	//1 = output | 0 = input
	DDRB = (0 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3) | (0 << PB4) | (1 << PB5) | (0 << PB6) | (1 << PB7);
	DDRC = (1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC3) | (1 << PC4) | (1 << PC5) | (0 << PC6);
	DDRD = (0 << PD0) | (1 << PD1) | (0 << PD2) | (0 << PD3) | (1 << PD4) | (1 << PD5) | (0 << PD6) | (0 << PD7);


	//Set the input pins to have pull-up resistors.
	//1 = pull-up enabled | 0 = pull-up disabled
	PORTD = (1 << PD0) | (1 << PD2) | (1 << PD3) | (1 << PD6) | (1 << PD7); 

	//Set the MCU to enter into Idle mode whenever the sleep command is called.
	MCUCR  = (1 << SE);					//Enables sleep mode when sleep is called.
	MCUCR |= (0 << SM2) | (0 << SM1) | (0 << SM0);		//When sleep is called, enter Idle mode.	

//DEBUGGING SPI:
//DDRB |= (1 << PB5) | (1 << PB3) | (1 << PB2);
//SPCR = (1 << SPE) | (1 << MSTR);
	//Configure the Watchdog Timer:
	WDTCR = (1 << WDE) | (1 << WDCE);			//Enable the Watchdog Timer.
	WDTCR |= (1 << WDP2) | (1 << WDP1) | (0 << WDP0) | (1 << WDE);	//Timeout set to 1.0 seconds.


	//Configure Timer1 for Output Compare (PWM 10-bit).
	OCR1A 	= ___FAILSAFE_MODE_DUTY___;			//Set the duty cycle for gear signal.
	TCCR1A  = (1 << COM1A1);				//Clear OC1A on Compare Match.
	TCCR1A |= (1 << WGM12) | (1 << WGM11) | (1 << WGM10);	//Set the system to Fast PWM Mode (10-bit).
	TCCR1B  = (0 << WGM13);					//Part of configuring Fast PWM Mode (10-bit).
	


	//Configure Timer1 for Input Capture Interrupts.
	TCCR1B |= (1 << ICES1);		        		//Trigger the Input Capture on rising edge.
	TCCR1B |= (1 << ICNC1);					//Enable noise canceler.
	TIMSK   = (1 << TICIE1);				//Enable the Input Capture Interrupt.
//	TIMSK  |= (1 << TOIE1);					//Enable the Overflow Interrupt.



	//Enable Timer1 (Initially the timer is disabled/stopped).
	TCCR1B |= (0 << CS12) | (1 << CS11) | (1 << CS10); 	//Set the clock to F_CPU / 256 64	



	wdt_reset();							//Feed the watchdog.



	//Enable interrupts now that everything has been set up.
	sei();
}//End Initialization

inline void Check_Manual_Pins()
{	
	Manual_Mode_Mask = 0;
	if (PORTD & __MAN_AILERON_PIN__)
		Manual_Mode_Mask |= (1 << __MUX_AILERON__);
	if (PORTD & __MAN_ELEVATOR_PIN__)
		Manual_Mode_Mask |= (1 << __MUX_ELEVATOR__);
	if (PORTD & __MAN_RUDDER_PIN__)
		Manual_Mode_Mask |= (1 << __MUX_RUDDER__);
	if (PORTD & __MAN_THROTTLE_PIN__)
		Manual_Mode_Mask |= (1 << __MUX_THROTTLE__);	
		
}//End Check_Input_Pins

inline void Calculate_Duty_Cycle()
{
	//Only calculate the duty cycle whenever the data is ready.
	if (Duty_Ready_Flag)
	{
	//	if (Fall_Desynchronized_Flag)
//DEBUGING SPI:
/*
PORTB |= (1 << PB2);
PORTB &= ~(1 << PB2);
//SPDR = 255;
//while (!(SPSR & (1 << SPIF)) );
SPDR = Rise_Point >> 8;
while (!(SPSR & (1 << SPIF)) ); //Wait for transmission to finish.
SPDR = Rise_Point & 0xFF;
while (!(SPSR & (1 << SPIF)) );
SPDR = Fall_Point >> 8;
while (!(SPSR & (1 << SPIF)) );
SPDR = Fall_Point & 0xFF;
while (!(SPSR & (1 << SPIF)) );
PORTB |= (1 << PB2);
*/
		if (Rise_Point > Fall_Point)
		{
			//Fall_Point += 0x03FF;	//Add the TOP value to the fall point to get an accurate time.
			//Fall_Desynchronized_Flag = 0;
			Positive_Duty_Time = Rise_Point - Fall_Point;
		}
		else
			Positive_Duty_Time = Fall_Point - Rise_Point;
		//Duty_Ready_Flag = 0;		//Clear the flag to prevent any issues with further calculations.
		//Positive_Duty_Time  = Fall_Point - Rise_Point;	//Delta = Final - Initial.
		//Positive_Duty_Cycle = Positive_Duty_Time / 0x03FF;
//if (Positive_Duty_Time > 10 )
//	PORTC |= (1 << PC0);
//else if (Positive_Duty_Time > 0)
//	PORTC |= (1 << PC2);
/*
PORTB &= ~(1 << PB2);
SPDR = Positive_Duty_Time >> 8;
while (!(SPSR & (1 << SPIF)));
SPDR = Positive_Duty_Time & 0xFF;
while (!(SPSR & (1 << SPIF)));
PORTB |= (1 << PB2);
*/

		Fall_Point = 0;
		Rise_Point = 0; 
		Duty_Ready_Flag = 0;
		if ( (Positive_Duty_Time >= ____LOWER_BOUND_TOLERANCE____(___ATITUDE_MODE_DUTY___) ) && 
 			(Positive_Duty_Time <= ____UPPER_BOUND_TOLERANCE____(___ATITUDE_MODE_DUTY___)))
		{
		//PORTC = (1 << PC0);
			//Set_Autonomous_Mode();		
			++Auto_Count;
		}//End else if statment

		else if ( (Positive_Duty_Time >= ____LOWER_BOUND_TOLERANCE____(___MANUAL_MODE_DUTY___) ) &&
			(Positive_Duty_Time <= ____UPPER_BOUND_TOLERANCE____(___MANUAL_MODE_DUTY___)))
		{
		//PORTC = (1 << PC2);
			//Set_Manual_Mode();
			++Man_Count;
		}//End else if statement

		else
		{
			//Set_Emergency_Stop_Mode();
			++Fail_Count;
		}//End else statement
	
		if (Auto_Count > 3)
		{
			Auto_Count = 0;
			Man_Count = 0;
			Fail_Count = 0;
			PORTC = (1 << PC0);
			Set_Autonomous_Mode();
		}//End if statement
		else if (Man_Count > 3)
		{
			Auto_Count = 0;
			Man_Count = 0;
			Fail_Count = 0;
			PORTC = (1 << PC2);
			Set_Manual_Mode();
		}//End else if
		else if (Fail_Count > 3)
		{
			Auto_Count = 0;
			Man_Count = 0;
			Fail_Count = 0;
			Set_Emergency_Stop_Mode();
		}//End else if
	}//End if statement	
}//End Calculate_Duty_Cycle
#ifdef ____CALIBRATION_MODE_ENABLED____
	inline void Calibration_Check()
	{
		//TODO: WILL NEED TO ADD MORE LEDS TO MAKE THIS WORK PROPERLY.

		//If the calibration pin is enabled, then run the calibration, and save it to EEPROM.
		if (PORTD & __CALIBRATION_PIN__)
		{
		
		}//End if statement
		//If the calibration pin is disabled, then load the calibration from EEPROM.
		else
		{
		
		}//End else statement
	}//End Calibration_Check
#endif

inline void Set_Emergency_Stop_Mode()
{
	//Turn the relay off.
	PORTC &= ~(1 << __RELAY_CONTROL__);

	//Turn off the other LEDs.
	PORTD &= ~( (1 << __MANUAL_MODE_LED__) | (1 << __AUTONOMOUS_MODE_LED__) );
	
	//Turn on the ESTOP LED to indicate which mode we're in.
	PORTD |= (1 << __ESTOP_MODE_LED__);
	
	//JUST A DEBUG STATEMENT.
	PORTC = (1 << PC3);
}//End Set_Emergency_Stop_Mode


inline void Set_Manual_Mode()
{
	//Turn the relay on.
	PORTC |= (1 << __RELAY_CONTROL__);
	
	//Turn off the other LEDs.
	PORTD &= ~( (1 << __ESTOP_MODE_LED__) | (1 << __AUTONOMOUS_MODE_LED__) );

	//Turn on the Manual Mode LED to indicate which mode we're in.
	PORTD |= (1 << __MANUAL_MODE_LED__);

//	PORTC = Manual_Mode_Mask;
	//PORTC = (1 << PC0);
}//End Set_Manual_Mode


inline void Set_Autonomous_Mode()
{
	//Turn the relay on.
	PORTC |= (1 << __RELAY_CONTROL__);
	
	//Turn off the other LEDs.
	PORTD &= ~( (1 << __ESTOP_MODE_LED__) | (1 << __MANUAL_MODE_LED__) );

	//Turn on the Autonomous Mode LED to indicate which mode we're in.
	PORTD |= (1 << __AUTONOMOUS_MODE_LED__);

//	PORTC = 0x00;
	//PORTC = (1 << PC2);
}//End Set_Autonomous_Mode

//===========================================
//Interrupt Service Routines:
//===========================================

//Timer1 Input Capture Interrupt:
//In this interrupt we are getting the rising and falling edge to identify the duty cycle of the input signal.
ISR (TIMER1_CAPT_vect)
{
	//ICF1 is cleared once this interrupt is entered.

	if (Duty_Ready_Flag)
	{
		//Clear the flag and quit for now.
		TIFR |= (1 << ICF1);
		return;
	}
	//Figure out if this is a rising or falling edge, and store it appropriately.
	if ( TCCR1B & (1 << ICES1) )
	{
		//Rise_Point = ICR1H;
		//Rise_Point = Rise_Point << 8;
		//Rise_Point += ICR1L;
		Rise_Point = ICR1;	
		Duty_Ready_Flag = 0;
	}//End if statement
	else
	{
/*
		Fall_Point = ICR1H;
		Fall_Point = Fall_Point << 8;
		Fall_Point += ICR1L;
*/
		Fall_Point = ICR1;
		Duty_Ready_Flag = 1;
	}//End else statement

	TCCR1B ^= (1 << ICES1);		//Toggle the edge detection (this is an XOR operation to toggle the bit).
	TIFR |= (1 << ICF1);
}//End Timer 1 Input Capture Interrupt

//Timer1 Overflow Interrupt: 
//In the event that the falling edge happens after the timer resets (due to desynchornization of the signal and
// the micro-controller) this will identify that it needs to compensate.
ISR (TIMER1_OVF_vect)
{
	Fall_Desynchronized_Flag = 0;

	if (!Duty_Ready_Flag)
	{
		Fall_Desynchronized_Flag = 1;	
	}//End if statement
}//End Timer 1 Overflow Interrupt

