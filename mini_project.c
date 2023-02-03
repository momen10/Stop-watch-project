/*
 * mini_project.c
 *
 *  Created on: Sep 14, 2022
 *      Author: Mo'men Ahmed
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
void control_display(unsigned char digit);

void display_zero(void);
void display_one(void);
void display_two(void);
void display_three(void);
void display_four(void);
void display_five(void);
void display_six(void);
void display_seven(void);
void display_eight(void);
void display_nine(void);

void enable_sec0(void);
void enable_sec1(void);
void enable_min0(void);
void enable_min1(void);
void enable_hour0(void);
void enable_hour1(void);

unsigned char sec0_digit=0, sec1_digit=0;
unsigned char min0_digit=0, min1_digit=0;
unsigned char hour0_digit=0, hour1_digit=0;

void set_timer1_comp(void)
{
	TCNT1=0;              //initializing the counter register to 0
	TCCR1A=0, TCCR1B=0;   //initializing all controls to 0
	TCCR1A = (1<<FOC1A) | (1<<FOC1B);  //not working with pwm module
	TCCR1B = (1<<WGM12) |(1<<CS12) |(1<<CS10); //enabling compare mode and prescaler 1024
	OCR1A =977;    //compare register
	TIMSK = (1<<OCIE1A);
}

ISR(TIMER1_COMPA_vect)
{
  sec0_digit ++;      //incrementing the seconds once the timer sends interrupt request
  if(sec0_digit==10)
  {
	  sec0_digit=0;
	  sec1_digit++;
  }
  if(sec1_digit==6)
  {
	  min0_digit++;
	  sec1_digit=0;
  }

  if(min0_digit==10)
  {
	  min0_digit=0;
	  min1_digit++;
  }
  if(min1_digit==6)
  {
	  hour0_digit++;
	  min1_digit=0;
  }

  if(hour0_digit==10)
  {
	  hour0_digit=0;
	  hour1_digit++;
  }
  if(hour1_digit==10)              //maximum time the stopwatch can display,
  {                                //after it it will begin from zero again
	  sec0_digit=0; sec1_digit=0;
	  	 min0_digit=0; min1_digit=0;
	  	 hour0_digit=0; hour1_digit=0;
  }
}

ISR(INT0_vect)
{
	sec0_digit=0; sec1_digit=0;     //in reset interrupt, begin from zero again,
	 min0_digit=0; min1_digit=0;    //set all digits to zero
	 hour0_digit=0; hour1_digit=0;
}

void INT0_INIT(void)
{
	DDRD &= ~(1<<PD2);  //pin 2 in port d is input
	PORTD |= (1<<PD2); //enabling pull up resistor
	GICR |= (1<<INT0); //enabling external interrupt INT0
	MCUCR |= (1<<ISC01); //generating interrupt request on falling edge
	MCUCR &= ~(1<<ISC00);
}

void INT1_INIT(void)
{
	DDRD &= ~(1<< PD3);  //pin 3 in port d is input
	MCUCR |= (1<<ISC10) | (1<<ISC11) ;  //enabling interrupt request on rising edge
	GICR |= (1<<INT1);   //enabling external interrupt INT1
}

ISR(INT1_vect)
{
	TCCR1B &= 0xf8;     //clearing the clock select registers to stop the timer (cs12,cs11,cs10)
}

void INT2_INIT(void)
{
	DDRB &= ~(1<<PB2);  //pin 2 in port b is input
	PORTB |= (1<<PB2); //enabling pull up resistor

	GICR |= (1<<INT2); //enabling external interrupt INT2
}

ISR(INT2_vect)
{
TCCR1B |= 0x05; //enabling the clock select bits again to be work on prescaler 1024
}


int main(void)
{
	DDRC |= 0x0f; //setting first 4 pins in port c as output
    PORTC &= 0xf0; //Initializing to zero
    DDRA |= 0x3f; //setting first 6 pins in port a as output
    PORTA &= 0xc0;//Initializing to zero
    sei();      //enabling the global interrupt bit
set_timer1_comp(); //starting the timer
INT0_INIT();       //calling the functions of 3 interrupts
INT1_INIT();
INT2_INIT();

    while(1)
    {    //displaying on the 7 segments
    	enable_sec0(); //enabling first 7 segment
    	control_display(sec0_digit);  //displaying the (second) digit on first 7 segment
    	_delay_ms(2); //delay 2 ms between each 7 segment and the other

    	enable_sec1();
    	control_display(sec1_digit);
    	_delay_ms(2);

    	enable_min0();
    	control_display(min0_digit);
    	_delay_ms(2);

    	enable_min1();
    	control_display(min1_digit);
    	_delay_ms(2);

    	enable_hour0();
    	control_display(hour0_digit);
    	_delay_ms(2);

    	enable_hour1();
    	control_display(hour1_digit);
    	_delay_ms(2);
    }
}

void control_display(unsigned char digit) //function to display on 7 segments
{                                     //accoring to the passed variable
	switch(digit)
	{
	case 0: display_zero();
	         break;
	case 1: display_one();
		         break;
	case 2: display_two();
		         break;
	case 3: display_three();
		         break;
	case 4: display_four();
		         break;
	case 5: display_five();
		         break;
	case 6: display_six();
		         break;
	case 7: display_seven();
		         break;
	case 8: display_eight();
		         break;
	case 9: display_nine();
		         break;
	}
}
void display_zero(void)
{
	PORTC &= 0xf0;
}
void display_one(void)
{
	PORTC &= 0xf0;
PORTC |= 0x01;
}
void display_two(void)
{
	PORTC &= 0xf0;
PORTC |= 0x02;
}
void display_three(void)
{
	PORTC &= 0xf0;
PORTC |= 0x03;
}
void display_four(void)
{
	PORTC &= 0xf0;
PORTC |= 0x04;
}
void display_five(void)
{
	PORTC &= 0xf0;
PORTC |= 0x05;
}
void display_six(void)
{
	PORTC &= 0xf0;
PORTC |= 0x06;
}
void display_seven(void)
{
	PORTC &= 0xf0;
PORTC |= 0x07;
}
void display_eight(void)
{
	PORTC &= 0xf0;
PORTC |= 0x08;
}
void display_nine(void)
{
	PORTC &= 0xf0;
PORTC |= 0x09;
}
// functions to enable each seven segment by insertion method
void enable_sec0(void)
{
	PORTA &= 0xc0;
	PORTA |= (1<<PA0);
}
void enable_sec1(void)
{
	PORTA &= 0xc0;
	PORTA |= (1<<PA1);
}
void enable_min0(void)
{
	PORTA &= 0xc0;
		PORTA |= (1<<PA2);
}
void enable_min1(void)
{
	PORTA &= 0xc0;
		PORTA |= (1<<PA3);
}
void enable_hour0(void)
{
	PORTA &= 0xc0;
		PORTA |= (1<<PA4);
}
void enable_hour1(void)
{
	PORTA &= 0xc0;
		PORTA |= (1<<PA5);
}
