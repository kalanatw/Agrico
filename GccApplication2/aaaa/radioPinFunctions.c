/*
* ----------------------------------------------------------------------------
* “THE COFFEEWARE LICENSE” (Revision 1):
* <ihsan@kehribar.me> wrote this file. As long as you retain this notice you
* can do whatever you want with this stuff. If we meet some day, and you think
* this stuff is worth it, you can buy me a coffee in return.
* -----------------------------------------------------------------------------
* Please define your platform spesific functions in this file ...
* -----------------------------------------------------------------------------
*/

#include <avr/io.h>

// Modify these variables to customize the ports/pins the RF module will use
#define RF_DDR  DDRC
#define RF_PORT PORTC
#define RF_PIN  PINC

#define set_bit(reg,bit) reg |= (1<<bit)
#define clr_bit(reg,bit) reg &= ~(1<<bit)
#define check_bit(reg,bit) (reg&(1<<bit))

/* ------------------------------------------------------------------------- */

 void nrf24_setupPins()
 {
	 set_bit(DDRB,7); // CE output
	 set_bit(DDRB,4); // CSN output
	 set_bit(DDRB,5); // SCK output
	 set_bit(DDRB,3); // MOSI output
	 clr_bit(DDRB,6); // MISO input
 }
/* ------------------------------------------------------------------------- */
void nrf24_ce_digitalWrite(uint8_t state)
{
	if(state)
	{
		set_bit(DDRB,7);
	}
	else
	{
		clr_bit(DDRB,7);
	}
}
/* ------------------------------------------------------------------------- */
void nrf24_csn_digitalWrite(uint8_t state)
{
	if(state)
	{
		set_bit(DDRB,4);
	}
	else
	{
		clr_bit(DDRB,4);
	}
}
/* ------------------------------------------------------------------------- */
void nrf24_sck_digitalWrite(uint8_t state)
{
	if(state)
	{
		set_bit(DDRB,5);
	}
	else
	{
		clr_bit(DDRB,5);
	}
}
/* ------------------------------------------------------------------------- */
void nrf24_mosi_digitalWrite(uint8_t state)
{
	if(state)
	{
		set_bit(PORTA,3);
	}
	else
	{
		clr_bit(PORTA,3);
	}
}
/* ------------------------------------------------------------------------- */
uint8_t nrf24_miso_digitalRead()
{
	return check_bit(DDRB,6);
}
/* ------------------------------------------------------------------------- */
