#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>
#include "nrf24.h"
#define DHT11_PIN 40
#define uart_pin_output()    DDRB |= (1<<2)
#define uart_set_pin()        PORTB |= (1<<2)
#define uart_clr_pin()        PORTB &= ~(1<<2)
#define uart_bit_dly()        _delay_us(100)
uint8_t c=0,I_RH,D_RH,I_Temp,D_Temp,CheckSum;
uint8_t SendArray[3];
uint8_t temp;
uint8_t q = 0;
uint8_t tx_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
uint8_t rx_address[5] = {0xD7,0xD7,0xD7,0xD7,0xD7};

void uart_init();
void soil_moisture(void);
void Dht_11_Receive(void);

int main(void)
{
	while(1)
	{
		uart_init();
		nrf24_init();
		nrf24_config(2,3);
		nrf24_tx_address(tx_address);
		nrf24_rx_address(rx_address);
		soil_moisture();
		Dht_11_Receive();
		nrf24_send(SendArray);
		while(nrf24_isSending());
		_delay_ms(1000);
	}

}

//soil Moisture
void ADC_Init()
{
	DDRA=0x0;		/*  Make ADC port as input  */
	ADCSRA = 0x87;		/*  Enable ADC, fr/128  */
}

int ADC_Read()
{
	ADMUX = 0x40;		/* Vref: Avcc, ADC channel: 0  */
	ADCSRA |= (1<<ADSC);	/* start conversion  */
	while((ADCSRA&(1<<ADIF))==0);	/* monitor end of conversion interrupt flag */
	ADCSRA |=(1<<ADIF);	/* set the ADIF bit of ADCSRA register */
	return(ADCW);		/* return the ADCW */
}


void soil_moisture(void)
{
	
	ADC_Init();		/* initialize the ADC */
	int adc_value;
	int moisture;
	adc_value = ADC_Read();	/* Copy the ADC value */
	moisture = 100-(adc_value*100.00)/1023.00;
	_delay_ms(500);
	SendArray[2]=moisture;

}


//DHT11

void Request()				/* Microcontroller send start pulse/request */
{
	DDRD |= (1<<DHT11_PIN);
	PORTD &= ~(1<<DHT11_PIN);	/* set to low pin */
	_delay_ms(20);			/* wait for 20ms */
	PORTD |= (1<<DHT11_PIN);	/* set to high pin */
}

void Response()				/* receive response from DHT11 */
{
	DDRD &= ~(1<<DHT11_PIN);
	while(PIND & (1<<DHT11_PIN));
	while((PIND & (1<<DHT11_PIN))==0);
	while(PIND & (1<<DHT11_PIN));
}
uint8_t Receive_data()			/* receive data */
{
	for (int q=0; q<8; q++)
	{
		while((PIND & (1<<DHT11_PIN)) == 0);  /* check received bit 0 or 1 */
		_delay_us(30);
		if(PIND & (1<<DHT11_PIN))/* if high pulse is greater than 30ms */
		c = (c<<1)|(0x01);	/* then its logic HIGH */
		else			/* otherwise its logic LOW */
		c = (c<<1);
		while(PIND & (1<<DHT11_PIN));
	}
	return c;
}

void Dht_11_Receive(void)
{
	Begin:
	Request();		/* send start pulse */
	Response();		/* receive response */
	I_RH=Receive_data();	/* store first eight bit in I_RH */
	D_RH=Receive_data();	/* store next eight bit in D_RH */
	I_Temp=Receive_data();	/* store next eight bit in I_Temp */
	D_Temp=Receive_data();	/* store next eight bit in D_Temp */
	CheckSum=Receive_data();/* store next eight bit in CheckSum */

	if ((I_RH + D_RH + I_Temp + D_Temp) != CheckSum)
	{
		goto Begin;
	}

	SendArray[0]=I_RH;
	SendArray[1]=I_Temp;
	
}
void uart_init()
{
	uart_pin_output();
}
void uart_put_char(uint8_t tx)
{
	uint8_t i;

	/* Start condition */
	uart_clr_pin();
	uart_bit_dly();

	for(i=0;i<8;i++)
	{
		if(tx & (1<<i))
		{
			uart_set_pin();
		}
		else
		{
			uart_clr_pin();
		}

		uart_bit_dly();
	}

	/* Stop condition */
	uart_set_pin();
	uart_bit_dly();
}
