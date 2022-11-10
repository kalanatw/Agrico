/*
 * AgricoFin.c
 *
 * Created: 11/16/2020 8:07:38 PM
 * Author : Kalana Weerakoon
 */ 

#include <avr/io.h>
#define F_CPU 16000000UL		/* define Clock Frequency */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include "nrf24.h"
#include "LCD_16x2_H_file.h"	/* include 16x2 LCD Header file */
#include "USART_Interrupt.h"	/* include USART Header file */


#define SREG   _SFR_IO8(0x3F)
#define uart_pin_output()    DDRB |= (1<<6)
#define uart_set_pin()        PORTB |= (1<<6)
#define uart_clr_pin()        PORTB &= ~(1<<6)
#define uart_bit_dly()        _delay_us(100)


#define set_bit(reg,bit) reg |= (1<<bit)
#define clr_bit(reg,bit) reg &= ~(1<<bit)
#define check_bit(reg,bit) (reg&(1<<bit))


void GSM_Begin();
void GSM_Calling(char *);
void GSM_HangCall();
void GSM_Response();
void GSM_Response_Display();
void GSM_Msg_Read(int);
bool GSM_Wait_for_Msg();
void GSM_Msg_Display();
void GSM_Msg_Delete(unsigned int);
void GSM_Send_Msg(char* , char*);
void GSM_Delete_All_Msg();

char buff[160];			/* buffer to store responses and messages */
char status_flag = 0;		/* for checking any new message */
volatile int buffer_pointer;
char Mobile_no[15];		/* store mobile no. of received message */
char message_received[60];	/* save received message */
int position = 0;		/* save location of current message */
//variables of nrf_receive Function
uint8_t temp;
uint8_t q = 0;
uint8_t SendArray[3];
uint8_t tx_address[5] = {0xD7,0xD7,0xD7,0xD7,0xD7};
uint8_t rx_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
	
//end
char Mobile_no_User[15]="0763495189";
#define LCD_DisplayPort PORTC
#define LCD_DisplayPortDDR DDRC

void solenoid_valve_on();


 int getsoilvalue(void);
 void RainFun(void);
 void send_msg_MyFunction(void);
 void detailPrint();
void GSMmain();
 void uart_put_char(uint8_t tx);
// solenoid_off();
 //solenoid_on();
 int nrf_Receive(void);
 int soil;
 int humidity;
 int temperature;
char soilc[5];
char humidityc[5];
char tempc[5];
int rainFlag=0; //Becoming one if rain detected

int SEC=0;
int MIN=0;
int HOU=0;
int ho24flag=0;

int main(void)
{
    /* Replace with your application code */
    while (1) 
    {
		
		sei();
		TCCR1B |=   1<<CS12  | 1<<WGM12;
		TIMSK |= 1<<OCIE1A;
		OCR1A=31249; //clear TCNT1 timer counter
		LCD_Init();
		LCD_String_xy(1,0,"Welcome to Agrico");
		nrf_Receive();
		detailPrint();
		solenoid_valve_on();
		RainFun();
		GSMmain();
		
		if (soil<20)
		{
		  send_msg_MyFunction();
		  ISR (TIMER1_COMPA_vect);
		  if(ho24flag==1){
			  DDRA = (DDRA | (1<<2)); //MAKING PIN2 OUTPUT
			  PORTA = (PORTA | (1<<2));
			  int i=0;
			  for (i=0;i<600;i++)
			  {		if (rainFlag==1)break;
				  if(strstr( message_received,"OFF"))break;
				  _delay_ms(500);
			  }
			  PORTA =( PORTA | ~(1<<2));
		  }
		}
		   
		
    }
	
	rainFlag=0;
	ho24flag=0;
}

void GSMmain()
{
	buffer_pointer = 0;
	int is_msg_arrived;
	memset(message_received, 0, 60);
	LCD_Init();
	USART_Init(9600);	/* initialize USART */
	sei();
	LCD_String_xy(1,0,"GSM Initializing");
	_delay_ms(3000);
	LCD_Clear();
	LCD_String_xy(1,0,"AT");
	GSM_Begin();		/* Start GSM with AT*/
	LCD_Clear();
	
		if(status_flag==1){
			is_msg_arrived = GSM_Wait_for_Msg(); /*check for message arrival*/
			if(is_msg_arrived== true)
			{
				LCD_Clear();
				LCD_String_xy(1,0,"new message"); /* new message arrived */
				_delay_ms(1000);
				LCD_Clear();
				GSM_Msg_Read(position); /* read arrived message */
				_delay_ms(3000);
				
				/*check if received message is "call me" */
				if(strstr( message_received,"ON")){
					DDRA = DDRA | (1<<2); //MAKING PIN2 OUTPUT
					PORTA = PORTA | (1<<2);
					int i=0;
					for (i=0;i<600;i++)
					{
						if (rainFlag==1)break;
						if(strstr( message_received,"OFF"))break;
						_delay_ms(500);
					}
					PORTA = PORTA | ~(1<<2);
					
				}
					if(strstr( message_received,"OFF")){
						
						 PORTA = PORTA | ~(1<<2);
					}
					if(strstr( message_received,"INFO")){
						
						char masge[50]="Soil moisture,Humidity,Temperature..";
						strcat(masge,soilc);
						strcat(masge,",");
						strcat(masge,humidityc);
						strcat(masge,",");
						strcat(masge,tempc);
						GSM_Send_Msg(Mobile_no_User,masge);
					}
				
				
				LCD_Clear();
				GSM_Msg_Delete(position); /* to save SIM memory delete current message */
				LCD_String_xy(1,0,"Clear msg");
				GSM_Response();
				_delay_ms(1000);
				
			}
			
			
			is_msg_arrived=0;
			status_flag=0;
			LCD_Clear();
		}
		LCD_String_xy(1,0,"waiting for msg");
		memset(Mobile_no, 0, 14);
		memset(message_received, 0, 60);
		

	
}
ISR (TIMER1_COMPA_vect) //interupt service routine
{
	if (SEC<60)
	{
		SEC++;
	}
	if (SEC==60)
	{
		if (MIN<60)
		{
			MIN++;
		}
		SEC=0;
	}
	if (MIN==60)
	{
		if (HOU<24)
		{
			HOU++;
		}
		MIN=0;
	}
	if (HOU==24)
	{
		HOU=0;
		ho24flag=1;
		return;
	}
	
}


void detailPrint()
{
	int soil=SendArray[2];
	int humidity=SendArray[0];
	int temperature=SendArray[1];
	itoa(soil,soilc,10);
	itoa(humidity,humidityc,10);
	itoa(temperature,tempc,10);
	LCD_String_xy(1,0,"Soil Moisture:");
	LCD_Clear();
	_delay_ms(100);
	LCD_String(soilc);
	LCD_Clear();
	_delay_ms(100);
	LCD_String_xy(1,0,"Temperature..:");
	LCD_Clear();
	_delay_ms(100);
	LCD_String(tempc);
	LCD_Clear();
	_delay_ms(100);
	LCD_String_xy(1,0,"Humidity..:");
	LCD_Clear();
	_delay_ms(100);
	LCD_String(humidityc);
	LCD_Clear();
	_delay_ms(100);
}

void GSM_Begin()
{

	while(1)
	{
		LCD_Command(0xc0);
		USART_SendString("ATE0\r"); /* send ATE0 to check module is ready or not */
		_delay_ms(500);
		if(strstr(buff,"OK"))
		{
			GSM_Response();
			memset(buff,0,160);
			break;
		}
		else
		{
			LCD_String("Error");
		}
	}
	_delay_ms(1000);

	LCD_Clear();
	LCD_String_xy(1,0,"Text Mode");
	LCD_Command(0xc0);
	USART_SendString("AT+CMGF=1\r"); /* select message format as text */
	GSM_Response();
	_delay_ms(1000);
}

void GSM_Msg_Delete(unsigned int position)
{
	buffer_pointer=0;
	char delete_cmd[20];

	/* delete message at specified position */
	sprintf(delete_cmd,"AT+CMGD=%d\r",position);
	USART_SendString(delete_cmd);
}

void GSM_Delete_All_Msg()
{
	USART_SendString("AT+CMGDA=\"DEL ALL\"\r"); /* delete all messages of SIM */
}

bool GSM_Wait_for_Msg()
{
	char msg_location[4];
	int i;
	_delay_ms(500);
	buffer_pointer=0;

	while(1)
	{
		/*eliminate "\r \n" which is start of string */

		if(buff[buffer_pointer]=='\r' || buff[buffer_pointer]== '\n'){
			buffer_pointer++;
		}
		else
		break;
	}

	/* "CMTI:" to check if any new message received */
	
	if(strstr(buff,"CMTI:")){
		while(buff[buffer_pointer]!= ',')
		{
			buffer_pointer++;
		}
		buffer_pointer++;
		
		i=0;
		while(buff[buffer_pointer]!= '\r')
		{
			msg_location[i]=buff[buffer_pointer];				      /* copy location of received message where it is stored */
			buffer_pointer++;
			i++;
		}

		/* convert string of position to integer value */
		position = atoi(msg_location);
		
		memset(buff,0,strlen(buff));
		buffer_pointer=0;

		return true;
	}
	else
	{
		return false;
	}
}

/* ISR routine to save responses/new message */
ISR(USART_RXC_vect)
{
	buff[buffer_pointer] = UDR;	/* copy UDR (received value) to buffer */
	buffer_pointer++;
	status_flag = 1;		/* flag for new message arrival */
}


void GSM_Send_Msg(char *num,char *sms)
{
	char sms_buffer[35];
	buffer_pointer=0;
	sprintf(sms_buffer,"AT+CMGS=\"%s\"\r",num);
	USART_SendString(sms_buffer); /*send command AT+CMGS="Mobile No."\r */
	_delay_ms(200);
	while(1)
	{
		if(buff[buffer_pointer]==0x3e) /* wait for '>' character*/
		{
			buffer_pointer = 0;
			memset(buff,0,strlen(buff));
			USART_SendString(sms); /* send msg to given no. */
			USART_TxChar(0x1a); /* send Ctrl+Z */
			break;
		}
		buffer_pointer++;
	}
	_delay_ms(300);
	buffer_pointer = 0;
	memset(buff,0,strlen(buff));
	memset(sms_buffer,0,strlen(sms_buffer));
}

void GSM_Calling(char *Mob_no)
{
	char call[20];
	sprintf(call,"ATD%s;\r",Mob_no);
	USART_SendString(call);	/* send command ATD<Mobile_No>; for calling*/
}

void GSM_HangCall()
{
	LCD_Clear();
	USART_SendString("ATH\r");	/*send command ATH\r to hang call*/
}

void GSM_Response()
{
	unsigned int timeout=0;
	int CRLF_Found=0;
	char CRLF_buff[2];
	int Response_Length=0;
	while(1)
	{
		if(timeout>=60000)								/*if timeout occur then return */
		return;
		Response_Length = strlen(buff);
		if(Response_Length)
		{
			_delay_ms(2);
			timeout++;
			if(Response_Length==strlen(buff))
			{
				for(int i=0;i<Response_Length;i++)
				{
					memmove(CRLF_buff,CRLF_buff+1,1);
					CRLF_buff[1]=buff[i];
					if(strncmp(CRLF_buff,"\r\n",2))
					{
						if(CRLF_Found++==2)				                                    /* search for \r\n in string */
						{
							GSM_Response_Display();
							return;
						}
					}

				}
				CRLF_Found = 0;

			}
			
		}
		_delay_ms(1);
		timeout++;
	}
	status_flag=0;
}

void GSM_Response_Display()
{
	buffer_pointer = 0;
	int lcd_pointer = 0;
	while(1)
	{
		/* search for \r\n in string */
		if(buff[buffer_pointer]== '\r' || buff[buffer_pointer]== '\n')
		{
			buffer_pointer++;
		}
		else
		break;
	}
	

	LCD_Command(0xc0);
	while(buff[buffer_pointer]!='\r')								   /* display response till "\r" */
	{
		LCD_Char(buff[buffer_pointer]);
		buffer_pointer++;
		lcd_pointer++;
		if(lcd_pointer==15)	/* check for end of LCD line */
		LCD_Command(0x80);
	}
	buffer_pointer = 0;
	memset(buff,0,strlen(buff));
}

void GSM_Msg_Read(int position)
{
	char read_cmd[10];
	sprintf(read_cmd,"AT+CMGR=%d\r",position);
	USART_SendString(read_cmd);	/* read message at specified location/position */
	GSM_Msg_Display();	/* display message */
}

void GSM_Msg_Display()
{
	_delay_ms(500);
	if(!(strstr(buff,"+CMGR")))	/*check for +CMGR response */
	{
		LCD_String_xy(1,0,"No message");
	}
	else
	{
		buffer_pointer = 0;
		
		while(1)
		{
			/*wait till \r\n not over*/

			if(buff[buffer_pointer]=='\r' || buff[buffer_pointer]== 'n') 			                {
				buffer_pointer++;
			}
			else
			break;
		}
		
		/* search for 1st ',' to get mobile no.*/
		while(buff[buffer_pointer]!=',')
		{
			buffer_pointer++;
		}
		buffer_pointer = buffer_pointer+2;

		/* extract mobile no. of message sender */
		for(int i=0;i<=12;i++)
		{
			Mobile_no[i] = buff[buffer_pointer];
			buffer_pointer++;
		}
		
		do
		{
			buffer_pointer++;
		}while(buff[buffer_pointer-1]!= '\n');
		
		LCD_Command(0xC0);
		int i=0;

		/* display and save message */
		while(buff[buffer_pointer]!= '\r' && i<31)
		{
			LCD_Char(buff[buffer_pointer]);
			message_received[i]=buff[buffer_pointer];
			
			buffer_pointer++;
			i++;
			if(i==16)
			LCD_Command(0x80); /* display on 1st line */
		}
		
		buffer_pointer = 0;
		memset(buff,0,strlen(buff));
	}
	status_flag = 0;
}
//RAIN FUNctions

void ADC_Init()
{
	DDRA=0x0;			/* Make ADC port as input */
	ADCSRA = 0x87;			/* Enable ADC, fr/128  */
	ADMUX = 0x40;			/* Vref: Avcc, ADC channel: 0 //MAKING AVCC PORT 5V*/
	
}

int ADC_Read(char channel)
{
	int Ain,AinLow;
	
	ADMUX=ADMUX|(channel & 0x0f);	/* Set input channel to read */

	ADCSRA |= (1<<ADSC);		/* Start conversion */
	while((ADCSRA&(1<<ADIF))==0);	/* Monitor end of conversion interrupt */
	
	_delay_us(10);
	AinLow = (int)ADCL;		/* Read lower byte*/
	Ain = (int)ADCH*256;		/* Read higher 2 bits and 
					Multiply with weight */
	Ain = Ain + AinLow;				
	return(Ain);			/* Return digital value*/
}                                                                                                                                     
 
 void RainFun(void)//call me ill get rain status
 {
	 int value;

	 ADC_Init();
	 LCD_Init();			/* Initialization of LCD */
	 LCD_Clear();
	 LCD_String("ADC value");	/* Write string on 1st line of LCD */
	 LCD_Command(0xc4);	/* LCD16x2 cursor position */
	 value=ADC_Read(0);	
	 if(value<300){
		 LCD_String_xy(1,0,"Heavy Rain");
		 if(value<1000)LCD_String_xy(1,0,"Moderate Rain");
		 rainFlag=1;
	 }
	 else LCD_String_xy(1,0,"No Rain");
	 
	 /* Read ADC channel 0 */
	 //itoa(value,String,10);	/* Integer to string conversion */
	// LCD_String(String);
	// LCD_String("  ");
	 
 }

 void send_msg_MyFunction(void)
 {
	  LCD_Init();
	  USART_Init(9600);	/* initialize USART */
	  sei();
	  LCD_String_xy(1,0,"GSM Initializing");
	  _delay_ms(3000);
	  LCD_Clear();
	  LCD_String_xy(1,0,"AT");
	  GSM_Begin();		/* Start GSM with AT*/
	  LCD_Clear();
	  LCD_String_xy(1,0,"Sending Warning..");
	
	  char msg[35]="Soil Moisture Warning Please Attain";
	  GSM_Send_Msg(Mobile_no_User,msg);
	  _delay_ms(3000);
	    LCD_Clear();
 }
 
 void solenoid_valve_on(void)
 {
	 DDRA = DDRA | (1<<1); //MAKING PIN1 OUTPUT
	 DDRA = DDRA & (~(1<<2));//MAKING PIN2 INPUT  (switch)
	 int pin_status = PINA & (1<<2);	/*Read status of pin PA2 */
	 if(pin_status)			/* Transmit status of pin PA2 on to pin PA1 to drive LED. */
	 {
		 PORTA = PORTA | (1<<1);	/* Switch is open, pin_status = 1, LED is ON */
	 }
	 else
	 {
		 PORTA = PORTA | ~(1<<1);	/* Switch is closed, pin_status = 0, LED is OFF */
	 }

 }
  void uart_init()
  {
	  uart_pin_output();
  }
  /* ------------------------------------------------------------------------- */
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
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
int nrf_Receive(void)
 {
	  /* init the software uart */
	  uart_init();
	  /* init the xprintf library */
	   LCD_Init();
	  /* simple greeting message */
	   LCD_String_xy(1,0,"Rx device is ready..");
	  /* init hardware pins */
	  nrf24_init();	  
	  /* Channel #2 , payload length: 4 */
	  nrf24_config(2,4);	  
	  /* Set the device addresses */
	  nrf24_tx_address(tx_address);
	  nrf24_rx_address(rx_address);
	  
		  if(nrf24_dataReady())
		  {
			  nrf24_getData(SendArray);
			  
		  }
	  return (int)SendArray;
 }

 /* ------------------------------------------------------------------------- */
 
 /* ------------------------------------------------------------------------- */