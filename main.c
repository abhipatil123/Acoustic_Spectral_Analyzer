/*
 * fft1.c
 *
 * Created: 12/1/2015 3:48:04 PM
 * Author : Gayatri Mestry
 * Refernce to leveraged code in the function headers
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h> 

#define F_CPU 16000000UL          // Frequency of oscillator
#include <util/delay.h>
#define BAUD 9600             	  //baud rate
volatile double buff1[32],getmax; //buffer for FFT
void fft(int n,int m);

#define FFT_SIZE	32   		// 32 point FFT implementation
#define log2FFT		5			//2^5 = 32
#define NUM			(2*FFT_SIZE)
#define log2N      (log2FFT + 1)

#define UBRR_Value ((F_CPU/(16UL*BAUD))-1)     //UBRR value

#define LED_CTRL_PORT	PORTB     //For LED working
#define LED_CTRL_DIR	DDRB
#define LED_ON			(1 << 3)
#define UART_CTRL_PORT	PORTD     //Port for USART
#define UART_CTRL_DIR	DDRD
#define UART_BEGIN		(1 << 0|1 << 1)

#define DATA_PORT	PORTC           //LCD Data port
#define DATA_DIR	DDRC
#define DATA_PIN	PINC

#define CTRL_PORT	PORTD          //LCD Control signal port
#define CTRL_DIR		DDRD

#define DI			(1 << 6)     //Data/Instruction
#define RW			(1 << 5)	 // Read/Write
#define EN			(1 << 7)     //Enable
#define RES			(1 << 4)     //Reset
#define CS1			(1 << 3)     //Chip Select1
#define CS2			(1 << 2)     //Chip select2

#define SCREEN_WIDTH	128      //no of columns
#define SCREEN_HEIGHT	64       //number of sections of pages

//Commands for GLCD
#define DISPLAY_SET_Y       0x40   
#define DISPLAY_SET_X       0xB8
#define DISPLAY_START_LINE  0xC0
#define DISPLAY_ON_CMD		0x3E

#define ON	0x01
#define OFF	0x00
#define DISPLAY_STATUS_BUSY	0x80
unsigned char screen_x;
unsigned char screen_y;
unsigned int show = 0,sample = 0,j=0;

/*      Function Declarations         */
static int usart_putchar1(char c, FILE *stream);
static FILE print = FDEV_SETUP_STREAM(usart_putchar1, NULL, _FDEV_SETUP_WRITE);

void USART_Init(); // configure USART
void USART_tx(unsigned char send_data); // Transmit
void USART_putstring(char* StringPtr);
unsigned char USART_rx(); // Receive
void GLCD_InitalizePorts(void);
void GLCD_Init();
void Select_page(unsigned char );
void GLCD_Comd(unsigned char );
void GLCD_Data(unsigned char dat);
void GLCD_ClearScreen(void);
void GOTO_XY(unsigned int x,unsigned int y);
void GLCD_Draw_line(unsigned short x,unsigned short y, unsigned short j);
void GLCD_Spectrum(double *value);
void GLCD_Bar_Graph(unsigned short x,unsigned short y, unsigned short j);


unsigned char data;

int main(void)
{
	
	/* Replace with your application code */
	unsigned char data;//pixel;
	double max=0;
	int i;
	double buff2[32];//min = 15;
		LED_CTRL_DIR |= LED_ON;
		LED_CTRL_PORT &= ~LED_ON;
		_delay_ms(2);
		CTRL_PORT |= RES; // RST = 1;             Resetting the AVR
		_delay_ms(5);
		CTRL_PORT &= RES; // RST =   0;
		_delay_ms(5);
		CTRL_PORT |= RES; // RST =    1;
		_delay_ms(5);
		GLCD_Init();              //Initialize GLCD
		_delay_ms(15);
	//USART_Init();
	
	
	ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);   //prescaler --125kHz
	ADMUX |=  (1<<REFS0);    						 //ADC reference to AVcc
	ADMUX |= (1 << ADLAR);							//Left shifting ADC Data register
	
	ADMUX |= (1<<MUX2) | (1<<MUX1) |(1<<MUX0);  	//selecting adc7
	ADCSRA |= (1<<ADATE);      						//free running mode
	
	ADCSRA |= (1<<ADEN); 							 //enable adc

	sei();
	ADCSRA |= (1<<ADSC); 							//start adc conversion

	//data = USART_rx();
	while (1)
	{
			
		for (i=0; i<NUM; i++)
		{
			while(!(ADCSRA & (1<<ADIF)));   	//checking idf value arrived in ADC Data Register High
			buff1[i] = ADCH;					//placing the values of ADCH in a buffer 
			_delay_us(125);                     //sampling at 8000Hz
	
		}
	
		fft(32,5);                             // Performing Fast Fourier Transform
	
		for(j=0;j<15;j++)
		{
			if(buff1[j]>max)                   //finding the maximum value in the buffer array of FFT values
			max = buff1[j];
		}
		
		for(j=0;j<15;j++)
		{
			(buff2[j]) = ((buff1[j])/max)*900;	 //normalizing the array
			//printf("%d %f\n",j,buff2[j]);		
		}
		
		GLCD_Spectrum(buff2);					//Drawing the GLCD spectrum to show all bar graphs
	}
	return 0;
}
/*  void USART_Init()
---This is used for initializing the USART by setting the baud rate through the UBRR value
---The parity bit and the data length of 8 bits is configured and the number of stop bits is 1
---The function also implement to use printf functionality to print values to the terminal   */
void USART_Init()
{
	UBRRH=(1<<7)|(unsigned char)(UBRR_Value>>8);
	UBRRL=(unsigned char)(UBRR_Value);
	UCSRC = (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0);
	UCSRB = (1<<RXEN)|(1<<TXEN);
	UCSRA &= 0x00;
	stdout = &print;
}

/*     void USART_tx(unsigned char send_data)
---In this function the UDRE bit is polled to know if the transmit buffer 
---is empty and ready to send data                          */
void USART_tx(unsigned char send_data)
{
	while(!(UCSRA & (1<<UDRE)));
	UDR = send_data;
}

/*   unsigned char USART_rx()
---The RXC bit in USCRA register is polled to check if the data 
---is arrived in the UDR buffer after reception of a byte    */
unsigned char USART_rx()
{
	while(!(UCSRA & (1<<RXC)));
	return UDR;
}

/*   void USART_putstring(char* StringPtr)
--This function is used to print a string of characters by using the USART_tx function  */
void USART_putstring(char* StringPtr){
	
	while(*StringPtr != 0x00){
		USART_tx(*StringPtr);
	StringPtr++;}
	
}

/* static int usart_putchar1(char c, FILE *stream)  
This is a standard function for using printf()
--Referred to user manual http://www.gnu.org/savannah-checkouts/non-gnu/avr-libc/user-manual/group__avr__stdio.html  */
static int usart_putchar1(char c, FILE *stream)
{
	if (c == '\n') usart_putchar1('\r', stream);
	
	while(!(UCSRA & (1<<UDRE)));
	UDR = c;
	
	return 0;
}

/**********************************************************/
/* fft.c                                                  */
/* (c) Douglas L. Jones                                   */
/* University of Illinois at Urbana-Champaign             */
/* January 19, 1992                                       */
/*                                                        */
/*   fft: in-place radix-2 DIT DFT of a complex input     */
/*                                                        */
/*   input:                                               */
/* n: length of FFT: must be a power of two               */
/* m: n = 2**m                                            */
/*   input/output                                         */
/* x: double array of length n with real part of data     */
/* y: double array of length n with imag part of data     */
/*                                                        */
/*   Permission to copy and use this program is granted   */
/*   under a Creative Commons "Attribution" license       */
/*   http://creativecommons.org/licenses/by/1.0/          */
/**********************************************************/
void fft(int n,int m)
{
	
	int i,j,k,n1,n2;
	double c,s,e,a,t1,t2;
	double y[32]={0.0};


	j = 0; /* bit-reverse */
	n2 = n/2;
	for (i=1; i < n - 1; i++)
	{
		n1 = n2;
		while ( j >= n1 )
		{
			j = j - n1;
			n1 = n1/2;
		}
		j = j + n1;

		if (i < j)
		{
			t1 = buff1[i];
			buff1[i] = buff1[j];
			buff1[j] = t1;
			t1 = y[i];
			y[i] = y[j];
			y[j] = t1;
		}
	}


	n1 = 0; /* FFT */
	n2 = 1;

	for (i=0; i < m; i++)
	{
		n1 = n2;
		n2 = n2 + n2;
		e = -6.283185307179586/n2;
		a = 0.0;

		for (j=0; j < n1; j++)
		{
			c = cos(a);
			s = sin(a);
			a = a + e;

			for (k=j; k < n; k=k+n2)
			{
				t1 = c*buff1[k+n1] - s*y[k+n1];
				t2 = s*buff1[k+n1] + c*y[k+n1];
				buff1[k+n1] = buff1[k] - t1;
				y[k+n1] = y[k] - t2;
				buff1[k] = buff1[k] + t1;
				y[k] = y[k] + t2;
				
			}
		}
	}
	for(k=0;k<128;k++)
	{

		buff1[k]=sqrt(buff1[k]*buff1[k]+y[k]*y[k]);
	}
	
}

/*     void GLCD_InitalizePorts(void)
---The GLCD control signal ports are initialized for output   */
void GLCD_InitalizePorts(void)
{
	CTRL_DIR |= (CS1 | CS2 | DI | RW | EN | RES);
	CTRL_PORT &= ~(CS1 | CS2| DI | RW | EN );
}

/*      void GLCD_Delay(void)
---Delay function to introduce delay for proper functioning of the GLCD  */
void GLCD_Delay(void)
{
	_delay_us(9);
}

/*     void GLCD_Init()
---This is the initialzation function for GLCD 
---The initialization commands are sent to setup the display screen to both the chips 
Reference https://www.pantechsolutions.net/microcontroller-boards/glcd-interfacing-with-8051-slicker */
void GLCD_Init()
{
	int i;
	unsigned char Comd[5]={0xC0,0xB8,0x40,0x3F};//LCD Command list
	GLCD_InitalizePorts();
	Select_page(1);           //send commands to page1
	for(i=0;i<4;i++)
	GLCD_Comd(Comd[i]);
	Select_page(0);           //send commands to page0
	for(i=0;i<4;i++)
	GLCD_Comd(Comd[i]);
	GLCD_ClearScreen();
}

/*  void Select_page(unsigned char Page)      
--This function enables a chip and disables the other to get or set the characters on GLCD  */
void Select_page(unsigned char Page)
{
	if(Page)
	{
		CTRL_PORT &= ~CS1;//CS1=0;       //Page 0 LCD IC1
		CTRL_PORT |= CS2; //CS2=1;
		
	}
	else
	{
		CTRL_PORT &= ~CS2;
		CTRL_PORT |= CS1;
		
	}
}

/*   void GLCD_Comd(unsigned char cmnd)
---This function initializes the GLCD control signal to send a command byte from the ATMega32A to GLCD  */
void GLCD_Comd(unsigned char cmnd)
{
	DATA_DIR = 0xFF;
	DATA_PORT = cmnd;
	CTRL_PORT &= ~(DI);
	CTRL_PORT &= ~(RW);
	CTRL_PORT |= EN;           //Enable high
	GLCD_Delay();
	CTRL_PORT &= ~EN;          //Enable low
}

/*   void GLCD_Data(unsigned char dat)
--- The functions sets the GLCD control signals to send a data byte from ATMega32A to the GLCD */
void GLCD_Data(unsigned char dat)
{
	DATA_DIR = 0xFF;
	DATA_PORT = dat;
	CTRL_PORT |= (DI);
	CTRL_PORT &= ~(RW);
	CTRL_PORT |= EN;
	GLCD_Delay();
	CTRL_PORT &= ~EN;
}

/*    void GLCD_ClearScreen(void)
---The function configures the columns of eac---h page as it iterates on the GLCD screen 
---and writes the dat 0x00 to the screen to clear it                         */
void GLCD_ClearScreen(void)
{
	int Page=0;
	int Column=0;
	
	for (Page = 0; Page < 8; Page++)
	{
		Select_page(1);                        //Display part of image to Page1
		GLCD_Comd(0xB8 | Page);
		GLCD_Comd(0x40);
		
		for (Column = 0; Column < 128; Column++)
		{
			if (Column == 64)
			{
				Select_page(0);                    //Display part of image to Page0
				GLCD_Comd(0xB8 | Page);
				GLCD_Comd(0x40);
			}
			GLCD_Data(0x00);
		}
	}
}    

/*    void GOTO_XY(unsigned int x,unsigned int y)
---This functions takes the parameters as the x and y location on the screen of GLCD display
---and sets he x and y location address to go to that particular location on GLCD           */
void GOTO_XY(unsigned int x,unsigned int y)
{
	unsigned short Col_Data;
	CTRL_PORT &= ~(DI);
	CTRL_PORT &= ~(RW);
	if(y<64) //left section of GLCD
	{
		Col_Data = y;
		Select_page(1);
		GLCD_Comd(0xB8 | x);                    //set x-address
		
		//Display part of image to Page0
		GLCD_Comd(DISPLAY_SET_Y|Col_Data);      //set y address
	}
	else
	{
		Col_Data = y-64; //select the next chip area to display
		Select_page(0);                        //Display part of image to Page1
		GLCD_Comd(0xB8 | x);              
		GLCD_Comd(DISPLAY_SET_Y|Col_Data);   //set Y address
	}
	
}

/*       void GLCD_Bar_Graph(unsigned short x,unsigned short y, unsigned short j) 
---This function uses the  GLCD_Draw_line() function to display a line and increment it 
---depending upon the value of FFT and plots a graph               */
void GLCD_Bar_Graph(unsigned short x,unsigned short y, unsigned short j)
{
	unsigned short xaxis =x;//yaxis=y;
	unsigned short k,l,m,n;
	l=j/8;		             	  //number of all dark columns
	m=j%8;  		              // number of columns to be filled extra to represent FFt value
	xaxis=0;
	for(n=0;n<l;n++)         
	{
		k=7;
		GLCD_Draw_line(xaxis,y,7);
		xaxis++;
	}
	for(n=0;n<m;n++)
	{
		k=m;
		GLCD_Draw_line(xaxis,y,k);
	}
	
}

/*    void GLCD_Draw_line(unsigned short x,unsigned short y, unsigned short j)
---This function takes in the x and y location and the value to be fille in th ecolumns to 
---show the bar graphs properly. The GLCD Font Crator tool is used to get the values                        */
void GLCD_Draw_line(unsigned short x,unsigned short y, unsigned short j)
{
	unsigned short Col_Data=0x00,i;
	for(i=0;i<8;i++)
	{
		switch(j)
		{	
			case 0: Col_Data = 0x01; break;
			case 1: Col_Data = 0x03; break;
			case 2: Col_Data = 0x07; break;
			case 3: Col_Data = 0x0F; break;
			case 4: Col_Data = 0x1F; break;
			case 5: Col_Data = 0x3F; break;
			case 6: Col_Data = 0x7F; break;
			case 7: Col_Data = 0xFF; break;
		}
		
		GOTO_XY(x,(y));
		GLCD_Data(Col_Data);
	y--;
	}
	
}

/*   void GLCD_Spectrum(double *value)
--The function takes in the value of fft buffer as a parameter
---and plots the graph for frequency bins from 1 to 14 on the GLCDvdisplay screen */
void GLCD_Spectrum(double *value)
{
	GLCD_ClearScreen();
	GLCD_Bar_Graph(0,127,value[1]);
	GLCD_Bar_Graph(0,118,value[2]);
	GLCD_Bar_Graph(0,109,value[3]);
	GLCD_Bar_Graph(0,100,value[4]);
	GLCD_Bar_Graph(0,91,value[5]);
	GLCD_Bar_Graph(0,82,value[6]);
	GLCD_Bar_Graph(0,73,value[7]);
	GLCD_Bar_Graph(0,64,value[8]);
	GLCD_Bar_Graph(0,55,value[9]);
	GLCD_Bar_Graph(0,46,value[10]);
	GLCD_Bar_Graph(0,37,value[11]);
	GLCD_Bar_Graph(0,28,value[12]);
	GLCD_Bar_Graph(0,19,value[13]);
	GLCD_Bar_Graph(0,10,value[14]);
    _delay_ms(12);       

}

