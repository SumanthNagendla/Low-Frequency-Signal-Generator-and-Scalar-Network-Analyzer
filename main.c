// Sumanth Nagendla
//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------
// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz
// Hardware configuration:
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1
//SSI Interface:
//   MOSI (SSI2Tx) on PB7
//   SCLK (SSI2Clk) on PB4
//Analog Interface:
//   AN0/PE3 is driven by the raw value from slow integrator
//   AN0/PE2 is driven by the raw value from fast integrator
//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "C:\ti\TivaWare_C_Series-2.1.3.156\inc\hw_memmap.h"
#include "C:\ti\TivaWare_C_Series-2.1.3.156\inc\hw_types.h"
#include "C:\ti\TivaWare_C_Series-2.1.3.156\driverlib\sysctl.h"
#include "C:\ti\TivaWare_C_Series-2.1.3.156\driverlib\interrupt.h"
#include "C:\ti\TivaWare_C_Series-2.1.3.156\driverlib\gpio.h"
#include "C:\ti\TivaWare_C_Series-2.1.3.156\driverlib\timer.h"


#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define PUSH_BUTTON  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))
#define CS_NOT       (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 1*4)))
#define A0           (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 6*4)))
#define STRING_LENGTH 50
#define DC_MODE 1
#define SINE_MODE 2
#define VOLTAGE_SINE_MODE 3
#define VOLTAGE_DC_MODE 4
#define SWEEP_MODE 5
#define PI 3.1415
#define SAMPLESIZE 15
#define SQUARE_MODE 6
#define SAWTOOTH_MODE 7
#define SQUAREDUTY_MODE 8
#define SQUAREPMODE 10



//Global Variables
uint8_t KeywordPosition = 0;
uint8_t Keywordlength = 1;
uint8_t FirstparameterPositon = 0;
uint8_t FirstParameterLength = 1;
uint8_t SecondparameterPosition = 0;
uint8_t SecondparameterLength = 1;
uint8_t NumberofParameters = 0;
float frequency; //multiplesofkhz
float firstsweepfrequency;
float secondsweepfrequency;
float voltage;
float amplitude;
uint32_t mode = 9;
char MainString[STRING_LENGTH];
char Str4[10];
uint32_t Table[4096];
bool negative;
uint32_t time = 0;
uint32_t Accumulator = 0;
uint32_t phase = 0;
float SineVoltage =0;
uint8_t DCVoltage =0;
float RawValue = 0;
float dutycycle = 50;
bool squareterm = 1;
uint32_t squarevol;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------
// Approximate busy waiting (in units of Milliseconds), given a 40 MHz system clock
// K is the number of times the loop may be called (500 as it is 500 ms)
// N is the number of times the inner loop must be called to achieve 1ms delay
void waitMillisecond(uint32_t ms)
{
	__asm("             PUSH {R1}");         // 1
	__asm("LOOP1:       MOV R1, #6666");     // K
    __asm("LOOP2:       SUB R1, #1");        // K*N
    __asm("             NOP");               // k*N
    __asm("             NOP");               // K*N
    __asm("             CBZ R1, DONE");      // K*(N-1 + 3)
    __asm("             B    LOOP2");        // k*((N-1)*2)
    __asm("DONE:        SUB R0, #1");        // K
    __asm("             CBZ R0, EXIT");      // (K-1)+3
    __asm("             B    LOOP1");        // (K-1)
    __asm("EXIT:        POP {R1}");          // 1
                                             // Sum is (3+k(6N+4) = 40000*500 => N= 6666 , when K = 500(500 ms))

}
void SetDACOutput(int data)
{
    SSI2_DR_R = data;                  // write data
    //while (SSI2_SR_R & SSI_SR_BSY);    // wait for transmission to stop
}
// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = c;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
    putcUart0(str[i]);
}
// Frequency counter service publishing latest frequency measurements every second
void Timer1Isr()
{
	Accumulator = Accumulator + phase;
	SetDACOutput(Table[Accumulator>>20]);
	TIMER1_ICR_R = TIMER_ICR_TATOCINT;              // clear interrupt flag
}

// Frequency counter service publishing latest frequency measurements every second
void Timer2Isr()
{

	 if(squareterm == 1)
	 {
	   squarevol = round(0x3000+2047+(amplitude/5)*2048);
	 }
	 else
	 {
		squarevol = round(0x3000+2048-(amplitude/5)*2048);
	 }
	 squareterm ^= 1;
	 SetDACOutput(squarevol);
	 TIMER2_ICR_R = TIMER_ICR_TATOCINT;              // clear interrupt flag
}

// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port A and F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOE;

    // Configure LED and pushbutton pins
    GPIO_PORTF_DIR_R |= 0x08;  // make bit 1 an outputs
    GPIO_PORTF_DR2R_R |= 0x08; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R |= 0x08;  // enable LED

    // Configure UART0 pins
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

    // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

    // Configure SSI2 pins for SPI configuration
    SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R2;           // turn-on SSI2 clocking
    GPIO_PORTB_DIR_R |= 0x90;                        // make bits 4 and 7 outputs
    GPIO_PORTB_DR2R_R |= 0xB0;                        // set drive strength to 2mA
    GPIO_PORTB_AFSEL_R |= 0xB0;                      // select alternative functions for MOSI, SCLK pins
    GPIO_PORTB_PCTL_R = GPIO_PCTL_PB7_SSI2TX | GPIO_PCTL_PB4_SSI2CLK | GPIO_PCTL_PB5_SSI2FSS; // map alt fns to SSI2
    GPIO_PORTB_DEN_R |= 0xB0;                        // enable digital operation on TX, CLK pins
    GPIO_PORTB_PUR_R |= 0x10;                        // must be enabled when SPO=1

    // Configure the SSI2 as a SPI master, mode 3, 8bit operation, 100 KHz bit rate
    SSI2_CR1_R &= ~SSI_CR1_SSE;                      // turn off SSI2 to allow re-configuration
    SSI2_CR1_R = 0;                                  // select master mode
    SSI2_CC_R = 0;                                   // select system clock as the clock source
    SSI2_CPSR_R = 20;                                // set bit rate to 1 MHz (if SR=0 in CR0)
    SSI2_CR0_R = SSI_CR0_SPH | SSI_CR0_SPO | SSI_CR0_FRF_MOTO | SSI_CR0_DSS_16; // set SR=0, mode 3 (SPH=1, SPO=1), 8-bit
    SSI2_CR1_R |= SSI_CR1_SSE;                       // turn on SSI2

    // Configure Timer 1 as the time base
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;       // turn-on timer
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER1_TAILR_R = 0x190;                          // set load value to 40e6 for 100k Hz interrupt rate
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)

    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R2;       // turn-on timer
	TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
	TIMER2_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
	TIMER2_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
	TIMER2_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
	TIMER2_TAILR_R =  0x190;                         // set load value as per square wave frequency
	TIMER2_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
	NVIC_EN0_R |= 1 << (INT_TIMER2A-16);             // turn-on interrupt 37 (TIMER1A)}

}

//Read ADC Value across the pin
int16_t readAdc0Ss3()
{
    if(frequency>1000)
    {
    	waitMillisecond(3000);
    }
    else
    {
    	waitMillisecond(25000);
    }
	ADC0_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    return ADC0_SSFIFO3_R;                           // get single result from the FIFO
}

//To configureTimer for square wave
void SquareTimer()
{
	SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R2;       // turn-on timer
	TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
	TIMER2_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
	TIMER2_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
	TIMER2_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
	TIMER2_TAILR_R = round(20000000/frequency);      // set load value as per square wave frequency
	TIMER2_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
	NVIC_EN0_R |= 1 << (INT_TIMER2A-16);             // turn-on interrupt 37 (TIMER1A)}
	TIMER2_CTL_R |= TIMER_CTL_TAEN;
}

//Custom function which prints the string of defined length including null characters
void printString(char* str)
{
    uint8_t i;
    for (i = 0; i < STRING_LENGTH; i++)
    putcUart0(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);
    return UART0_DR_R & 0xFF;
}

//Step2 to receive a input string and return for processing
void GetInputString(char *str)
{
    char *InputString;
    InputString = str;
    uint8_t count = 0;
    // Display greeting
    putsUart0("\r\nEnter the Command:");
    // For each character received the string count increases and the string ends for return
    while(count<STRING_LENGTH)
    {
        char c = getcUart0();
        if(c=='\r')
        {
        	if(count==0)
        	{

        	}
        	else
            break;
        }
        else if(c == 8)
        {
        	if(count > 0)
        	{
        		count--;
        		InputString[count]=NULL;

        	}
        	else if(count==0)
        	{
        		InputString[count]=NULL;
        	}
        	else
        	{

        	}

        }
        else if (c>=' ')
        {
            InputString[count]=c;
            count++;
        }
        else;
    }
    InputString[count]=NULL;
    //putsUart0("\r\n The String entered is: ");
    //putsUart0(InputString);
}

//To check if the character is an alphabet
bool isitalphabet(char c)
{
    bool a=0;
    if(c>='a' && c<='z')
    {
        a=1;
    }
    else if(c>='A' && c<='Z')
    {
        a=1;
    }
    else;
    return a;
}

//To check if the character is a number
bool isitnumber(char c)
{
    bool a=0;
    if(c>='0' && c<='9')
    {
        a=1;
    }
    else;
    return a;
}

//Function which converts an Integer to string (Courtesy: Stack over flow modified)
void itos(int i,char *b)
{
    char const digit[] = "0123456789";
    char *p;
    p = b;
    int value = 0;
    int shifter = i;
    do
    {
        //Move to where representation ends
        ++value;
        shifter = shifter/10;
    }
    while(shifter);
    p[value] = NULL;
    do
    {
        //Move back, inserting digits as u go
        value--;
        p[value] = digit[i%10];
        i = i/10;
    }
    while(i);
}

//To find out the command is a valid command
bool iscommand(char * a,int l)
{
    uint8_t loop = 0;
    uint8_t k;
    memset(Str4,NULL,10);
    for(loop=0;loop<Keywordlength;loop++)
    {
        Str4[loop]=MainString[KeywordPosition+loop];
    }
    Str4[Keywordlength]=NULL;
    k = strcmp(Str4,a);
    if(k==0 && l==(NumberofParameters-1))
    return 1;
    else
    return 0;
}

int ParsetheInputString()
{

    GetInputString(MainString);
    char Str1[5];
    memset(Str1,NULL,5);
    char Str2[5];
    memset(Str2,NULL,5);
    char Str3[5];
    memset(Str3,NULL,5);
    uint8_t maincount = 0;
    while(maincount<STRING_LENGTH)
    {
        if(isitalphabet(MainString[maincount]))
        {
            if(maincount>0)
            {
                if(isitalphabet(MainString[maincount-1]));
                else
                {
                    if(NumberofParameters==1)
                    {
                        FirstparameterPositon=maincount;
                    }
                    else if(NumberofParameters==2)
                    {
                        SecondparameterPosition=maincount;
                    }
                	NumberofParameters++;

                }
            }
            else if(maincount==0)
            {
                NumberofParameters++;
                KeywordPosition = maincount;
            }
            else;
        }
        else if(isitnumber(MainString[maincount]))
        {
            if((isitnumber(MainString[maincount-1])) || ((MainString[maincount-1]=='.' ) && (isitnumber(MainString[maincount-2]))));
            else
            {
                if(NumberofParameters==1)
                {
                    FirstparameterPositon=maincount;
                }
                else if(NumberofParameters==2)
                {
                    SecondparameterPosition=maincount;
                }
                else;
                if(isitalphabet(MainString[maincount-1]))
                {
                    Keywordlength = maincount - KeywordPosition;
                }
                NumberofParameters++;
            }
        }
        else
        {
            if((MainString[maincount]=='.') && (isitnumber(MainString[maincount-1])) && (isitnumber(MainString[maincount+1])))
            {
            }
            else if(MainString[maincount]=='-' && (isitnumber(MainString[maincount+1])))
            {
            	negative=1;
            }
            else
            {
                MainString[maincount]=NULL;
            }
            if(isitalphabet(MainString[maincount-1]))
            {

                if(NumberofParameters == 1)
                {
                	Keywordlength = maincount - KeywordPosition;
                }
                else if(NumberofParameters == 2)
                {
                    FirstParameterLength = maincount - FirstparameterPositon;
                }
                else if(NumberofParameters == 3)
                {
                    SecondparameterLength = maincount - SecondparameterPosition;
                }
            }
            else if((isitnumber(MainString[maincount-1])&&(MainString[maincount]!='.'))||(isitnumber(MainString[maincount-1])&&(MainString[maincount]=='.')&&(MainString[maincount+1]=='.')))
            {
                if(NumberofParameters == 2)
                {
                    FirstParameterLength = maincount - FirstparameterPositon;
                }
                else if(NumberofParameters == 3)
                {
                    SecondparameterLength = maincount - SecondparameterPosition;
                }
                else;
            }
        }
        maincount++;
    }
    /* Prints for Debugging purpose
    //putsUart0("\r\nNumber of Fields:");
    //putcUart0(NumberofParameters+'0');
    //putsUart0("\r\nString is:");
    //printString(MainString);
    itos(KeywordPosition,Str1);
    itos(FirstparameterPositon,Str2);
    itos(SecondparameterPosition,Str3);
    if(NumberofParameters==1)
    {
        //putsUart0("\r\nThe keyword starting bit is: ");
        //putsUart0(Str1);
        //putsUart0(" Length: ");
        //putcUart0(Keywordlength+'0');
    }
    else if(NumberofParameters==2)
    {
        //putsUart0("\r\nThe keyword position is: ");
        //putsUart0(Str1);
        //putsUart0(" Length: ");
        //putcUart0(Keywordlength+'0');
        //putsUart0("\r\nThe first parameter position is: ");
        //putsUart0(Str2);
        //putsUart0(" Length: ");
        //putcUart0(FirstParameterLength+'0');
    }
    else if(NumberofParameters==3)
    {
        //putsUart0("\r\nThe keyword position is: ");
        //putsUart0(Str1);
        //putsUart0(" Length: ");
        //putcUart0(Keywordlength+'0');
        //putsUart0("\r\nThe first parameter position is: ");
        //putsUart0(Str2);
        //putsUart0(" Length: ");
        //putcUart0(FirstParameterLength+'0');
        //putsUart0("\r\nThe Second parameter position is: ");
        //putsUart0(Str3);
        //putsUart0(" Length: ");
        //putcUart0(SecondparameterLength+'0');
    }*/
    return 0;
}

//Function whic extracts the value from the command parameters
float getnumber(int parameternumber)
{
    uint8_t loop = 0;
	char numberstring[10];
    float value = 0.0;
    memset(numberstring,NULL,10);
    if(parameternumber==1)
    {
        for(loop=0;loop<FirstParameterLength;loop++)
        {
            numberstring[loop]=MainString[FirstparameterPositon+loop];
        }
        numberstring[FirstParameterLength ]=NULL;
        value = atof(numberstring);
    }
    else if(parameternumber==2)
    {
        for(loop=0;loop<SecondparameterLength;loop++)
        {
            numberstring[loop]=MainString[SecondparameterPosition+loop];
        }
        numberstring[SecondparameterLength  ]=NULL;
        value = atof(numberstring);
    }
    else
    {
        //putsUart0("\r\nWrong Parameter number");
    }
    return value;
}

//To process the command and set the mode of operation
void ProcessCommand()
{
	char Teststring[10];
	char commandstring[10];
	uint8_t loop =0;
    if(iscommand("sine",2))
    {
        //putsUart0("Its the Sine Era )");
    	if(isitnumber(MainString[FirstparameterPositon]) && isitnumber(MainString[SecondparameterPosition]))
    	{
    		frequency = getnumber(1);
    		amplitude = getnumber(2);
    		mode = SINE_MODE;
    	}
    	else
    	{
    		mode = 9;
    	}

    }
    else if(iscommand("dc",1))
    {

    	if(isitnumber(MainString[FirstparameterPositon]))
    	{
    	  //putsUart0("Its the DC Era )");
          voltage= getnumber(1);
          mode = DC_MODE;
    	}
    	else
    	{
    	    mode = 9;
    	}
    }
    else if(iscommand("reset",0))
    {
    	__asm("    .global _c_int00\n"
    	          "    b.w     _c_int00");
    }
    else if(iscommand("square",2))
    {
    	 //putsUart0("Its the Sine Era )");
    	    	if(isitnumber(MainString[FirstparameterPositon]) && isitnumber(MainString[SecondparameterPosition]))
    	    	{
    	    		frequency = getnumber(1);
    	    		amplitude = getnumber(2);
    	    		mode = SQUARE_MODE;
    	    	}
    	    	else
    	    	{
    	    		mode = 9;
    	    	}

    }
    else if(iscommand("squarep",2))
    {
    	 //putsUart0("Its the Sine Era )");
    	    	if(isitnumber(MainString[FirstparameterPositon]) && isitnumber(MainString[SecondparameterPosition]))
    	    	{
    	    		frequency = getnumber(1);
    	    		amplitude = getnumber(2);
    	    		mode = SQUAREPMODE;
    	    	}
    	    	else
    	    	{
    	    		mode = 9;
    	    	}

    }

    else if(iscommand("square",1))
       {
       	 //putsUart0("Its the Sine Era )");
       	    	if(isitnumber(MainString[FirstparameterPositon]))
       	    	{
       	    		dutycycle = getnumber(1);
       	    		mode = SQUAREDUTY_MODE;
       	    	}
       	    	else
       	    	{
       	    		mode = 9;
       	    	}

       }
    else if(iscommand("sawtooth",2))
        {
        	 //putsUart0("Its the Sine Era )");
        	    	if(isitnumber(MainString[FirstparameterPositon]) && isitnumber(MainString[SecondparameterPosition]))
        	    	{
        	    		frequency = getnumber(1);
        	    		amplitude = getnumber(2);
        	    		mode = SAWTOOTH_MODE;
        	    	}
        	    	else
        	    	{
        	    		mode = 9;
        	    	}

        }
    else if(iscommand("voltage",2))
    {
    	memset(Teststring,NULL,10);
    	memset(commandstring,NULL,10);
    	if(isitalphabet(MainString[FirstparameterPositon]) && isitalphabet(MainString[SecondparameterPosition]))
    	{
    		for(loop=0;loop<FirstParameterLength;loop++)
    		{
    			Teststring[loop]=MainString[FirstparameterPositon+loop];
    		}
    		Teststring[loop+1]=NULL;
    		for(loop=0;loop<SecondparameterLength;loop++)
    		{
    			commandstring[loop]=MainString[SecondparameterPosition+loop];
    		}
    		commandstring[loop+1]=NULL;
    	   if((strcmp(Teststring,"test")==0)&&(strcmp(commandstring,"sine")==0))
    	   {
    		   mode = VOLTAGE_SINE_MODE;
    	   }
    	   else if((strcmp(Teststring,"test")==0)&&(strcmp(commandstring,"dc")==0))
		   {
    		   mode = VOLTAGE_DC_MODE;
		   }
    	   else
    	   {
    		   mode = 9;
    	   }
    	}
    	else
    	{
    		mode = 9;
    	}
    }
    else if(iscommand("sweep",2))
    {
        if(isitnumber(MainString[FirstparameterPositon]) && isitnumber(MainString[SecondparameterPosition]))
    	{
        	firstsweepfrequency = getnumber(1);
        	secondsweepfrequency = getnumber(2);
    		mode = SWEEP_MODE;
    	}
    	else
    	{
    		mode = 9;
    	}

    }
    else
    {
        //putsUart0("\r\nThe Keyword is:");
        //putsUart0(Str4);
        //putsUart0("\r\nThe number of parameters are:");
        //putcUart0(NumberofParameters+'0');
    }
}


//To generate the lookup Table for sine wave
void GenerateSineLookUpTable()
{
	uint16_t i;
	for(i=0;i<4096;i++)
	{
		Table[i]=0x3000+2048-(amplitude/5)*2048*sin(i*2*PI/4096);
	}
}

//To generate the lookup table for square wave
void GenerateSquareLookUpTable()
{
	uint16_t i;
	for(i=0;i<2048;i++)
	{
		Table[i]=round(0x3000+2048-(amplitude/5)*2048);
	}
	for(i=2048;i<4096;i++)
	{
		Table[i]=round(0x3000+2047+(amplitude/5)*2048);
	}
}

//To generate the lookup table for square wave as per dutycycle
void GenerateSquareDutyLookUpTable()
{
	uint16_t i;
    uint16_t k;
    k=4096*dutycycle/100;
	for(i=0;i<4096-k;i++)
	{
		Table[i]=round(0x3000+2047+(amplitude/5)*2048);
	}
	for(i=4096-k;i<4096;i++)
	{
		Table[i]=round(0x3000+2048-(amplitude/5)*2048);

	}
}

//To generate the lookup table for sawtooth wave
void GenerateSawtoothLookUpTable()
{
	uint16_t i;
	for(i=0;i<4096;i++)
	{
		Table[i]=0x3000+2048-(amplitude/5)*i;//*2048*i/4096;
	}
}

//To Simulate the sine wave
void SimulateSineOutput()
{
	phase = frequency*pow(2,32)/100000;
	GenerateSineLookUpTable();
	    if(amplitude <= 5 && negative==0)
	    {
	    TIMER2_CTL_R &= ~TIMER_CTL_TAEN;
		TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer;
	    }
	    else if(negative==1)
	    {
	      putsUart0("\r\n Negative Freq and Amplitude are not allowed");
	    }
	    else
	    {
	    	putsUart0("\r\n Sine Voltage is out of limit");

	    }

}

//To simulate the square wave
void SimulateSquareOutput()
{
	phase = frequency*pow(2,32)/100000;
	GenerateSquareLookUpTable();
	    if(amplitude <= 5 && negative==0)
	    {
	    TIMER2_CTL_R &= ~TIMER_CTL_TAEN;
		TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer;
	    }
	    else if(negative==1)
	    {
	      putsUart0("\r\n Negative Freq and Amplitude are not allowed");
	    }
	    else
	    {
	    	putsUart0("\r\n Square Voltage is out of limit");

	    }

}

//To simulate the Square wave with duty cycle
void SimulateSquareDutyOutput()
{
	phase = frequency*pow(2,32)/100000;
	GenerateSquareDutyLookUpTable();
}

//To simulate the sawtooth wave form
void SimulateSawtoothOutput()
{
	phase = frequency*pow(2,32)/100000;
	GenerateSawtoothLookUpTable();
	    if(amplitude <= 5 && negative==0)
	    {
	    TIMER2_CTL_R &= ~TIMER_CTL_TAEN;
		TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer;
	    }
	    else if(negative==1)
	    {
	      putsUart0("\r\n Negative Freq and Amplitude are not allowed");
	    }
	    else
	    {
	    	putsUart0("\r\n SawTooth Voltage is out of limit");

	    }

}

//A custom function for sweep command sine wave generation
void SweepSineSimulate()
{
	phase = frequency*pow(2,32)/100000;
	GenerateSineLookUpTable();
}

//To generate a DC Voltage
void SimulateDcOutput()
{
    uint32_t DACVoltage;
    char Str6[10];
    memset(Str6,NULL,10);
    if(negative == 1)
    {
    	DACVoltage = round(0x3000+2047+(voltage/5)*2048);
    }
    else
    {
    	DACVoltage = round(0x3000+2048-(voltage/5)*2048);
    }

    itos(DACVoltage,Str6);
    //putsUart0("\r\nVoltage is");
    //putsUart0(Str6);
    if(voltage <= 5)
    {
    	SetDACOutput(DACVoltage);
    }
    else
    {
    	putsUart0("\r\nDC Voltage is out of limit");

    }

}

//A function which extracts characters from string and aligns to number (Courtesy: Stack over flow modified)
int n_tu(int number, int count)
{
    int result=1;
    while(count-- > 0)
    result *= number;

    return result;
}

//Function which converts an Integer to string (Courtesy: Stack over flow modified)
void float_to_string(float f, char r[])
{
    long long int length, length2, i, number, position, sign;
    float number2;

    sign = -1;   // -1 == positive number
    if (f < 0)
    {
        sign = '-';
        f *= -1;
    }


    number2 = f;
    number = f;
    length = 0;  // size of decimal part
    length2 = 0; //  size of tenth

    /* calculate length2 tenth part*/
    while( (number2 - (float)number) != 0.0 && !((number2 - (float)number) < 0.0) )
    {
         number2 = f * (n_tu(10.0, length2 + 1));
         number = number2;

         length2++;
    }

    /* calculate length decimal part*/
    for(length = (f >= 1) ? 0 : 1; f >= 1; length++)
        f /= 10;

    position = length;
    length = length + 1 + length2;
    number = number2;
    if(sign == '-')
    {
        length++;
        position++;
    }

    for(i = length; i >= 0 ; i--)
    {
        if(i == (length))
            r[i] = '\0';
        else if(i == (position))
            r[i] = '.';
        else if(sign == '-' && i == 0)
            r[i] = '-';
        else
        {
            r[i] = (number % 10) + '0';
            number /=10;
        }
    }
}

//To Configure ADC based on the frequency
void ConfigureADC()
{
	if(frequency>1000)
	{
	    SYSCTL_RCGCADC_R |= 1;                           // turn on ADC module 0 clocking
		GPIO_PORTE_AFSEL_R |= 0x04;                      // select alternative functions for AN0 (PE3)
		GPIO_PORTE_DEN_R &= ~0x04;                       // turn off digital operation on pin PE3
		GPIO_PORTE_AMSEL_R |= 0x04;                      // turn on analog operation on pin PE3
		ADC0_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
		ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
		ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
		ADC0_SSMUX3_R = 1;                               // set first sample to AN0
		ADC0_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end
		ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation
	}
	else
	{
	    SYSCTL_RCGCADC_R |= 1;                           // turn on ADC module 0 clocking
		GPIO_PORTE_AFSEL_R |= 0x08;                      // select alternative functions for AN0 (PE2)
		GPIO_PORTE_DEN_R &= ~0x08;                       // turn off digital operation on pin PE2
		GPIO_PORTE_AMSEL_R |= 0x08;                      // turn on analog operation on pin PE2
		ADC0_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
		ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
		ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
		ADC0_SSMUX3_R = 0;                               // set first sample to AN0
		ADC0_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end
		ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation
	}
}

//To Measure Sine voltage
void MeasureSineVoltage()
{
	ConfigureADC();
	char sinevalueout[50];
	char frequencyout[50];
	SineVoltage =0;
	RawValue = readAdc0Ss3();
	SineVoltage = (RawValue+0.5)*5/4096;
	float_to_string(SineVoltage,sinevalueout);
	float_to_string(frequency,frequencyout);
	putsUart0("\r\nThe amplitude of the Sine wave is: ");
    //putsUart0(frequencyout);
	//putsUart0(" is: ");
	putsUart0(sinevalueout);
}

//To Measure Sweep Sine Voltage
void MeasureSweepSineVoltage()
{
	ConfigureADC();
	char sinevalueout[50];
	char frequencyout[50];
	SineVoltage =0;
	RawValue = readAdc0Ss3();
	waitMillisecond(2000);
	SineVoltage = (RawValue+0.5)*5/4096;
	float_to_string(SineVoltage,sinevalueout);
	float_to_string(frequency,frequencyout);
	putsUart0("\r\nThe amplitude of the Sine wave for the frequency ");
    putsUart0(frequencyout);
	putsUart0(" is: ");
	putsUart0(sinevalueout);
}

//To sweep from one frequency to other frequency
void SweepFrequency()
{
	uint16_t factor;
	uint8_t iter = 0;
	if(firstsweepfrequency>secondsweepfrequency)
	{
		factor = (uint16_t)((firstsweepfrequency-secondsweepfrequency)/SAMPLESIZE);
		frequency = secondsweepfrequency;
		for(iter=0;iter<SAMPLESIZE;iter++)
		{
			frequency = frequency+factor;
			SweepSineSimulate();
			waitMillisecond(1);
			MeasureSweepSineVoltage();
		}
	}
	else
	{
		factor = (uint16_t)((secondsweepfrequency-firstsweepfrequency)/SAMPLESIZE);
		frequency = firstsweepfrequency;
		for(iter=0;iter<SAMPLESIZE;iter++)
		{
			frequency = frequency+factor;
			SimulateSineOutput();
			waitMillisecond(1);
			MeasureSweepSineVoltage();
		}
	}
}

//To measure the DC Voltage
void MeasureDCVoltage()
{
	float RawValue = 0;
	DCVoltage =0;
	waitMillisecond(5000);
	RawValue = readAdc0Ss3();
	DCVoltage = (uint8_t)(RawValue+0.5)*3.3/4096;
}

//To Flash led thrice initially with a 300ms delay
void FlashLed()
{
	uint8_t i;
	for(i=0;i<6;i++)
	{
		waitMillisecond(300);
		GREEN_LED ^= 1;

	}

}

//Process the mode and call the respective operation
void ProcessMode()
{
    switch(mode)
    {
        case DC_MODE:
        putsUart0("\r\nDC Mode of Operation");
        TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
        TIMER2_CTL_R &= ~TIMER_CTL_TAEN;
        SimulateDcOutput();
        break;
        case SINE_MODE:
        putsUart0("\r\nSINE Mode of Operation");
        SimulateSineOutput();
        break;
        case VOLTAGE_SINE_MODE:
        putsUart0("\r\nVoltage Sine Mode of Operation");
        MeasureSineVoltage();
        break;
        case VOLTAGE_DC_MODE:
        MeasureDCVoltage();
        putsUart0("\r\nVolatge DC Mode of Operation");
        break;
        case SWEEP_MODE:
        putsUart0("\r\nSweep Mode of Operation");
        SweepFrequency();
        break;
        case SQUARE_MODE:
        putsUart0("\r\nSquare Mode of Operation");
        SimulateSquareOutput();
        break;
        case SAWTOOTH_MODE:
        putsUart0("\r\nSawtooth Mode of Operation");
        SimulateSawtoothOutput();
        break;
        case SQUAREDUTY_MODE:
        putsUart0("\r\nSquare duty Mode of Operation");
        SimulateSquareDutyOutput();
        break;
        case SQUAREPMODE:
        putsUart0("\r\nSquare wave precision  Mode of Operation");
        SquareTimer();
        break;
        default:
        TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
        TIMER2_CTL_R &= ~TIMER_CTL_TAEN;
        putsUart0("\r\n***Wrong pattern entered***");
        break;
    }
}



//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------
int main(void)
{
    // Initialize hardware
    initHw();
    FlashLed();
    putsUart0("\r\n------------------Low Frequency Signal Generator and Scalar Network Analyzer------------------------\r\n");
    putsUart0("\r\nCommand Help:");
    putsUart0("\r\n1.sine freq amp (Generates a sine wave of specified Amplitude and Frequency)");
    putsUart0("\r\n2.dc voltage (Generates a specified voltage)");
    putsUart0("\r\n3.voltage test command (Tests and returns the voltage of wave generated by command)");
    putsUart0("\r\n4.sweep freq1 freq2(Sweeps sine wave generated from one freq1 to freq2)");
    putsUart0("\r\n5.reset(resets the controller)");
    putsUart0("\r\n6.square freq amp (Generates a square wave of specified Amplitude and Frequency)");
    putsUart0("\r\n7.sawtooth freq amp (Generates a sawtooth wave of specified Amplitude and Frequency)");
    putsUart0("\r\n8.Square dutycycle (Modifies the existing Square wave duty cycle)");
    putsUart0("\r\n9.Squarep freq amp (Generates a square wave of specified Amplitude and Frequency in P Mode)");
    while(1)
    {
    	memset(MainString,NULL,STRING_LENGTH); //Initializing the input string each time
    	KeywordPosition = 0;
    	Keywordlength = 1;
    	FirstparameterPositon = 0;
    	FirstParameterLength = 1;
    	SecondparameterPosition = 0;
    	SecondparameterLength = 1;
    	NumberofParameters = 0;
    	mode = 9;
    	putsUart0("\r\n------------------------------------------------------------------------------------------------\r\n");
    	ParsetheInputString();
		ProcessCommand();
		ProcessMode();
    }

}
