// EMBEDDED MICROCONTROLLER SYSTEMS
// FALL 2018
// PROJECT: LCR METER
// ID: 28

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include "tm4c123gh6pm.h"
#include "driverlib/pin_map.h"

//step 5
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define HIGHSIDE_R   (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 2*4))) //PD2
#define MEASURE_LR   (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 3*4))) //PD3
#define MEASURE_C    (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 1*4))) //PE1
#define LOWSIDE_R    (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 2*4))) //PE2
#define INTEGRATE    (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 3*4))) //PE1

uint32_t i, count = 0, count1 = 0, count2 = 0, argc = 0;
uint32_t pos[81];
uint32_t a,validcmd,len;
uint32_t max_char = 80;
uint32_t min_argc=0;
float deltav;              // differential voltage
uint16_t raw1,raw2;        // raw value
float r1,r2;              //variable for voltage cmd
uint64_t cap = 0;                //variable to capture timer
float resistance,res1,res2,res3;
float capacitance,capa1,capa2,capa3;
float inductance,ind1,ind2,ind3;
float esr,esr1,esr2,esr3;
float tow;
char s;                   //switch case
char e;                   //esr switch case
char str[81];
char str1[81];
char type[81];
char c,ch;
char* arg1;char* arg2;
char x[20];              //voltage cmd variable
char z[20];              //time variable in res cmd
char re[20];             //resistance variable
char v1[20];             // DUT 1
char v2[20];             // DUT 2
char cp[20];             // capacitance variable
char in[20];             // inductance variable
float voldiv;            // dut2/dut1
char es[20];             // esr variable



// hardware initialization function
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN
            | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use AP (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port F,A,D,E peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOD | SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOC ;

    // Configure LED
    GPIO_PORTF_DIR_R = 0x0A;  // bit 1 is output
    GPIO_PORTF_DR2R_R = 0x0A; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = 0x0A;  // enable LED

    // Configure Measure pins
    GPIO_PORTD_DIR_R = 0x0c;  // bit 2 & 3 is output
    GPIO_PORTD_DR2R_R = 0x0c; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTD_DEN_R = 0x0c;  // enable PD2,PD3
    GPIO_PORTE_DIR_R = 0x0e;  // bit 1,2 & 3 is output
    GPIO_PORTE_DR2R_R = 0x0e; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTE_DEN_R = 0x0e;  // enable PE1,PE2,PE3

    // Configure UART0 pins
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0; // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3;                         // default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                       // default, added for clarity
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

    // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                 // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                 // use system clock (40 MHz)
    UART0_IBRD_R = 21; // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

    // Configure AN9 as an analog input DUT1
    SYSCTL_RCGCADC_R |= 0x01;                           // turn on ADC module 0 clocking
    GPIO_PORTE_AFSEL_R |= 0x20;                      // select alternative functions for AN0 (PE4)
    GPIO_PORTE_DEN_R &= ~0x20;                       // turn off digital operation on pin PE4
    GPIO_PORTE_AMSEL_R |= 0x20;                      // turn on analog operation on pin PE4
    ADC0_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
    ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
    ADC0_SSMUX3_R = 9;                               // set first sample to AN9
    ADC0_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation

    // Configure AN8 as an analog input DUT2
    SYSCTL_RCGCADC_R |= 0x02;                           // turn on ADC module 0 clocking
    GPIO_PORTE_AFSEL_R |= 0x10;                      // select alternative functions for AN0 (PE5)
    GPIO_PORTE_DEN_R &= ~0x10;                       // turn off digital operation on pin PE5
    GPIO_PORTE_AMSEL_R |= 0x10;                      // turn on analog operation on pin PE5
    ADC1_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
    ADC1_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
    ADC1_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
    ADC1_SSMUX3_R = 8;                               // set first sample to AN8
    ADC1_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end
    ADC1_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation

    // Configure WITIMER
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R5;     // turn-on timer
    WTIMER5_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER5_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER5_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR; // configure for edge time mode, count up
    WTIMER5_CTL_R = TIMER_CTL_TAEVENT_POS;           // measure time from positive edge to positive edge
    //WTIMER5_IMR_R = TIMER_IMR_CAEIM;                 // turn-on interrupts
    WTIMER5_TAV_R = 0;                               // zero counter for first period
    NVIC_EN3_R |= 1 << (INT_WTIMER5A-16-96);         // turn-on interrupt 120 (WTIMER5A)

    // Configure PC7 pin
       SYSCTL_RCGCACMP_R|=0x01;                          // turn on the clock module for comparator
       GPIO_PORTC_DEN_R &= ~0x80;                         // disable the digital function
       //GPIO_PORTC_DIR_R|=0x00;                           // pc7 to input
       //GPIO_PORTC_DR2R_R = 0x00;                         //set drive strength to 2mA (not needed since default configuration -- for clarity)
       GPIO_PORTC_AFSEL_R=0x80;                          // select the analog alternate function
       GPIO_PORTC_AMSEL_R |= 0x80;                       // enable the analog mode
      // GPIO_PORTF_PCTL_R|=GPIO_PCTL_PF0_C0O | GPIO_PCTL_PF1_C1O;             // PF0 output
       COMP_ACREFCTL_R|=0x0000020F;                      // set the voltage to reference 2.469
       //COMP_ACSTAT0_R|=0x00000002;                       // when Cin- > Cin+
       COMP_ACCTL0_R|=0x0000040C;                         //  either edge edge
       COMP_ACINTEN_R|=0x01;                             // enable the interrupt
       COMP_ACRIS_R |=0x01;                               // isr
       /*COMP_ACINTEN_R|=0x01;                             // enable the interrupt
       COMP_ACRIS_R |=0x01;                              // fire the interrupt
       NVIC_EN0_R &=~(1<<(INT_COMP0-16));                // disable the interrupt*/
}




// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF)
        ;
    UART0_DR_R = c;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
        putcUart0(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE)
        ;
    return UART0_DR_R & 0xFF;
}

// the assembly function is in us
void waitmillisec(uint32_t ms)
{
    __asm("WMS_LOOP0:   MOV  R1, #6");
    // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");
    // 6
    __asm("             CBZ  R1, WMS_DONE1");
    // 5+1*3
    __asm("             NOP");
    // 5
    __asm("             NOP");
    // 5
    __asm("             B    WMS_LOOP1");
    // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");
    // 1
    __asm("             CBZ  R0, WMS_DONE0");
    // 1
    __asm("             NOP");
    // 1
    __asm("             B    WMS_LOOP0");
    // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");
    // ---
    // 40 clocks/us + error

}

// flashing red led
void led()
{
    RED_LED = 1;
    waitmillisec(500000);
    RED_LED = 0;
}

// getting string from user step 2
int getstring()
{
    while (1)
    {
        count = 0;
        Jump: c = getcUart0();
              ch=tolower(c);
        if (c == 8)
        {
            if (count > 0)
            {
                count--;
                goto Jump;
            }
            else
            {
                goto Jump;
            }
        }
        if (c == 13)
        {
            str[count] = 0;
            break;
        }
        else
        {
            if ((c >= 32) || (c == 9))
            {
                str[count++] = ch;
                if (count == max_char)
                {
                    str[count + 1] = '\0';
                    break;
                }
                else
                {
                    goto Jump;
                }
            }
            else
            {
                goto Jump;
            }
        }
    }
    len=strlen(str);
    return str;
}


// parsing of string step 3
int parsestring()
{
    for(i=0;i<strlen(str);i++)
    {
        if((str[i] >= 97 && str[i] <= 122)||(str[i]==95))
        {
            if((str[i-1] >= 97 && str[i-1] <= 122)||(str[i-1]==95))
            {
                goto Jump1;
            }
            else
            {
                argc++;
                pos[count1]=i;
                type[count2]='a';
                count1++;
                count2++;
                str1[i]=str[i];
            }
            Jump1:str1[i]=str[i];
        }
        else if((str[i] >= 48) && (str[i] <= 57))
        {
            if((str[i-1]>=48) && (str[i-1]<=57))
            {
                goto Jump1;
            }
            else
            {
                argc++;
                pos[count1]=i;
                type[count2]='n';
                count1++;
                count2++;
                str1[i]=str[i];
            }

        }
        else if((str[i]==32) || (str[i]==42) || (str[i]==43) || (str[i]==44) || (str[i]==45) || (str[i]==46) || (str[i]==47))
        {
            str1[i]='\0';
        }

    }
    /*for(i=0;i<argc;i++)
    {
        putsUart0(str1+pos[i]);
        putsUart0("\n\r");
    }*/
    if((strcmp("set",&str1[pos[0]])==0))                   //setting the arguments for set command
        {
            min_argc=argc-1;
            return min_argc;
        }
    else if((strcmp("voltage",&str1[pos[0]]==0)))         //setting the arguments for voltage command
        {
            min_argc=argc-1;
            return min_argc;
        }
    else if((strcmp("resistor",&str1[pos[0]]==0)))        //setting the arguments for resistor command
        {
            min_argc=argc-1;
            return min_argc;
        }
    else if((strcmp("capacitor",&str1[pos[0]]==0)))        //setting the arguments for resistor command
        {
            min_argc=argc-1;
            return min_argc;
        }
    else if((strcmp("inductor",&str1[pos[0]]==0)))        //setting the arguments for resistor command
        {
            min_argc=argc-1;
            return min_argc;
        }
    else if((strcmp("esr",&str1[pos[0]]==0)))        //setting the arguments for resistor command
        {
            min_argc=argc-1;
            return min_argc;
        }
}


char* getvalue(int y)    // THIS LOOP GETS CHARACTERS ENTERED BY THE USER FROM FIELD POSITION FROM PARSER
{
    char *j = &str1[pos[y]];

    return j;

}

int16_t readAdc0Ss3()
{
    //putsUart0("\n\r ADC0");
    ADC0_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    return ADC0_SSFIFO3_R;                           // get single result from the FIFO
}

int16_t readAdc1Ss3()
{
    //putsUart0("\n\r ADC1");
    ADC1_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
    while (ADC1_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    return ADC1_SSFIFO3_R;                           // get single result from the FIFO
}

float voltage()
{
    raw1 = readAdc0Ss3();
    r1= ((raw1*3.3)/4096);
    raw2 = readAdc1Ss3();
    r2= ((raw2*3.3)/4096);
    deltav = (r1-r2);
    putsUart0("\n\r Volt1:");
    sprintf(v1, "%.2f", r1);
    putsUart0(v1);
    putsUart0("\n\r");
    putsUart0("\n\r Volt2:");
    sprintf(v2, "%.2f", r2);
    putsUart0(v2);
    putsUart0("\n\r");
    putsUart0("\n\r Difference in Voltage:");
    sprintf(x, "%.3f", deltav);
    putsUart0( x);
    putsUart0("\n\r");
}

void compIsr() //comparator interrupt
{
    cap=WTIMER5_TAV_R;
    //WTIMER5_TAV_R = 0;
    GREEN_LED=1;
    //waitmillisec(500000);
    GREEN_LED = 0;
    COMP_ACMIS_R |=0x01;
    NVIC_EN0_R &=~(1<<(INT_COMP0-16));
    //COMP_ACMIS_R|=0x01;
}

bool iscommand(char* verb,int arg)
{
    if((strcmp(verb,&str1[pos[0]])==0) && (strcmp(arg,min_argc)==0))
        return 1;
    else
        return 0;

}



// supporting commands step 4
commands()
{
    if(iscommand("set",2))
    {
        arg1 = getvalue(1);
        arg2 = getvalue(2);

                if ((strcmp(arg1, "meas_c")) == 0 && (strcmp(arg2, "on")) == 0)
                {
                   MEASURE_LR = 0;
                   MEASURE_C = 1;
                   //putsUart0("\n\r Meas_C on ");
                   putsUart0("\n\r");

                }
                else if ((strcmp(arg1, "meas_c")) == 0 && (strcmp(arg2, "off")) == 0)
                {
                    MEASURE_C = 0;
                    //putsUart0("\n\r Meas_C off ");
                    putsUart0("\n\r");
                }
                else if ((strcmp(arg1, "meas_lr")) == 0 && (strcmp(arg2, "on")) == 0)
                {
                     MEASURE_C = 0;
                     MEASURE_LR = 1;
                     //putsUart0("\n\r Meas_LR on ");
                     putsUart0("\n\r");

                }
                else if ((strcmp(arg1, "meas_lr")) == 0 && (strcmp(arg2, "off")) == 0)
                {
                     MEASURE_LR = 0;
                     //putsUart0("\n\r Meas_LR off ");
                     putsUart0("\n\r");

                }
                else if ((strcmp(arg1, "highside_r")) == 0 && (strcmp(arg2, "on")) == 0)
                {
                    LOWSIDE_R=0;
                    HIGHSIDE_R = 1;
                    //putsUart0("\n\r High on ");
                    putsUart0("\n\r");

                }
                else if ((strcmp(arg1, "highside_r")) == 0 && (strcmp(arg2, "off")) == 0)
                {

                    HIGHSIDE_R = 0;
                    //putsUart0("\n\r High off ");
                    putsUart0("\n\r");

                }
                else if ((strcmp(arg1, "lowside_r")) == 0 && (strcmp(arg2, "on")) == 0)
                {
                    HIGHSIDE_R=0;
                    LOWSIDE_R = 1;
                    //putsUart0("\n\r Low on ");
                    putsUart0("\n\r");

                }
                else if ((strcmp(arg1, "lowside_r")) == 0 && (strcmp(arg2, "off")) == 0)
                {
                    LOWSIDE_R = 0;
                    //putsUart0("\n\r Low off ");
                    putsUart0("\n\r");

                }
                else if ((strcmp(arg1, "integrate")) == 0 && (strcmp(arg2, "on")) == 0)
                {
                    putsUart0("\n\r ");
              Jump3:putsUart0("Select");
                    putsUart0("1)Lowside & Integrate On    2)Highside & Integrate On 3)Only Integrate:");
                    putsUart0("\n\r");
                    s=getcUart0();
                    switch(s)
                    {
                        case '1':
                                 HIGHSIDE_R=0;
                                 LOWSIDE_R = 1;
                                 INTEGRATE = 1;
                                 //putsUart0("\n\r Low & Int on ");
                                 putsUart0("\n\r");
                                 break;
                        case '2':
                                 LOWSIDE_R = 0;
                                 HIGHSIDE_R=1;
                                 INTEGRATE = 1;
                                 //putsUart0("\n\r High & Int on ");
                                 putsUart0("\n\r");
                                 break;
                        case '3':
                                 INTEGRATE = 1;
                                 //putsUart0("\n\r Int on");
                                 putsUart0("\n\r");
                                 break;
                        default:
                               putsUart0("\n\r");
                               putsUart0("wrong selection");
                               putsUart0("\n\r");
                               goto Jump3;

                    }

                }
                else if ((strcmp(arg1, "integrate")) == 0 && (strcmp(arg2, "off")) == 0)
                {
                    INTEGRATE = 0;
                    //putsUart0("\n\r Int off ");

                }
    }


    //step 6  Voltage cmd
    else if(iscommand("voltage",0))
    {
        voltage();

       /* raw1 = readAdc0Ss3();
        r1= ((raw1*3.3)/4096);
        raw2 = readAdc1Ss3();
        r2= ((raw2*3.3)/4096);
        deltav = (r1-r2);
        putsUart0("\n\r Volt1:");
        sprintf(v1, "%.2f", r1);
        putsUart0(v1);
        putsUart0("\n\r");
        putsUart0("\n\r Volt2:");
        sprintf(v2, "%.2f", r2);
        putsUart0(v2);
        putsUart0("\n\r");
        putsUart0("\n\r Difference in Voltage:");
        sprintf(x, "%.3f", deltav);
        putsUart0( x);
        putsUart0("\n\r");*/

    }

    //step 7 Reset cmd
    else if(iscommand("reset",0))
    {
        NVIC_APINT_R= NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
        putsUart0("\n\r");
    }


    //step 8 resistor cmd
    else if(iscommand("resistor",0))
    {
        uint8_t l=0;
        putsUart0("Measuring");
        putsUart0("\n\r");
        INTEGRATE=1;
        LOWSIDE_R=1;
        waitmillisec(2000000);
        LOWSIDE_R=0;
        WTIMER5_CTL_R |= TIMER_CTL_TAEN;
        WTIMER5_TAV_R=0;
        MEASURE_LR=1;
        NVIC_EN0_R |=(1<<(INT_COMP0-16));
        waitmillisec(2000000);
        putsUart0("Time:");
        sprintf(z,"%u", cap);
        putsUart0(z);
        putsUart0("\n\r");
        if((cap>=46) && (cap<=83))          // 1.5 ohm to 7.6
        {
        res1=(1*cap);
        res2=res1-35.587;
        resistance=res2/5.55;
        putsUart0("Resistance Value:");
        sprintf(re,"%f",resistance);
        putsUart0(re);
        putsUart0("\n\r");
        }
        if((cap>=718) && (cap<=5133))          // 10 ohm to 100 ohm
        {
         res1=(1*cap);
         res2=res1-425.98;
         resistance=res2/46.83;
         putsUart0("Resistance Value:");
         sprintf(re,"%f",resistance);
         putsUart0(re);
         putsUart0("\n\r");
        }
        if((cap>=5210) && (cap<=50180))      // 100 to 1k ohm
        {
        res1=(180*cap);
        res2=res1-39400;
        res3=res2/8993;
        resistance=res3;
        putsUart0("Resistance Value:");
        sprintf(re,"%f",resistance);
        putsUart0(re);
        putsUart0("\n\r");
        }
        if((cap>=50180) && (cap<=501200))      //  1k to 10k ohm
        {
         res1=(150*cap);
         res2=res1-10000;
         res3=res2/7517;
         resistance=res3;
         putsUart0("Resistance Value:");
         sprintf(re,"%f",resistance);
         putsUart0(re);
         putsUart0("\n\r");
        }
        if((cap>=501200) && (cap<=5200000))      //  10k to 100k ohm
        {
        res1=(225*cap);
        res2=res1+4700000;
        res3=res2/11747;
        resistance=res3+1000;
        putsUart0("Resistance Value:");
        sprintf(re,"%f",resistance);
        putsUart0(re);
        putsUart0("\n\r");
        }
        if((cap>=5200000) && (cap<=26500000))      //  100k to 500k ohm
        {
         res1=(4*cap);
         res2=res1+500000;
         res3=res2/213;
         resistance=res3;
         putsUart0("Resistance Value:");
         sprintf(re,"%f",resistance);
         putsUart0(re);
         putsUart0("\n\r");
        }
        if((cap>=26500000) && (cap<=55768881))      //  500k to 1m ohm
        {
         res1=(500000*cap);
         res2=res1+1384440500000;
         resistance=res2/29268881;
         putsUart0("Resistance Value:");
         sprintf(re,"%f",resistance);
         putsUart0(re);
         putsUart0("\n\r");
        }
        voltage();
        /*res1=cap+67484;
        res2=(res1/55.737);
        resistance=res2;
        sprintf(re,"%f",resistance);
        putsUart0(re);*/
        putsUart0("\n\r");
        l=strlen(z);
        for(i=0;i<l;i++)
        {
          z[i]=0;
        }
        cap=0;
        INTEGRATE=0;
        MEASURE_LR=0;
        resistance=0;
        COMP_ACREFCTL_R|=0x00000000;
    }


    // step 9 Capacitor cmd
     else if(iscommand("capacitor",0))
     {
       uint8_t l1=0;
       MEASURE_C=1;
       LOWSIDE_R=1;
       waitmillisec(1000000);
       MEASURE_C=0;
       LOWSIDE_R=0;
       putsUart0("Measuring");
       putsUart0("\n\r");
       MEASURE_C=1;
       LOWSIDE_R=1;
       waitmillisec(15000000);
       LOWSIDE_R=0;
       WTIMER5_CTL_R |= TIMER_CTL_TAEN;
       WTIMER5_TAV_R=0;
       HIGHSIDE_R=1;
       NVIC_EN0_R |=(1<<(INT_COMP0-16));
       waitmillisec(15000000);
       putsUart0("Time:");
       sprintf(z,"%u", cap);
       putsUart0(z);
       putsUart0("\n\r");
       if((cap>=2013) && (cap<=2637))      //  100pf to 200pf ohm
       {
         capa1=(1*cap);
         capa2=capa1-1389;
         capacitance=capa2/6240000;
         putsUart0("capacitance Value:");
         sprintf(cp,"%f",capacitance);
         putsUart0(cp);
         putsUart0("\n\r");
       }
       if((cap>=2637) && (cap<=3125))      //  200pf to 300pf ohm
       {
         capa1=(1*cap);
         capa2=capa1-851;
         capacitance=capa2/7580000;
         putsUart0("capacitance Value:");
         sprintf(cp,"%f",capacitance);
         putsUart0(cp);
         putsUart0("\n\r");
       }
       if((cap>=3125) && (cap<=3676))      //  200pf to 400pf ohm
       {
         capa1=(1*cap);
         capa2=capa1-1472;
         capacitance=capa2/5510000;
         putsUart0("capacitance Value:");
         sprintf(cp,"%f",capacitance);
         putsUart0(cp);
         putsUart0("\n\r");
       }
       if((cap>=3676) && (cap<=6510))      //  400pf to 1nf ohm
       {
         capa1=(1*cap);
         capa2=capa1-1787;
         capacitance=capa2/4723334;
         putsUart0("capacitance Value:");
         sprintf(cp,"%f",capacitance);
         putsUart0(cp);
         putsUart0("\n\r");
       }
       if((cap>6510) && (cap<=554000))      //  1nf to 0.1uf ohm
       {
        capa1=(99*cap);
        capa2=capa1-97000;
        capacitance=capa2/547490000;
        putsUart0("capacitance Value:");
        sprintf(cp,"%f",capacitance);
        putsUart0(cp);
        putsUart0("\n\r");
       }
       if((cap>=554000) && (cap<=5104600))      //  0.1uf to 1uf ohm
       {
        capa1=(9*cap);
        capa2=capa1-435400;
        capacitance=capa2/45506000;
        putsUart0("capacitance Value:");
        sprintf(cp,"%f",capacitance);
        putsUart0(cp);
        putsUart0("\n\r");
       }
       if((cap>=5104600) && (cap<=61953650))      //  1uf to 10uf ohm
       {
        capa1=(9*cap);
        capa2=capa1+10907650;
        capacitance=capa2/56849050;
        putsUart0("capacitance Value:");
        sprintf(cp,"%f",capacitance);
        putsUart0(cp);
        putsUart0("\n\r");
       }
       if((cap>=61953650) && (cap<=300017453))      //  10uf to 47uf ohm
       {
        capa1=(5*cap);
        capa2=capa1-5259520;
        capa3=capa2/30450873;
        capacitance=capa3-1;
        putsUart0("capacitance Value:");
        sprintf(cp,"%f",capacitance);
        putsUart0(cp);
        putsUart0("\n\r");
       }
       if((cap>=300017453) && (cap<=635574841))      //  47uf to 100uf ohm
       {
         capa1=(1*cap);
         capa2=capa1-2447694;
         capa3=capa2/6331272;
         capacitance=capa3;
         putsUart0("capacitance Value:");
         sprintf(cp,"%f",capacitance);
         putsUart0(cp);
         putsUart0("\n\r");
       }

       voltage();
       l1=strlen(z);
       for(i=0;i<l1;i++)
       {
         z[i]=0;
       }
       cap=0;
       MEASURE_C=0;
       HIGHSIDE_R=0;
       capacitance=0;
       COMP_ACREFCTL_R|=0x00000000;
    }


    // Step 10 Inductor cmd
     else if(iscommand("inductor",0))
    {

        putsUart0("Measuring");
        putsUart0("\n\r");
        INTEGRATE=0;
        HIGHSIDE_R=0;
        MEASURE_C=0;
        LOWSIDE_R=1;
        waitmillisec(5000000);
        LOWSIDE_R=0;
        MEASURE_LR=1;
        LOWSIDE_R=1;
        WTIMER5_CTL_R |= TIMER_CTL_TAEN;
        WTIMER5_TAV_R=0;
        NVIC_EN0_R |=(1<<(INT_COMP0-16));
        waitmillisec(5000000);
        putsUart0("Time:");
        sprintf(z,"%u", cap);
        putsUart0(z);
        putsUart0("\n\r");
        if((cap>=39) && (cap<=127))          // 22uh to 100uh
        {
         ind1=(cap);
         ind2=ind1-15.744;
         ind3=ind2/1102.6;
         inductance= ind3;
         putsUart0("Inductance:");
         sprintf(in,"%f", inductance);
         putsUart0(in);
         putsUart0("\n\r");
        }
        if((cap>=127) && (cap<=509))         // 100uh to 380uh
        {
         ind1=cap;
         ind2=ind1+14.636;
         ind3=ind2/1316.4;
         inductance= ind3;
         putsUart0("Inductance:");
         sprintf(in,"%f", inductance);
         putsUart0(in);
         putsUart0("\n\r");
        }
        if((cap>=127) && (cap<=6391))          //380uh to 3.3mh
        {
          ind1=cap;
          ind2=ind1+116.74;
          ind3=ind2/1668.7;
          inductance= ind3;
          putsUart0("Inductance:");
          sprintf(in,"%f", inductance);
          putsUart0(in);
          putsUart0("\n\r");
        }
        voltage();
        voldiv=(r2/r1);
        putsUart0(" Test ESR cmd   1) yes 2) no ");
        putsUart0("\n\r");
 Jump4: e=getcUart0();
        switch(e)
        {
            case '1':
                 esr1= voldiv;
                 esr2= 33/esr1;
                 esr3= esr2-33;
                 esr=esr3;
                 putsUart0("ESR:");
                 sprintf(es,"%f", esr);
                 putsUart0(es);
                 putsUart0("\n\r");
                 break;
            case '2':
                 break;
            default:
                 putsUart0("\n\r");
                 putsUart0("wrong selection");
                 putsUart0("\n\r");
                 goto Jump4;
        }
        l1=strlen(z);
        for(i=0;i<l1;i++)
        {
          z[i]=0;
        }
        LOWSIDE_R=0;
        MEASURE_LR=0;
        MEASURE_C=1;
        LOWSIDE_R=1;
        cap=0;

    }


    else if(iscommand("esr",0))
    {

        putsUart0("Measuring");
        putsUart0("\n\r");
        INTEGRATE=0;
        HIGHSIDE_R=0;
        MEASURE_C=0;
        voltage();
        LOWSIDE_R=1;
        waitmillisec(5000000);
        LOWSIDE_R=0;
        MEASURE_LR=1;
        LOWSIDE_R=1;
        WTIMER5_CTL_R |= TIMER_CTL_TAEN;
        WTIMER5_TAV_R=0;
        NVIC_EN0_R |=(1<<(INT_COMP0-16));
        waitmillisec(5000000);
        voltage();
        voldiv=(r2/r1);
        esr1= voldiv;
        esr2= 33/esr1;
        esr3= esr2-33;
        esr=esr3;
        putsUart0("ESR:");
        sprintf(es,"%f", esr);
        putsUart0(es);
        putsUart0("\n\r");
        LOWSIDE_R=0;
        MEASURE_LR=0;
        MEASURE_C=1;
        LOWSIDE_R=1;
    }


/*  else if(iscommand("auto",0))
    {

    }*/

    else
    {
        putsUart0("\n\r Invalid Command");
        putsUart0("\n\r");
    }

    for(i=0;i<len;i++)
    {
        str[i]=0;
        str1[i]=0;
        pos[i]=0;
        type[i]=0;
    }

}


//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    // Initialize hardware
    initHw();

    while (1)
    {
        argc=0;
        count1=0;
        count2=0;
        led();
        putsUart0("\nEnter the Cmd:\n\r");
        putsUart0("\n\r");
        getstring();
        putsUart0("\nEntered Cmd is:\n\r");
        putsUart0(str);
        putsUart0("\n\r");
        //putsUart0("\nAfter parsing:\n\r ");
        parsestring();
        commands();
    }
}
