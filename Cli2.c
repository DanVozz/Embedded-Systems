/* Final Project - Speed Measurement System
 * Description: This program uses two ultrasonic sensors connected to an MSP430 to measure
 * the speed of an object moving between them and count laps on sensor triggers.
 * Three leds are used to indicate the number of laps the object has made.
 * Uart is used to print the speed of the object.
 * By Daniel Vega
 */

#include <msp430.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define ECHO2 0x01   // P2.0 as Echo for the first sensor
#define TRIG2 0x10   // P1.4 as Trigger for the first sensor
#define TRIG1 0x20   // P1.5 as Trigger for the second sensor
#define ECHO1 0x02   // P2.1 as Echo for the second sensor

// Define pins for LED indicators
#define LED2 0x01  // P1.0
#define LED1 0x40  // P1.6
#define LED3 0x08  // P1.3


//UART pins

#define UART_RXD 0x02   // P1.1 is RxD //
#define UART_TXD 0x04  // P1.2 = UART TxD
#define TIMER0_PERIOD 1200  // 100 ms. Based off SMACLK

#define MAX_CMD_WORDS 2
#define MAX_CMD_WORD_LEN 10

#define TIMER_PERIOD 1 // About 833 us

/* Globals */
char *g_pTxString;
volatile unsigned int g_donePrinting;

/* RX UART */
char g_cmdString[MAX_CMD_WORDS][MAX_CMD_WORD_LEN];
unsigned int g_cmdWordIdx=0;
unsigned int g_cmdLetterIdx=0;
volatile unsigned int g_doneReceiving;

unsigned int g_ledState = 0; 


/* TimerA-0 Interrupt Service Routine */
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A(void)
{
    /* Wake-up main thread */
    LPM0_EXIT;
}

#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI_TX_ISR(void)
{
    char byte;

    /* Get next character from string */
    byte = g_pTxString[0];

    if (byte == '\0')
    {
        /* All done.  Disable TX interrupts */
        IE2 &= ~UCA0TXIE;

        /* Notify main process that we are done */
        g_donePrinting = 1;
    }
    else
    {
        /* Push the next byte into USCI TX state machine */
        UCA0TXBUF = byte;

        /* Advance the string pointer to the next character */
        g_pTxString++;
    }
}

/* Gets the next command
 * The caller will use gCmdString directly
 * Returns the number of words received
 */
unsigned int getNextCmd()
{
    g_doneReceiving = 0;

    /* Reset the command strings */
    g_cmdWordIdx=0;
    g_cmdLetterIdx=0;

    /* Enable USCI RX interrupts */
    IE2 |= UCA0RXIE;

    /* Wait until done receiving commands */
    while (!g_doneReceiving);

    /* Send another newline */
    UART_Print_String("\n\r");

    return g_cmdWordIdx;
}
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI_RX_ISR(void)
{
    char byte;

    /* Get received byte */
    byte = UCA0RXBUF;

    /* Echo byte back to TX */
    while (!(IFG2 & UCA0TXIFG));
    UCA0TXBUF = byte;

    switch (byte)
    {
    case ' ':
        /* Terminate the string */
        g_cmdString[g_cmdWordIdx][g_cmdLetterIdx] = '\0';

        /* Move to the next word */
        g_cmdWordIdx++;
        g_cmdLetterIdx=0;

        if (g_cmdWordIdx == MAX_CMD_WORDS)
        {
            /* Don't listen to RX anymore */
            IE2 &= ~UCA0RXIE;
            g_doneReceiving = 1;
        }
        break;

    case '\r':
        /* Terminate the string */
        g_cmdString[g_cmdWordIdx][g_cmdLetterIdx] = '\0';
        g_cmdWordIdx++;
        g_cmdLetterIdx=0;

        /* Don't listen to RX anymore */
        IE2 &= ~UCA0RXIE;
        g_doneReceiving = 1;
        break;

    default:
        /* Copy received byte into current string */
        g_cmdString[g_cmdWordIdx][g_cmdLetterIdx++] = byte;

        if (g_cmdLetterIdx == MAX_CMD_WORD_LEN)
        {
            /* Don't listen to RX anymore */
            IE2 &= ~UCA0RXIE;
            g_doneReceiving = 1;
        }
    }
}

void UART_Print_String(char string[])
{
    g_donePrinting = 0;

    /* Store the pointer to the next character in the string */
    g_pTxString = &string[1];

    /* Put the first byte into the USCI TX state machine */
    while (!(IFG2 & UCA0TXIFG));
    UCA0TXBUF = string[0];

    /* Enable USCI TX interrupts */
    IE2 |= UCA0TXIE;

    /* Wait until done printing response */
    while (!g_donePrinting);
}

void main(void)
{
    unsigned int initial_time1, time_elapsed1, distance_cm1, total_time;
    unsigned int initial_time2, time_elapsed2, distance_cm2;
    unsigned int sensor1_time = 0, sensor2_time = 0; // Add these lines
    unsigned int speed_cms_scaled = 0; // Speed in cm/s scaled by 100
    unsigned int laps = 0; // count laps
    char num_string1[16], num_string2[16], speed_string[64]; // Add speed_string declaration here
    unsigned int numWords;
  
    /* Stop watchdog timer */
    WDTCTL = WDTPW | WDTHOLD;

    /* Set pin 1.4 to output */
    P1SEL &= ~TRIG1;
    P1SEL2 &= ~TRIG1;
    P1DIR |= TRIG1;
    P1OUT &= ~TRIG1;

    /* Set pin 1.5 to output for second sensor */
    P1SEL &= ~TRIG2;
    P1SEL2 &= ~TRIG2;
    P1DIR |= TRIG2;  // Set P1.4 as output
    P1OUT &= ~TRIG2;

    /* Set pin 2.0 to input */
    P2SEL &= ~ECHO1;
    P2SEL2 &= ~ECHO1;
    P2REN &= ~ECHO1;
    P2DIR &= ~ECHO1;    // Set P2.0 as input

    /* Set pin 2.1 to input for second sensor */
    P2SEL &= ~ECHO2;
    P2SEL2 &= ~ECHO2;
    P2REN &= ~ECHO2;
    P2DIR &= ~ECHO2;

    // Set LED pins as outputs
    P1DIR |= (LED1 | LED2 | LED3);
    // Initially turn off LEDs
    P1OUT &= ~(LED1 | LED2 | LED3);

    /* Set DCOCLK to 1 MHz. This drives SMCLK by default. */
    DCOCTL = 0;             // Reset
    BCSCTL1 = CALBC1_1MHZ;  // Settings for 1 MHz clock
    DCOCTL = CALDCO_1MHZ;   // Set frequency to 1 MHz (calibrated)

    /* Select ACLK source from VLOCLK */
    BCSCTL3 |= LFXT1S_2;

    /* Set Timer 0 to drive the cyclic scheduler */
    TACCR0 = TIMER0_PERIOD;
    TACTL = TASSEL_1 | ID_0 | MC_1 | TACLR;
    TACCTL0 = CCIE;

    /* Set Timer 1 as continuous source for time measurements */
    TA1CTL = TASSEL_2 | ID_0 | MC_2 | TACLR;

    /* Configure USCI for UART operation */
    UCA0CTL1 |= UCSWRST;
    P1SEL |= UART_RXD;
    P1SEL2 |= UART_RXD;
    P1SEL |= UART_TXD;
    P1SEL2 |= UART_TXD;
    UCA0CTL0 = 0;
    UCA0CTL1 = UCSSEL_2;
    UCA0BR0 = 104;
    UCA0BR1 = 0;
    UCA0MCTL = UCBRS0;
    UCA0CTL1 &= ~UCSWRST;

    /* Enable interrupts globally */
    __enable_interrupt();

    while (1)
    {
        LPM0;  // Sleep until next clock tick

        P1OUT |= TRIG1;
        __delay_cycles(10);
        P1OUT &= ~TRIG1;
        while (!(P2IN & ECHO1));
        initial_time1 = TA1R;
        while (P2IN & ECHO1);
        time_elapsed1 = TA1R - initial_time1;
        distance_cm1 = time_elapsed1 / 58;

        // Handle second sensor
        P1OUT |= TRIG2;
        __delay_cycles(10);
        P1OUT &= ~TRIG2;
        while (!(P2IN & ECHO2));
        initial_time2 = TA1R;
        while (P2IN & ECHO2);
        time_elapsed2 = TA1R - initial_time2;
        distance_cm2 = time_elapsed2 / 58;

        /* Print the prompt indication */
        UART_Print_String("> ");

        numWords = getNextCmd();
        // Handle first sensor
        if ( (numWords == 2) && (strcmp(g_cmdString[0], "led")==0))
        {
            if (strcmp(g_cmdString[1], "on")==0)
            {
                g_ledState = 1;
                P1OUT |= (LED1 | LED2 | LED3);
                UART_Print_String("OK\n\r");
            }

            if (strcmp(g_cmdString[1], "off")==0)
            {
                g_ledState = 0;
                P1OUT &= ~(LED1 | LED2 | LED3);
                UART_Print_String("OK\n\r");
            }
        }

        if ( (numWords == 2) && (strcmp(g_cmdString[0], "sensors")==0))
        {
            if (strcmp(g_cmdString[1], "on")==0)
            {
                // Print distances
                sprintf(num_string1, "D1: %d cm\n\r", distance_cm1);
                UART_Print_String(num_string1);

                sprintf(num_string2, "D2: %d cm\n\r", distance_cm2);
                UART_Print_String(num_string2);

                // Reset the flag if no conditions were met to print anything
            }
            if (strcmp(g_cmdString[1], "off")==0)
            {
                g_ledState = 0;
                UART_Print_String("OK\n\r");
            }
        }
        if ((numWords == 2) && (strcmp(g_cmdString[0], "color")==0) )
        {
            if (strcmp(g_cmdString[1], "green")==0)
            {
                P1OUT |= LED1;
                UART_Print_String("OK\n\r");
            }
            if (strcmp(g_cmdString[1], "yellow")==0)
            {
                P1OUT |= LED2;
                UART_Print_String("OK\n\r");
            }
            if (strcmp(g_cmdString[1], "red")==0)
            {
                P1OUT |= LED3;
                UART_Print_String("OK\n\r");
            }
        }
    }
}




