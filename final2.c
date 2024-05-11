/* Final Project - Speed Measurement System
 * Description: This program uses two ultrasonic sensors connected to an MSP430 to measure
 * the speed of an object moving between them and count laps on sensor triggers.
 * Three leds are used to indicate the number of laps the object has made.
 * Uart is used to print the speed of the object.
 * By Daniel Vega
 */

#include <msp430.h>
#include <stdio.h>

#define ECHO2 0x01   // P2.0 as Echo for the first sensor
#define TRIG2 0x10   // P1.4 as Trigger for the first sensor
#define TRIG1 0x20   // P1.5 as Trigger for the second sensor
#define ECHO1 0x02   // P2.1 as Echo for the second sensor

// Define pins for LED indicators
#define LED2 0x01  // P1.0
#define LED1 0x40  // P1.6
#define LED3 0x08  // P1.3


//UART pins

#define UART_TXD 0x04  // P1.2 = UART TxD
#define TIMER0_PERIOD 1200  // 100 ms. Based off SMACLK

/* Globals */
char *g_pString;
int g_donePrinting = 1;
int detection1 = 0; // 0 means no car was detected last time; 1 means a car was detected
int detection2 = 0;

/* TimerA-0 Interrupt Service Routine */
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A(void)
{
    /* Wake-up main thread */
    LPM0_EXIT;
}

/* USCI TX Interrupt Service Routine */
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI_TX_ISR(void)
{
    char byte = g_pString[0];
    if (byte == '\0')
    {
        /* All done.  Disable interrupts */
        IE2 &= ~UCA0TXIE;
        /* Set the all-done indication */
        g_donePrinting = 1;
    }
    else
    {
        /* Push the next byte into USCI TX state machine */
        UCA0TXBUF = byte;
        /* Advance the string pointer to the next character */
        g_pString++;
    }
}

void UART_Print_String(char string[])
{
    /* Store the pointer to the next character in the string */
    g_pString = &string[1];
    /* Put the first byte into the USCI TX state machine */
    UCA0TXBUF = string[0];
    /* Enable USCI TX interrupts */
    IE2 |= UCA0TXIE;
}

void main(void)
{
    unsigned int initial_time1, time_elapsed1, distance_cm1, total_time;
    unsigned int initial_time2, time_elapsed2, distance_cm2;
    unsigned int sensor1_time = 0, sensor2_time = 0; // Add these lines
    unsigned int speed_cms_scaled = 0; // Speed in cm/s scaled by 100
    unsigned int laps = 0; // count laps
    int status1 = 0; // Initial state for sensor 1
    int status2 = 0; // Initial state for sensor 2
    char num_string1[16], num_string2[16], speed_string[64]; // Add speed_string declaration here
    
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

    while (1) {
        LPM0;  // Sleep until next clock tick

        // Handle first sensor
        P1OUT |= TRIG1;
        __delay_cycles(10);
        P1OUT &= ~TRIG1;
        while (!(P2IN & ECHO1));
        initial_time1 = TA1R;
        while (P2IN & ECHO1);
        time_elapsed1 = TA1R - initial_time1;
        distance_cm1 = time_elapsed1 / 58;

       // Check current distance for sensor 1 and update the sensor state
        if (distance_cm1 < 21) {
            status1 = 1; // Object detected within range
        } else {
            status1 = 0; // No object detected
        }

        // Compare the new state with the last known state for sensor 1
        if (detection1 != status1) {
            if (status1 == 1) {
                if (laps > 0) {
                    // Actions to be performed on detection, like decrementing parking spots
                    // display_hex_digit(parking_spots);
                }
            }
            detection1 = status1; // Update the last known state
        }

        // Handle second sensor
        P1OUT |= TRIG2;
        __delay_cycles(10);
        P1OUT &= ~TRIG2;
        while (!(P2IN & ECHO2));
        initial_time2 = TA1R;
        while (P2IN & ECHO2);
        time_elapsed2 = TA1R - initial_time2;
        distance_cm2 = time_elapsed2 / 58;

        // Check current distance for sensor 2 and update the sensor state
        if (distance_cm2 < 21) {
            status2 = 1; // Object detected within range
        } else {
            status2 = 0; // No object detected
        }

        // Compare the new state with the last known state for sensor 2
        if (detection2 != status2) {
            if (status2 == 1) {
                if (laps < 4) {
                    laps++; // Increment laps only if the object is detected
                }
            }
            detection2 = status2; // Update the last known state
        }

        // Process distances and calculate speed if necessary
        if (distance_cm1 < 21 || distance_cm1 > 24) {
            sensor1_time = TA1R;  // Record time when condition is met

            if(laps == 1)
            {
                P1OUT |= LED1;
            }
            if(laps == 2)
            {
                P1OUT |= LED2;
            }
            if(laps == 3)
            {
                P1OUT |= LED3;
            }
            if(laps == 4)
            {
                P1OUT &= ~(LED1 | LED2 | LED3); // Turn off all LEDs
                laps = 0;
            }
            //laps++;
        }

        if (distance_cm2 < 21 || distance_cm2 > 24) {
            sensor2_time = TA1R;  // Record time when condition is met
            if (sensor1_time != 0 && sensor2_time != 0) {
                unsigned int time_diff = sensor2_time - sensor1_time;
                if (time_diff != 0) {
                    unsigned int distance_between_sensors_scaled = 5500; // Assume 55 cm * 100 for scaling
                    // Calculate the speed in centimeters per second (cm/s) scaled by a factor of 1000.
                    // The formula used is:
                    // speed_cms_scaled = (distance_in_cm * scaling_factor * microseconds_per_second) / time_in_microseconds
                    // where:
                    // - distance_in_cm * scaling_factor = 5500 (distance scaled up by 100 for precision)
                    // - microseconds_per_second = 1000000 (converts seconds to microseconds for unit consistency)
                    // - time_in_microseconds = time_diff (the time difference between sensor triggers in microseconds)
                    // The scaling by 1000000 ensures that the division by time_diff (in microseconds) results in a unit conversion
                    // from microseconds to seconds, maintaining the speed in cm/s, but scaled by 1000 for precision.
                    speed_cms_scaled = (distance_between_sensors_scaled * 1000000) / time_diff;
                    // Format the speed value for printing. The speed is divided by 1000 to convert it from the scaled value
                    // back to a real-world value in cm/s. This step adjusts for the initial scaling factor applied
                    // to the distance (x100) and the unit conversion scaling (x1000000 / x1000 = x1000 overall scaling).
                    sprintf(speed_string, "Speed: %d cm/s\n\r", speed_cms_scaled / 1000);
                    if (g_donePrinting) {
                        g_donePrinting = 0;
                        UART_Print_String(speed_string);
                    }
                }
                // Reset times after use
                sensor1_time = 0;
                sensor2_time = 0;
            }
        }
        /*
        // Print distances
        if (g_donePrinting)
        {
            g_donePrinting = 0;
            if (distance_cm1 < 21 || distance_cm1 > 24)
            {
                sprintf(num_string1, "D1: %d cm\n\r", distance_cm1);
                UART_Print_String(num_string1);
            }
            if (distance_cm2 < 21 || distance_cm2 > 24)
            {
                sprintf(num_string2, "D2: %d cm\n\r", distance_cm2);
                UART_Print_String(num_string2);
            }
            // Reset the flag if no conditions were met to print anything
            if ((distance_cm1 >= 21 && distance_cm1 <= 24) && (distance_cm2 >= 21 && distance_cm2 <= 24))
            {
                g_donePrinting = 1;
            }
        }*/
    }
}
