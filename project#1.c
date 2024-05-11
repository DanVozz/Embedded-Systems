
 /* Embedded Systems
 * Project #1
 * By Daniel Vega
 */
#include <msp430.h>

#define B1 0x02 // Red
#define B3 0x08 // Green
#define B5 0x20 // Blue
#define B6 0x08 // Button
#define B135 (B1 | B3 | B5)
#define B036 (B6 | B1 | B3 | B5)

#define TIMER_PERIOD 1
#define CHANGE_COLOR 2000
#define LED_ON_TICKS 1
#define GENERAL_TICKS 6
#define TEMP_TOLERANCE 3


void updateLEDColorsBasedOnTemperature(current_temp, temp_reference, toggle_count);

/* TimerA-0 Interrupt Service Routine */
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A(void)
{
    /* Return to active mode */
    __low_power_mode_off_on_exit ();
}

void main(void)
{
    int temp_reference;
    int current_temp;
    volatile unsigned int color_state = 0;
    volatile unsigned int color_change_count = 0;
    volatile unsigned int toggle_count = 0;
    toggle_count = 0;

    /* Stop watchdog timer */
    WDTCTL = WDTPW | WDTHOLD;

    /* Select ACLK source from VLOCLK */
    BCSCTL3 |= LFXT1S_2;

    /* Set pins to I/O mode, disable pull-ups */
    P2SEL &= ~(B135);
    P2SEL2 &= ~(B135);
    P2REN &= ~(B135);

    /* Set pin 3 direction to input, pins 1 & 6 to output */
    P2DIR &= ~(B6);
    P2DIR |= (B135);

    /* Turn both LEDs off */
    P2OUT &= ~(B135);

    /* Configure ADC
     * SREF_1: Use VREF as VR+, and Vss as VR-
     * ADC10SHT_3: Use 64x sample & hold time
     * ADC10ON: Enable the ADC module
     * REFON: Enable the internal reference
     * REF2_5V: Use 1.5 V. reference
     * INCH_10: Source = internal temperature sensor
     * ADC10SSEL_0: Select ADC10OSC as ADC clock source
     * SHS_0: Use ADC10OSC as source for sample & hold
     * CONSEQ_0: Single conversion
     */
    ADC10CTL0 = SREF_1 | ADC10SHT_3 | ADC10ON | REFON;
    ADC10CTL1 = INCH_10 | ADC10SSEL_0 | SHS_0 | CONSEQ_0;

    /* Wait 5ms for reference to settle */
    TACTL = TASSEL_1 | ID_0 | MC_2 | TACLR;
    while (TAR < 30);

    /* Enable ADC captures
     * This needs to happen after all configurations are done
     */
    ADC10CTL0 |= ENC;

    /* Trigger one capture, to set the initial temperature reference */
    ADC10CTL0 |= ADC10SC;                   // Trigger capture
    while ((ADC10CTL0 & ADC10IFG) == 0);    // Wait for capture to finish
    temp_reference = ADC10MEM;              // Get value

    /* Configure Timer_A
     * MC_1: Continuous-up mode to TACCR0
     */
    TACCR0 = 1200;  // About 100 ms
    TACCTL0 = CCIE; // Enable interrupts on Compare 0
    TACTL = TASSEL_1 | ID_0 | MC_1 | TACLR;

    /* Enable interrupts globally */
    __enable_interrupt();

    while(1)
    {

        toggle_count = (toggle_count + 1) % GENERAL_TICKS;

        /* Trigger ADC capture */
        ADC10CTL0 |= ADC10SC;

        /* Wait for conversion to finish
         * Note that this goes against the single-tick philosophy of cyclic schedulers
         * If this were to take longer than one tick, it would break things
         */
        while ((ADC10CTL0 & ADC10IFG) == 0);
        current_temp = ADC10MEM;

        //toggle_count = (toggle_count + 1) % GENERAL_TICKS;
        P2OUT &= ~B135; // Clear LEDs
        /* Check for button pressed */
        if ((P1IN & B6) == 0)
        {
            /* Set the temperature reference */
            temp_reference = current_temp;
        }

        //while ((TACTL & TAIFG) == 0); //interfiere con los otros timers
        //TACTL &= ~TAIFG;

        color_change_count++;
        if (color_change_count >= CHANGE_COLOR)
        {
            color_change_count = 0;
            toggle_count = 0;
        }

        toggle_count = (toggle_count + 1) % GENERAL_TICKS;
        updateLEDColorsBasedOnTemperature(current_temp, temp_reference, toggle_count);

    }
}

void updateLEDColorsBasedOnTemperature(int current_temp, int temp_reference, unsigned int toggle_count)
{

    if (current_temp == temp_reference)
    {
        /* Turn green on */
        P2OUT &= ~B135;
        P2OUT |= B3;
    }//Base color green

    //Start of the warm colors
    //Warm colors will transition from green to yellow to orange to red
    else if (current_temp > (temp_reference + TEMP_TOLERANCE) && current_temp <= (temp_reference + TEMP_TOLERANCE + 2))
    {
        if (toggle_count < LED_ON_TICKS)
        {
            P2OUT |= B1|B3;
        }
        else
        {
            P2OUT |= B3;
        }// Yellow-Green
    }
    else if (current_temp > (temp_reference + TEMP_TOLERANCE+2) && current_temp <= (temp_reference + TEMP_TOLERANCE + 4))
    {
        if (toggle_count < LED_ON_TICKS+1)
        {
            P2OUT |= B1|B3;
        }
        else
        {
            P2OUT |= B3;
        }// Yellow-Green
    }
    else if (current_temp > (temp_reference + TEMP_TOLERANCE + 4) && current_temp <= (temp_reference + TEMP_TOLERANCE + 6))
    {
        if (toggle_count < LED_ON_TICKS+2)
        {
            P2OUT |= B1|B3;
        }
        else
        {
            P2OUT |= B3;
        }// Yellow-Green
    }
    else if (current_temp > (temp_reference + TEMP_TOLERANCE + 6) && current_temp <= (temp_reference + TEMP_TOLERANCE + 8))
    {
        if (toggle_count < LED_ON_TICKS+3)
        {
            P2OUT |= B1|B3;
        }
        else
        {
            P2OUT |= B3;
        }// Yellow-Green
    }
    else if (current_temp > (temp_reference + TEMP_TOLERANCE +8) && current_temp <= (temp_reference + TEMP_TOLERANCE + 10))
    {
        if (toggle_count < LED_ON_TICKS+4)
        {
            P2OUT |= B1|B3;
        }
        else
        {
            P2OUT |= B3;
        }// Yellow-Green
    }
    else if (current_temp > (temp_reference + TEMP_TOLERANCE + 10) && current_temp <= (temp_reference + TEMP_TOLERANCE + 12))
    {
        if (toggle_count < LED_ON_TICKS+3)
        {
            P2OUT |= B1|B3;
        }
        else
        {
            P2OUT |= B1;
        }// Orange-Yellow
    }
    else if (current_temp > (temp_reference + TEMP_TOLERANCE + 12) && current_temp <= (temp_reference + TEMP_TOLERANCE + 14))
    {
        if (toggle_count < LED_ON_TICKS+2)
        {
            P2OUT |= B1|B3;
        }
        else
        {
            P2OUT |= B1;
        }// Orange-Yellow
    }
    else if (current_temp > (temp_reference + TEMP_TOLERANCE + 14) && current_temp <= (temp_reference + TEMP_TOLERANCE + 16))
    {
        if (toggle_count < LED_ON_TICKS+1)
        {
            P2OUT |= B1|B3;
        }
        else
        {
            P2OUT |= B1;
        }// Orange
    }
    else if (current_temp > (temp_reference + TEMP_TOLERANCE + 16) && current_temp <= (temp_reference + TEMP_TOLERANCE + 18))
    {
        if (toggle_count < LED_ON_TICKS)
        {
            P2OUT |= B1|B3;
        }
        else
        {
            P2OUT |= B1;
        }// Red-Orange
    }
    else if (current_temp > (temp_reference + TEMP_TOLERANCE + 18) && current_temp <= (temp_reference + TEMP_TOLERANCE + 20))
    {
        if (toggle_count < LED_ON_TICKS+1)
        {
            P2OUT |= B1|B3;
        }
        else
        {
            P2OUT |= B1;
        }// Red-Orange
    }
    else if (current_temp > (temp_reference + TEMP_TOLERANCE + 20) && current_temp <= (temp_reference + TEMP_TOLERANCE + 22))
    {
        if (toggle_count < LED_ON_TICKS+2)
        {
            P2OUT |= B1|B3;
        }
        else
        {
            P2OUT |= B1;
        }// Red-Orange
    }
    else if (current_temp > (temp_reference + TEMP_TOLERANCE + 22))
    {
        P2OUT |= B1;
        // Red
    }
    //Start of the Cold colors
    //Cold colors will transition from green to cyan to blue to purple
    else if (current_temp < (temp_reference + TEMP_TOLERANCE) && current_temp >= (temp_reference + TEMP_TOLERANCE - 2))
    {
        if (toggle_count < LED_ON_TICKS)
        {
            P2OUT |= B3|B5;
        }
        else
        {
            P2OUT |= B3;
        }// Green-Cyan
    }
    else if (current_temp < (temp_reference + TEMP_TOLERANCE - 2) && current_temp >= (temp_reference + TEMP_TOLERANCE - 4))
    {
        if (toggle_count < LED_ON_TICKS+1)
        {
            P2OUT |= B3|B5;
        }
        else
        {
            P2OUT |= B3;
        }// Green-Cyan
    }
    else if (current_temp < (temp_reference + TEMP_TOLERANCE - 4) && current_temp >= (temp_reference + TEMP_TOLERANCE - 6))
    {
        if (toggle_count < LED_ON_TICKS+2)
        {
            P2OUT |= B3|B5;
        }
       // Less-Cyan
    }
    else if (current_temp <  (temp_reference + TEMP_TOLERANCE -6) && current_temp >= (temp_reference + TEMP_TOLERANCE - 8))
    {
        if (toggle_count < LED_ON_TICKS+3)
        {
            P2OUT |= B3|B5;
        }
       //  Less-Cyan
    }
    else if (current_temp <  (temp_reference + TEMP_TOLERANCE - 8) && current_temp >= (temp_reference + TEMP_TOLERANCE - 10))
    {
        P2OUT |= B3|B5 ; // Cyan
    }
    else if (current_temp <  (temp_reference + TEMP_TOLERANCE - 10) && current_temp >= (temp_reference + TEMP_TOLERANCE - 12))
    {
        if (toggle_count < LED_ON_TICKS+1)
        {
            P2OUT |= B3|B5;
        }
        else
        {
            P2OUT |= B5;
        }// Cyan Blue
    }
    else if (current_temp <  (temp_reference + TEMP_TOLERANCE - 12) && current_temp >= (temp_reference + TEMP_TOLERANCE - 14))
    {
        if (toggle_count < LED_ON_TICKS+2)
        {
            P2OUT |= B3|B5;
        }
        else
        {
            P2OUT |= B5;
        }// Cyan Blue
    }
    else if (current_temp <  (temp_reference + TEMP_TOLERANCE - 14) && current_temp >= (temp_reference + TEMP_TOLERANCE - 16))
    {
        P2OUT |= B5;//Blue
    }
    else if (current_temp <  (temp_reference + TEMP_TOLERANCE - 16) && current_temp >= (temp_reference + TEMP_TOLERANCE - 18))
    {
        if (toggle_count < LED_ON_TICKS)
        {
            P2OUT |= B5 | B1;
        }
        else
        {
            P2OUT |= B5;
        }// Blue purple
    }
    else if (current_temp <  (temp_reference + TEMP_TOLERANCE - 18) && current_temp >= (temp_reference + TEMP_TOLERANCE - 20))
    {
        if (toggle_count < LED_ON_TICKS+1)
        {
            P2OUT |= B5 | B1;
        }
        else
        {
            P2OUT |= B5;
        }// Blue purple
    }
    else if (current_temp <  (temp_reference + TEMP_TOLERANCE - 20) && current_temp >= (temp_reference + TEMP_TOLERANCE - 22))
    {
        if (toggle_count < LED_ON_TICKS+2)
        {
            P2OUT |= B5 | B1;
        }
        else
        {
            P2OUT |= B5;
        }// Blue purple
    }
    else if (current_temp <  (temp_reference + TEMP_TOLERANCE - 22) && current_temp >= (temp_reference + TEMP_TOLERANCE - 24))
    {
        if (toggle_count < LED_ON_TICKS+3)
        {
            P2OUT |= B5 | B1;
        }
        else
        {
            P2OUT |= B5;
        }// Blue purple
    }
    else if (current_temp <  (temp_reference + TEMP_TOLERANCE - 24))
    {
            P2OUT |= B5 | B1;
    }// purple
}
