#include "msp.h"
#include "Clock.h"

uint8_t readSwitches(void);


/**
 * main.c
 */
void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer
    Clock_Init48MHz();

    // Initializing the switches
    P1 -> SEL0 &= ~0x12;
    P1 -> SEL1 &= ~0x12;
    P1 -> DIR  &= ~0x12;
    P1 -> REN  |=  0x12;

    // Initializing the BGR led
    P2 -> SEL0 &= ~0x07; // BGR LED pins are
    P2 -> SEL1 &= ~0x07; // P2.2-0;
    P2 -> DIR |= 0x07;

    // Starting color
    int Color = 1;

    // Set the starting color
    P2 -> OUT |= Color;
    P2 -> OUT &= Color;

    // Main loop
    while(1){
        // Read in the switch values
        uint8_t switches = readSwitches();

        // If both switches are pressed
        if(switches == 0x00){
            // Delay for 10ms
            Clock_Delay1ms(10);

            // Increment to change the color
            Color++;

            // Set the color
            P2 -> OUT |= Color;
            P2 -> OUT &= Color;

            // Wait until at least one switch is released
            while(readSwitches() == 0x00){
                Clock_Delay1ms(1);
            }
        }
    }
}


// readSwitches(void)
// Outputs the 2 pins from the P1->IN register that corresponds to the state of the switches
uint8_t readSwitches(void){
    return (uint8_t)(P1->IN & 0x12);
}

