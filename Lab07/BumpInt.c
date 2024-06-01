// BumpInt.c
// Runs on MSP432, interrupt version
// Provide low-level functions that interface bump switches on the robot.
// Daniel Valvano and Jonathan Valvano
// July 11, 2019

// Edited by J. Tadrous 06/02/2022

#include <stdint.h>
#include "msp.h"
#include "CortexM.h" // Global Interrupt Control


void(*rTask)(uint8_t); // Globalizing input function







void BumpInt_Init(void(*task)(uint8_t)){
    // Initialize Bump sensors
    // Make six Port 4 pins inputs
    // Activate interface pullup
    // pins 7,6,5,3,2,0 -> 0xED
    // Interrupt on falling edge (on touch)

    rTask = task;

    DisableInterrupts();
    P4->SEL0 &= ~0xED;
    P4->SEL1 &= ~0xED;
    P4->DIR &= ~0xED;
    P4->REN |= 0xED;
    P4->OUT |= 0xED;
    P4->IES |= 0xED;
    P4->IFG &= ~0xED;
    P4->IE |= 0xED;
    NVIC->IP[9] = (NVIC->IP[9]&0xFF00FFFF) | 0x00400000;
    NVIC->ISER[1] |= 0x00000040;
    EnableInterrupts();
}


uint8_t Bump_Read(void){
    // Read current state of 6 switches
    // Returns a 6-bit positive logic result (0 to 63)
    // bit 5 Bump5
    // bit 4 Bump4
    // bit 3 Bump3
    // bit 2 Bump2
    // bit 1 Bump1
    // bit 0 Bump0

    uint8_t data = P4->IN;
    uint8_t ret = 0;

    ret |= (data&0x01);      // Bump0
    ret |= (data&0x0C) >> 1; // Bump1&2
    ret |= (data&0xE0) >> 2; // Bump3,4,&5

    return ret;
}
// we do not care about critical section/race conditions
// triggered on touch, falling edge
void PORT4_IRQHandler(void){
    rTask(Bump_Read()); // Execute task from high-level software
    P4 -> IFG &= ~0xED; // ACK all
}

