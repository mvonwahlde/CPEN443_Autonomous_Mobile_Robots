#include "msp.h"
#include "Clock.h"
#include "BumpInt.h"
#include "Motor.h"
#include "RobotLights.h"

#define FORWARDS_SPEED      5000 /* Forwards duty cycle (out of 15000) */
#define BACKWARDS_SPEED     2000 /* Backwards duty cycle (out of 15000) */
#define TURNING_SPEED       2000 /* Spinning duty cycle (out of 15000) */

#define BACKWARDS_TIME      1500 /* Time in ms to delay when backing up */
#define TURNING_TIME        1500 /* Time in ms to delay when turning */
#define FRONT_EXTRA_TIME    1000 /* Extra time to wait when the front is hit */

#define FRONT_RIGHT_BUMPER  2    /* Front right bumper number */
#define FRONT_LEFT_BUMPER   3    /* Front left bumper number */


// User-defined ISR for the bumpers
void HandleCollision(uint8_t ISR_BumpData);


/**
 * main.c
 */
void main(void)
{
    // Initialize the clock and stop the watchdog timer
    Clock_Init48MHz();
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;

	// Initialize the motors, LEDs, and the bumpers
	Motor_Init();
	MvtLED_Init();
	BumpInt_Init(&HandleCollision); // Pass the function in from main

	// Move the robot forwards and turn on the front lights
	Motor_Forward(FORWARDS_SPEED, FORWARDS_SPEED);
	Front_Lights_ON();

	// Continue moving forwards until an interrupt occurs
	while(1){
	    Clock_Delay1ms(25);
	}
}


// Handle Collision
// Bumper Interrupt Service Routine
// Input: The data from the bumpers
void HandleCollision(uint8_t ISR_BumpData){
    // Store the bumper data
    uint8_t Bump_Data = ISR_BumpData;

    // Containers
    uint8_t left = 0;      // How many of the left bumpers were hit
    uint8_t right = 0;     // How many of the right bumpers were hit
    uint8_t hit_front = 0; // Were either of the front two bumpers hit?

    int i;

    // Check which bumpers were hit
    for(i = 0; i < 6; i++){
        // Check if the current bumper was hit
        if( (Bump_Data & (0x01 << i)) == 0 ){
            if(i < 3){ // Right Side
                if(i == FRONT_RIGHT_BUMPER)
                    hit_front = 1;
                right++;
            } else { // Left Side
                if(i == FRONT_LEFT_BUMPER)
                    hit_front = 1;
                left++;
            }
        }
    }

    // Move the robot backwards
    Motor_Backward(BACKWARDS_SPEED, BACKWARDS_SPEED);
    Front_Lights_OFF();
    Back_Lights_ON();
    Clock_Delay1ms(BACKWARDS_TIME);
    Back_Lights_OFF();

    // Determined direction to turn the robot
    if(hit_front > 0) {
        // Turn left for an extended period of time
        Motor_Left(TURNING_SPEED, TURNING_SPEED);
        Clock_Delay1ms(FRONT_EXTRA_TIME);
    } else if(left >= right) {
        // If left side was hit, turn right
        Motor_Right(TURNING_SPEED, TURNING_SPEED);
    } else {
        // If the right side was hit, turn left
        Motor_Left(TURNING_SPEED, TURNING_SPEED);
    }

    // Wait, then move the robot forwards
    Clock_Delay1ms(TURNING_TIME);
    Motor_Forward(FORWARDS_SPEED, FORWARDS_SPEED);
    Front_Lights_ON();
}
