#include "msp.h"
#include "Clock.h"
#include "Motor.h"
#include "Tachometer.h"
#include "RobotLights.h"
#include "Odometry.h"
#include "ADC14.h"


///////////////////////////////////////////////////////////////////////////////////////
// Definitions
///////////////////////////////////////////////////////////////////////////////////////

#define DESTINATION_X_POS   0    /* Destination's X position (mm) */
#define DESTINATION_Y_POS   1820 /* Destination's Y position (mm) */
#define DESTINATION_HEADING 90   /* Does not matter. Destination's heading (degrees) */

#define STARTING_X_POS   0       /* Robot's starting X position (mm) */
#define STARTING_Y_POS   0       /* Robot's starting Y position (mm) */
#define STARTING_HEADING 90      /* Robot's starting heading (degrees) */

#define START_DELAY_MS    1000   /* Delay before starting program (milliseconds)  */
#define DELAY_MS          250    /* Delay between each step in the main loop (milliseconds) */
#define MAX_SPIN_DEGREES  90     /* Maximum degrees to spin before attempting to move forwards (when going around an object) */


///////////////////////////////////////////////////////////////////////////////////////
// Main Program
///////////////////////////////////////////////////////////////////////////////////////

void Pause(void); /* Debug function */


// ---------- main ----------
// Program entry point
// Inputs: none
// Output: none
void main(void){
    // Stop watchdog timer
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;

    // Initializations
    Clock_Init48MHz();
    ADC0_InitSWTriggerCh17_14_16();
    Motor_Init();
    Tachometer_Init();
    MvtLED_Init();
    Front_Lights_OFF();
    Back_Lights_OFF();

    // Containers for use in main loop
    ret_t reachedDest;
    side_t spinDirection;

    // Coordinates for the robot and the destination
    Coordinates robot = {STARTING_X_POS, STARTING_Y_POS, STARTING_HEADING};
    const Coordinates destination = {DESTINATION_X_POS, DESTINATION_Y_POS, DESTINATION_HEADING};

    // Slight delay before starting
    Clock_Delay1ms(START_DELAY_MS);

    // Main loop
    while(TRUE){

        // Correct heading by spinning towards the destination
        Odometry_CorrectSpin(&robot, &destination);
        Clock_Delay1ms(DELAY_MS);

        // Move forward until robot reaches destination or is blocked
        reachedDest = Odometry_DriveForward(&robot, &destination);
        Clock_Delay1ms(DELAY_MS);

        // If the robot reached the destination, this function will not return
        Odometry_CheckFinished(reachedDest);

        // Robot reached an obstacle, so pick a direction to spin
        spinDirection = Odometry_PickSide(&robot, &destination);

        // Spin in the direction until the opposite sensor is not scanning the object
        Odometry_Spin(&robot, spinDirection, MAX_SPIN_DEGREES);
        Clock_Delay1ms(DELAY_MS);

        // Move forwards for a certain distance (return variable not used)
        Odometry_Forward(&robot);
        Clock_Delay1ms(DELAY_MS);
    }
}


// ---------- Pause ----------
// Debug function. Infinite loop to prevent the program from continuing
// Inputs: none
// Output: none
void Pause(void){
    while(TRUE){
        Clock_Delay1ms(INFINITE_LOOP_DELAY);
    }
}
