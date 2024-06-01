// By John Tadrous
// 02/12/2023
// Motor movement controller by tachometer measured steps

#include "msp.h"
#include "Clock.h"
#include "Motor.h"
#include "Tachometer.h"
#include "Precision_Moves.h"
#include "RobotLights.h"


#define START_DELAY_MS    1000  /* Delay to set the robot down (milliseconds)  */
#define FORWARD_SPEED     600   /* Speed of the robot during tests (0.1 RPM)   */
#define STAR_SIDE_LENGTH  20    /* Side length of the star shape (centimeters) */
#define CIRCLE_RADIUS     40    /* Radius of the circle (centimeters)          */


// main.c - Program entry point
void main(void){
    // Stop watchdog timer
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;

    // Initializations
    Clock_Init48MHz();
    Motor_Init();
    MvtLED_Init();
    Tachometer_Init();
    Front_Lights_OFF();
    Back_Lights_OFF();

    // Slight delay before starting
    Clock_Delay1ms(START_DELAY_MS);

    // ---------- Tests ----------
    Motor_Precision_CircleCCW(FORWARD_SPEED, CIRCLE_RADIUS);
    //Motor_Precision_CircleCW(FORWARD_SPEED, CIRCLE_RADIUS);
    //Motor_Precision_StarCCW(FORWARD_SPEED, STAR_SIDE_LENGTH);
    // ------ End of Tests -------

    // Infinite Loop
    while(1){
        Clock_Delay1ms(START_DELAY_MS);
    }
}
