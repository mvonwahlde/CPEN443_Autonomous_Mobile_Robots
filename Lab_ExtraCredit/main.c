#include "msp.h"
#include "Clock.h"
#include "Motor.h"
#include "RobotLights.h"
#include "ADC14.h"
#include "Distance.h"


typedef uint8_t bool;

///////////////////////////////////////////////////////////////////////////////////////
// Definitions
///////////////////////////////////////////////////////////////////////////////////////

#define DIST_THRESHOLD 200
#define FOLLOW_DISTANCE 100
#define ACCEPTABLE_ERROR 20
#define LOOP_DELAY 10
#define SPINNING_SPEED 3000
#define FORWARD_SPEED  2000
#define BACKWARD_SPEED 2000
#define TRUE 1
#define DUTY_LOWER_LIMIT 0
#define DUTY_UPPER_LIMIT 6000

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
    MvtLED_Init();
    Front_Lights_OFF();
    Back_Lights_OFF();

    uint32_t leftDist, centerDist, rightDist;
    bool leftObject, centerObject, rightObject;

    // Main loop
    while(TRUE){
        Distance_GetDistances(&leftDist, &centerDist, &rightDist);

        leftObject = leftDist < DIST_THRESHOLD;
        centerObject = centerDist < DIST_THRESHOLD;
        rightObject = rightDist < DIST_THRESHOLD;

        // find an object to follow;
        if(leftObject || centerObject || rightObject){
            // Find minimum to follow
            if(centerObject && (centerDist <= leftDist && centerDist <= rightDist)){
                // Follow center object
                if(centerDist <= FOLLOW_DISTANCE - ACCEPTABLE_ERROR){
                    // Backwards
                    int32_t backSpeed = BACKWARD_SPEED + BACKWARD_SPEED * (FOLLOW_DISTANCE - centerDist) / 30;
                    if(backSpeed < DUTY_LOWER_LIMIT){backSpeed = DUTY_LOWER_LIMIT;}
                    if(backSpeed > DUTY_UPPER_LIMIT){backSpeed = DUTY_UPPER_LIMIT;}
                    Motor_Backward(backSpeed, backSpeed);
                    Front_Lights_OFF();
                    Back_Lights_ON();
                } else if(centerDist >= FOLLOW_DISTANCE + ACCEPTABLE_ERROR){
                    // Forwards
                    int32_t frontSpeed = FORWARD_SPEED + FORWARD_SPEED * (centerDist - FOLLOW_DISTANCE) / 50;
                    if(frontSpeed < DUTY_LOWER_LIMIT){frontSpeed = DUTY_LOWER_LIMIT;}
                    if(frontSpeed > DUTY_UPPER_LIMIT){frontSpeed = DUTY_UPPER_LIMIT;}
                    Motor_Forward(frontSpeed, frontSpeed);
                    Front_Lights_ON();
                    Back_Lights_OFF();

                } else {
                    // Stop
                    Motor_Stop();
                    Front_Lights_OFF();
                    Back_Lights_OFF();
                }

            } else if(leftObject && (leftDist < centerDist && leftDist < rightDist)){
                // Follow left object
                Motor_Left(SPINNING_SPEED, SPINNING_SPEED);
            } else if(rightObject && (rightDist < leftDist && rightDist < centerDist)){
                // Follow right object
                Motor_Right(SPINNING_SPEED, SPINNING_SPEED);
            }

            // Keep it a certain distance and turn towards it
        } else {
            Motor_Stop();
            Front_Lights_OFF();
            Back_Lights_OFF();
        }

        Clock_Delay1ms(LOOP_DELAY);
    }
}


// ---------- Pause ----------
// Debug function. Infinite loop to prevent the program from continuing
// Inputs: none
// Output: none
void Pause(void){
    while(TRUE){
        Clock_Delay1ms(LOOP_DELAY);
    }
}
