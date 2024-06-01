#include "msp.h"
#include "Clock.h"
#include "ADC14.h"
#include "RobotLights.h"
#include "Motor.h"
#include "BumpInt.h"
#include <math.h>


// Characters to represent the distance sensors (each has a different formula)
#define RIGHT_DISTANCE_SENSOR    'r'
#define CENTER_DISTANCE_SENSOR   'c'
#define LEFT_DISTANCE_SENSOR     'l'

// Different duty cycles for each movement
#define FORWARDS_SPEED            5000
#define BACKWARDS_SPEED           3000
#define SPINNING_SPEED            3000
#define SWERVING_SPEED_FAST       5000
#define SWERVING_SPEED_SLOW       0

// Different waiting times
#define SAMPLE_TIME               100
#define BACKWARDS_TIME            1000
#define SPINNING_TIME             1000

// Distance where it is on a collision course
#define COLLISION_DISTANCE_MM     100

// Bumper numbers for the front two
#define FRONT_RIGHT_BUMPER        2
#define FRONT_LEFT_BUMPER         3


void HandleCollision(uint8_t ISR_BumpData);
void computeDistances(uint32_t leftADC, uint32_t *leftDist, uint32_t centerADC, uint32_t *centerDist, uint32_t rightADC, uint32_t *rightDist);
uint32_t computeDistance(uint32_t adcReading, char side);


/**
 * main.c
 */
void main(void){
    // Stop watchdog timer
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;

    // Initializations
    Clock_Init48MHz();
    ADC0_InitSWTriggerCh17_14_16();
    MvtLED_Init();
    Motor_Init();
    BumpInt_Init(&HandleCollision); // Pass the function in from main

    // Move the robot forwards and turn on the front lights
    Motor_Forward(FORWARDS_SPEED, FORWARDS_SPEED);
    Front_Lights_ON();

    uint32_t leftADC, centerADC, rightADC;
    uint32_t leftDist, centerDist, rightDist;

    while(1){

        // Get distances from each sensor
        ADC_In17_14_16(&leftADC, &centerADC, &rightADC);
        computeDistances(leftADC, &leftDist, centerADC, &centerDist, rightADC, &rightDist);

        if(leftDist <= COLLISION_DISTANCE_MM && rightDist <= COLLISION_DISTANCE_MM){
            // Turn around
            Motor_Right(SPINNING_SPEED, SPINNING_SPEED);
            Clock_Delay1ms(SPINNING_TIME - SAMPLE_TIME); // 900ms + 100ms(later) = 1 second

        } else if(centerDist <= COLLISION_DISTANCE_MM){
            // Back up
            Front_Lights_OFF();
            Back_Lights_ON();
            Motor_Backward(BACKWARDS_SPEED, BACKWARDS_SPEED);
            Clock_Delay1ms(BACKWARDS_TIME/2);

            // Turn right
            Front_Lights_ON();
            Back_Lights_OFF();
            Motor_Right(SPINNING_SPEED, SPINNING_SPEED);
            Clock_Delay1ms(SPINNING_TIME - SAMPLE_TIME);

        } else if(rightDist <= COLLISION_DISTANCE_MM){
            // Swerve left
            Motor_Forward(SWERVING_SPEED_FAST, SWERVING_SPEED_SLOW);

        } else if(leftDist <= COLLISION_DISTANCE_MM){
            // Swerve right
            Motor_Forward(SWERVING_SPEED_SLOW, SWERVING_SPEED_FAST);

        } else {
            // Continue moving forwards
            Motor_Forward(FORWARDS_SPEED, FORWARDS_SPEED);
        }

        // Wait for the sample time
        Clock_Delay1ms(SAMPLE_TIME);
    }
}


// Handle Collision
// Bumper Interrupt Service Routine
// Input: The data from the bumpers
void HandleCollision(uint8_t ISR_BumpData){
    // Store the bumper data
    uint8_t Bump_Data = ISR_BumpData;

    // Containers
    uint32_t left = 0;      // How many of the left bumpers were hit
    uint32_t right = 0;     // How many of the right bumpers were hit
    uint32_t hit_front = 0; // Were either of the front two bumpers hit?

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
        Motor_Left(SPINNING_SPEED, SPINNING_SPEED);
        Clock_Delay1ms(SPINNING_TIME);
    } else if(left >= right) {
        // If left side was hit, turn right
        Motor_Right(SPINNING_SPEED, SPINNING_SPEED);
    } else {
        // If the right side was hit, turn left
        Motor_Left(SPINNING_SPEED, SPINNING_SPEED);
    }

    // Wait, then move the robot forwards
    Clock_Delay1ms(SPINNING_TIME);
    Motor_Forward(FORWARDS_SPEED, FORWARDS_SPEED);
    Front_Lights_ON();
}


// Compute all distances from the center
// Inputs: ADC values and pointers to store distances
void computeDistances(uint32_t leftADC, uint32_t *leftDist, uint32_t centerADC,
                      uint32_t *centerDist, uint32_t rightADC, uint32_t *rightDist){
    *leftDist = computeDistance(leftADC, LEFT_DISTANCE_SENSOR);
    *centerDist = computeDistance(centerADC, CENTER_DISTANCE_SENSOR);
    *rightDist = computeDistance(rightADC, RIGHT_DISTANCE_SENSOR);
}


// Formulas to compute each distance
// Inputs: ADC values for the distance and the corresponding side
uint32_t computeDistance(uint32_t adcReading, char side){
    double distanceMM;

    if(side == RIGHT_DISTANCE_SENSOR){
        distanceMM = 3.0*pow(10.0,6.0)*pow((double)adcReading,-1.116);
    } else if(side == CENTER_DISTANCE_SENSOR){
        distanceMM = 6.0*pow(10.0,6.0)*pow((double)adcReading,-1.182);
    } else { // side == LEFT_DISTANCE_SENSOR
        distanceMM = 3.0*pow(10.0,6.0)*pow((double)adcReading,-1.11);
    }

    return (uint32_t)round(distanceMM);
}
