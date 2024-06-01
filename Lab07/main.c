#include "msp.h"
#include "Clock.h"
#include "ADC14.h"
#include "RobotLights.h"
#include "Motor.h"
#include "BumpInt.h"
#include <math.h>


// Constant for the square root of 2
#define SQRT2 (double)1.41421356

// Characters to represent the distance sensors (each has a different formula)
#define RIGHT_DISTANCE_SENSOR    'r'
#define CENTER_DISTANCE_SENSOR   'c'
#define LEFT_DISTANCE_SENSOR     'l'

// Bumper numbers for the front right and left bumpers
#define FRONT_RIGHT_BUMPER    2
#define FRONT_LEFT_BUMPER     3

// Three states for the robot: driving (mostly) straight, turning left, and turning right
#define NORMAL_STATE       0
#define SPIN_LEFT_STATE    1
#define SPIN_RIGHT_STATE   2

// Key Distances (mm)
#define MIN_DISTANCE             100 /* Minimum distance from the wall */
#define MAX_DISTANCE             140 /* Maximum distance from the wall */
#define START_TURN_DISTANCE      200 /* Distance that identifies a left turn */
#define MIN_FRONT_DISTANCE      (int)((double)MIN_DISTANCE * SQRT2 / 2.0) /* Distance from the front that identifies a right turn */
#define FORWARDS_DISTANCE        100 /* Distance to drive forwards before starting a left turn */
#define MIN_TARGET_DISTANCE      100 /* Minimum target center distance before making a left turn */

// Speeds for the normal state
#define FORWARDS_SPEED 3500
#define SLIGHT_TURN_FAST FORWARDS_SPEED
#define SLIGHT_TURN_SLOW FORWARDS_SPEED / 2

// Speeds for the left turn state
#define TURN_LEFT_SPEED_FAST FORWARDS_SPEED
#define TURN_LEFT_SPEED_SLOW FORWARDS_SPEED / 2

// Speed for the spinning right state
#define SPIN_RIGHT_SPEED FORWARDS_SPEED / 2

// Speeds and times for the bumper interrupts
#define BACKWARDS_SPEED FORWARDS_SPEED / 2
#define BACKWARDS_TIME 500
#define SPINNING_SPEED BACKWARDS_SPEED
#define SPINNING_TIME BACKWARDS_TIME

// Time in milliseconds between each sample
#define SAMPLE_TIME 10


typedef int state_t;

void HandleCollision(uint8_t ISR_BumpData);
void computeDistances(uint32_t leftADC, uint32_t *leftDist, uint32_t centerADC, uint32_t *centerDist, uint32_t rightADC, uint32_t *rightDist);
uint32_t computeDistance(uint32_t adcReading, char side);


/*
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
	BumpInt_Init(&HandleCollision);

	// Move the robot forwards and turn on the front lights
	Motor_Forward(FORWARDS_SPEED, FORWARDS_SPEED);
	Back_Lights_OFF();
	Front_Lights_ON();

	// Containers for the distance sensors' ADC readings and corresponding distances
	uint32_t leftADC, centerADC, rightADC;
	uint32_t leftDist, centerDist, rightDist;

	// Containers for the state, and some of their variables
	state_t state = NORMAL_STATE;
	int stage;
	int targetDist;
	int prevDist;

	// Main Loop
	while(1){
	    // Get distances from each sensor
	    ADC_In17_14_16(&leftADC, &centerADC, &rightADC);
	    computeDistances(leftADC, &leftDist, centerADC, &centerDist, rightADC, &rightDist);

	    // Check which state to enter (if already in another state continue with that one)
	    if( (rightDist > START_TURN_DISTANCE && state == NORMAL_STATE) || state == SPIN_LEFT_STATE ){
	        // State for making a left turn
	        if(state == NORMAL_STATE){
	            // If entering from the normal state, calculate a target to distance from the center
	            targetDist = centerDist - FORWARDS_DISTANCE;
	            if(targetDist < MIN_TARGET_DISTANCE)
	                targetDist = MIN_TARGET_DISTANCE;

	            // Start at the beginning of the left turn state
	            stage = 0;
	        }

	        state = SPIN_LEFT_STATE;

	    } else if( (centerDist < MIN_FRONT_DISTANCE && state == NORMAL_STATE) || state == SPIN_RIGHT_STATE ){
	        // State for making a right turn
	        if(state == NORMAL_STATE){
	            // If entering from the normal state, set necessary variables
	            prevDist = rightDist;
	            stage = 0;
	        }

	        state = SPIN_RIGHT_STATE;

	    } else {
	        // Normal state, check alignment with the wall
	        state = NORMAL_STATE;

	        if(rightDist < MIN_DISTANCE){
	            // Too close, slight turn right
	            Motor_Forward(SLIGHT_TURN_FAST, SLIGHT_TURN_SLOW);
	        } else if(rightDist > MAX_DISTANCE){
	            // Too far, slight turn left
	            Motor_Forward(SLIGHT_TURN_SLOW, SLIGHT_TURN_FAST);
	        } else {
	            // Perfect, drive straight
	            Motor_Forward(FORWARDS_SPEED, FORWARDS_SPEED);
	        }
	    }


	    // Check if in an alternate state
	    if(state == SPIN_LEFT_STATE){
	        // Turning left state
	        if(stage == 0){
	            // Move forwards for certain distance
	            if(centerDist < targetDist){
	                Motor_Stop();
	                stage = 1;
	            } else {
	                Motor_Forward(FORWARDS_SPEED, FORWARDS_SPEED);
	            }

	        } else if(stage == 1){
	            // Turn to the left until it is within range of the wall
	            if(rightDist < MAX_DISTANCE){
	                Motor_Stop();
	                state = NORMAL_STATE;
	            } else {
	                Motor_Forward(TURN_LEFT_SPEED_SLOW, TURN_LEFT_SPEED_FAST);
	            }
	        }

	    } else if(state == SPIN_RIGHT_STATE){
	        // Turning right state
	        if(stage == 0){
	            // Spin until the left distance increases
	            if((prevDist - (int)rightDist) < 0){
	                Motor_Stop();
	                stage = 1;
	            } else {
	                prevDist = rightDist;
	                Motor_Right(SPIN_RIGHT_SPEED, SPIN_RIGHT_SPEED);
	            }

	        } else if(stage == 1){
	            // Spin until left distance reaches minimum distance
	            if(rightDist >= MIN_DISTANCE){
	                Motor_Stop();
	                state = NORMAL_STATE;
	            } else {
	                Motor_Right(SPIN_RIGHT_SPEED, SPIN_RIGHT_SPEED);
	            }
	        }
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
