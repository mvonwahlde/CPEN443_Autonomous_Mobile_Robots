#include "msp.h"
#include "AP.h"
#include "CortexM.h"
#include "Clock.h"
#include "RobotLights.h"
#include "Motor.h"
#include "ADC14.h"

#include <stdint.h>
#include <math.h>

// Distances for Object Ahead notifications
#define MIN_DISTANCE_MM 90
#define HYSTERESIS_DISTANCE_MM 20
#define MAX_DISTANCE_MM (MIN_DISTANCE_MM + HYSTERESIS_DISTANCE_MM)

// Characters to represent the distance sensors (each has a different formula)
#define RIGHT_DISTANCE_SENSOR    'r'
#define CENTER_DISTANCE_SENSOR   'c'
#define LEFT_DISTANCE_SENSOR     'l'

// Function Prototypes/Declarations (for empty functions)
void setDirection(void){};
void setDutyCycle(void);
void getRightDist(void){};
void getCenterDist(void){};
void getLeftDist(void){};
void objectAheadStatus(void){};
void computeDistances(uint32_t leftADC, uint32_t *leftDist, uint32_t centerADC, uint32_t *centerDist, uint32_t rightADC, uint32_t *rightDist);
uint32_t computeDistance(uint32_t adcReading, char side);


// Global Variables that are changed over BLE
// Write
uint8_t direction = 0x00;
uint16_t dutyCycle = 0x0000;

// Read
uint32_t rightDist = 0x00000000;
uint32_t centerDist = 0x00000000;
uint32_t leftDist = 0x00000000;

// Notify
uint8_t objectAhead = 0x00;


// main.c
void main(void){
    volatile int r;

    // Stop Watchdog Timer
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;

    // Initializations
    Clock_Init48MHz();
    ADC0_InitSWTriggerCh17_14_16();
    Motor_Init();
    MvtLED_Init();
    r = AP_Init();

    // Stop the robot initially
    Motor_Stop();
    Front_Lights_OFF();
    Back_Lights_OFF();

    // Variables for the distances sensors
    uint32_t leftADC, centerADC, rightADC;
    uint8_t prevObjectAhead = 0x00;

    // Creating a GATT Service
    AP_AddService(0xFFF0);

    // Write characteristics
    AP_AddCharacteristic(0xFFF1,1,&direction,0x02,0x08,"direction",0,&setDirection);
    AP_AddCharacteristic(0xFFF2,2,&dutyCycle,0x02,0x08,"dutyCycle",0,&setDutyCycle);

    // Read characteristics
    AP_AddCharacteristic(0xFFF3,4,&rightDist,0x01,0x02,"rightDist",&getRightDist,0);
    AP_AddCharacteristic(0xFFF4,4,&centerDist,0x01,0x02,"centerDist",&getCenterDist,0);
    AP_AddCharacteristic(0xFFF5,4,&leftDist,0x01,0x02,"leftDist",&getLeftDist,0);

    // Notification for an object ahead
    AP_AddNotifyCharacteristic(0xFFF6, 1, &objectAhead, "Object Ahead", &objectAheadStatus); // CCCD = 0

    // Register the service start advertising
    AP_RegisterService();
    AP_StartAdvertisement();

    // Main loop
    while(1){
        // Get distances from each sensor
        ADC_In17_14_16(&leftADC, &centerADC, &rightADC);
        computeDistances(leftADC, &leftDist, centerADC, &centerDist, rightADC, &rightDist);

        // Wake up the BLE module
        AP_BackgroundProcess();

        // Check for object ahead
        if(leftDist <= MIN_DISTANCE_MM
           || centerDist <= MIN_DISTANCE_MM
           || rightDist <= MIN_DISTANCE_MM){
            objectAhead = 1;
        } else
        if(leftDist >= MAX_DISTANCE_MM
           || centerDist >= MAX_DISTANCE_MM
           || rightDist >= MAX_DISTANCE_MM){
            objectAhead = 0;
        }

        // Check if notification needs to be sent
        if((objectAhead && !prevObjectAhead) || (!objectAhead && prevObjectAhead)){
            // If there is an object ahead, set the direction to be stopped
            if(objectAhead)
                direction = 0x00;

            // Send the notification
            AP_SendNotification(0);
        }

        // Set the motors and light depending on the direction
        if(direction == 0x00){
            // Stop: turn off motors and lights
            Motor_Stop();
            Front_Lights_OFF();
            Back_Lights_OFF();

        } else if(direction == 0x0F){
            // Forwards
            if(objectAhead){
                // If there is an object ahead, stop and send notification
                Motor_Stop();
                Front_Lights_ON();
                Back_Lights_OFF();
                direction = 0x00;
                AP_SendNotification(0);
            } else {
                // Go forwards and turn light on
                Motor_Forward(dutyCycle, dutyCycle);
                Front_Lights_ON();
                Back_Lights_OFF();
            }

        } else if(direction == 0x0B){
            // Back up for 50ms
            Motor_Backward(dutyCycle, dutyCycle);
            Front_Lights_OFF();
            Back_Lights_ON();
            Clock_Delay1ms(50);
            direction = 0x00;

        } else if(direction == 0x0A){
            // Spin left for 50ms
            Motor_Left(dutyCycle, dutyCycle);
            Front_Lights_ON();
            Back_Lights_OFF();
            Clock_Delay1ms(50);
            direction = 0x00;

        } else if(direction == 0x0C){
            // Spin right for 50ms
            Motor_Right(dutyCycle, dutyCycle);
            Front_Lights_ON();
            Back_Lights_OFF();
            Clock_Delay1ms(50);
            direction = 0x00;

        }

        // Set the previous object ahead to the current
        prevObjectAhead = objectAhead;

        Clock_Delay1ms(10);
    }
}


void setDutyCycle(void){
    // dutyCycle
    if(dutyCycle > 7500){
        dutyCycle = 7500;
    }
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
