// An API providing robot movement functions with precision using the tachometer
// By John Tadrous
// Created 02/14/2023
#include <stdio.h>
#include "msp.h"
#include "Clock.h"
#include "Motor.h"
#include "Tachometer.h"
#include "Precision_Moves.h"
#include "RobotLights.h"


// Make a CCW circle with provided radius and RPM speed. Minimum radius is 7 cm. Min speed is 35 RPM
// Input speed is in 0.1 RPM
void Motor_Precision_CircleCCW(uint16_t speed, uint16_t radius){
    // Unallowed inputs
    if(radius < MIN_RADIUS || speed < MIN_SPEED){
        return;
    }

    // Calculate the different radiuses for inner wheel, CoG, and outer wheel
    uint16_t R  = CM_TO_MM(radius);
    uint16_t Ri = INNER_RADIUS(R);
    uint16_t Ro = OUTER_RADIUS(R);

    // Calculate the steps that the left and right wheels need to travel
    float L = CIRCUMFERENCE(R);
    int32_t desiredLSteps = DISTANCE_TO_STEPS( (int32_t)(L * Ri / R) );
    int32_t desiredRSteps = DISTANCE_TO_STEPS( (int32_t)(L * Ro / R) );

    // If left is zero, set high and rely on the right steps
    if(desiredLSteps == 0){
        desiredLSteps = desiredRSteps;
    }

    // Calculate the RPM for the left and right side
    uint16_t leftRPM = speed * Ri / R;
    uint16_t rightRPM = speed * Ro / R;

    // Input the RPMs and desired steps to drive in a circle
    Motor_Forward_RPM(leftRPM, rightRPM, desiredLSteps, desiredRSteps);
}


// Make a CW circle with provided radius and RPM speed. Minimum radius is 7 cm. Min speed is 35 RPM
// Input speed is in 0.1 RPM
void Motor_Precision_CircleCW(uint16_t speed, uint16_t radius){
    // Unallowed inputs
    if(radius < MIN_RADIUS || speed < MIN_SPEED){
        return;
    }

    // Calculate the different radiuses for inner wheel, CoG, and outer wheel
    uint16_t R  = CM_TO_MM(radius);
    uint16_t Ri = INNER_RADIUS(R);
    uint16_t Ro = OUTER_RADIUS(R);

    // Calculate the steps that the left and right wheels need to travel
    float L = CIRCUMFERENCE(R);
    int32_t desiredRSteps = DISTANCE_TO_STEPS( (int32_t)(L * Ri / R) );
    int32_t desiredLSteps = DISTANCE_TO_STEPS( (int32_t)(L * Ro / R) );

    // If left is zero, set high and rely on the right steps
    if(desiredRSteps == 0){
        desiredRSteps = desiredLSteps;
    }

    // Calculate the RPM for the left and right side
    uint16_t rightRPM = speed * Ri / R;
    uint16_t leftRPM = speed * Ro / R;

    // Input the RPMs and desired steps to drive in a circle
    Motor_Forward_RPM(leftRPM, rightRPM, desiredLSteps, desiredRSteps);
}


// Make a CCW Pentagram with provided side length and RPM speed in 0.1 RPM. Minimum side length is 20 cm. Min speed is 40 RPM
void Motor_Precision_StarCCW(uint16_t speed, uint16_t sideLength){
    // Unallowed inputs
    if(sideLength < MIN_SIDE_LENGTH || speed < MIN_STAR_SPEED){
        return;
    }

    // Calculate the steps needed for each side length
    int32_t forwardSteps = DISTANCE_TO_STEPS(CM_TO_MM(sideLength));

    // Drive in the shape of a star
    int i;
    for(i = 0; i < NUM_STAR_POINTS; i++){
        Motor_Forward_RPM(speed, speed, forwardSteps, forwardSteps);
        Clock_Delay1ms(STAR_DELAY_MS);
        Motor_Precision_Left(STAR_SPINNING_SPEED, ANGLE_TO_STEPS(180 - STAR_INTERIOR_ANGLE));
        Clock_Delay1ms(STAR_DELAY_MS);
        Motor_Forward_RPM(speed, speed, forwardSteps, forwardSteps);
        Clock_Delay1ms(STAR_DELAY_MS);
        Motor_Precision_Right(STAR_SPINNING_SPEED, ANGLE_TO_STEPS(180 - STAR_EXTERIOR_ANGLE));
        Clock_Delay1ms(STAR_DELAY_MS);
    }
}


// This function turns the robot right for a given number of steps (desiredSteps) to realize
// a certain angular turn about the center of graviy. The left and right wheels needs to turn at the
// same speed in opposite directions
// Input speed is the PWM duty cycle from 0 to 14999
void Motor_Precision_Right(int16_t speed, int32_t desiredSteps){
    // Check for invalid inputs
    if(speed < MIN_DUTY_CYCLE){return;}
    if(speed > MAX_DUTY_CYCLE){return;}

    // Containers to hold initial steps and current steps
    int32_t leftSteps, rightSteps;
    int32_t leftInitSteps, rightInitSteps;

    // Containers for the error and duty cycles
    int32_t error;
    int32_t updateLeft = speed, updateRight = speed;

    // Get the initial steps of the tachometer
    Tachometer_Get_Steps(&leftInitSteps, &rightInitSteps);
    leftSteps = leftInitSteps;
    rightSteps = rightInitSteps;

    // Start spinning the motors
    Motor_Right(updateLeft, updateRight);
    Front_Lights_ON();

    // Proportional controller
    while( ((leftSteps - leftInitSteps) < desiredSteps) && ((rightInitSteps - rightSteps) < desiredSteps) ){
        // Get current steps and compute error
        Tachometer_Get_Steps(&leftSteps, &rightSteps);
        error = (leftSteps - leftInitSteps) - (rightInitSteps - rightSteps);

        // Set the new duty cycles based on the proportional term
        updateLeft = speed - KP_SPIN(error);
        updateRight = speed + KP_SPIN(error);

        // Limit the update values to min and max duty cycles
        if(updateLeft < MIN_DUTY_CYCLE){updateLeft = MIN_DUTY_CYCLE;}
        if(updateLeft > MAX_DUTY_CYCLE){updateLeft = MAX_DUTY_CYCLE;}
        if(updateRight < MIN_DUTY_CYCLE){updateRight = MIN_DUTY_CYCLE;}
        if(updateRight > MAX_DUTY_CYCLE){updateRight = MAX_DUTY_CYCLE;}

        // Set the new duty cycles and delay
        Motor_Right(updateLeft, updateRight);
        Clock_Delay1ms(SPIN_DELAY_MS);
    }

    // Active braking
    Motor_Left(updateLeft, updateRight);
    Clock_Delay1ms(ACTIVE_BRAKING_DELAY_MS);
    Motor_Stop();
    Front_Lights_OFF();
}


// This function turns the robot left for a given number of steps (desiredSteps) to realize
// a certain angular turn about the center of graviy. The left and right wheels needs to turn at the
// same speed in opposite directions
// Input speed is the PWM duty cycle from 0 to 14999
void Motor_Precision_Left(int16_t speed, int32_t desiredSteps){
    // Check for invalid inputs
    if(speed < MIN_DUTY_CYCLE){return;}
    if(speed > MAX_DUTY_CYCLE){return;}

    // Containers to hold initial steps and current steps
    int32_t leftSteps, rightSteps;
    int32_t leftInitSteps, rightInitSteps;

    // Containers for the error and duty cycles
    int32_t error;
    int32_t updateLeft = speed, updateRight = speed;

    // Get the initial steps of the tachometer
    Tachometer_Get_Steps(&leftInitSteps, &rightInitSteps);
    leftSteps = leftInitSteps;
    rightSteps = rightInitSteps;

    // Start spinning the motors
    Motor_Left(updateLeft, updateRight);
    Front_Lights_ON();

    // Proportional controller
    while( ((rightSteps - rightInitSteps) < desiredSteps) && ((leftInitSteps - leftSteps) < desiredSteps) ){
        // Get current steps and compute error
        Tachometer_Get_Steps(&leftSteps, &rightSteps);
        error = (leftInitSteps - leftSteps) - (rightSteps - rightInitSteps);

        // Set the new duty cycles based on the proportional term
        updateLeft = speed - KP_SPIN(error);
        updateRight = speed + KP_SPIN(error);

        // Limit the update values to min and max duty cycles
        if(updateLeft < MIN_DUTY_CYCLE){updateLeft = MIN_DUTY_CYCLE;}
        if(updateLeft > MAX_DUTY_CYCLE){updateLeft = MAX_DUTY_CYCLE;}
        if(updateRight < MIN_DUTY_CYCLE){updateRight = MIN_DUTY_CYCLE;}
        if(updateRight > MAX_DUTY_CYCLE){updateRight = MAX_DUTY_CYCLE;}

        // Set the new duty cycles and delay
        Motor_Left(updateLeft, updateRight);
        Clock_Delay1ms(SPIN_DELAY_MS);
    }

    // Active braking
    Motor_Right(updateLeft, updateRight);
    Clock_Delay1ms(ACTIVE_BRAKING_DELAY_MS);
    Motor_Stop();
    Front_Lights_OFF();
}


// This function utilizes the tachometer and a PI controller to maintain  wheel
// speed at desired rpm. It returns whenever left or right wheel reaches its target steps
// RPM input is given in 0.1 RPM
void Motor_Forward_RPM(uint16_t leftRPM, uint16_t rightRPM, int32_t desiredLSteps, int32_t desiredRSteps){
    // Containers for steps and times
    int32_t leftSteps, rightSteps;
    int32_t prevLeftSteps, prevRightSteps;
    int32_t leftInitSteps, rightInitSteps;
    uint32_t leftTime, rightTime;
    uint32_t prevLeftTime, prevRightTime;
    uint32_t leftInitTime, rightInitTime;

    // Containers for RPM, integral terms, update values, and errors
    int32_t errorL, errorR;
    int32_t leftActRPM = 0, rightActRPM = 0;
    int32_t leftIntegral = 0, rightIntegral = 0;
    int32_t updateLeft = RPM_TO_DUTY_EST(leftRPM), updateRight = RPM_TO_DUTY_EST(rightRPM);

    // Get the initial steps and time, then set appropriate values
    Tachometer_Get_SpaceTime(&leftInitSteps, &rightInitSteps, &leftInitTime, &rightInitTime);
    leftSteps = leftInitSteps;
    prevLeftSteps = leftInitSteps;
    rightSteps = rightInitSteps;
    prevRightSteps = rightInitSteps;
    prevLeftTime = leftInitTime;
    prevRightTime = rightInitTime;

    // Start spinning the motors
    Motor_Forward(updateLeft, updateRight);
    Front_Lights_ON();

    // Proportional Integral Controller
    while( ((leftSteps - leftInitSteps) < desiredLSteps) && ((rightSteps - rightInitSteps) < desiredRSteps) ){
        // Get the current steps and time
        Tachometer_Get_SpaceTime(&leftSteps, &rightSteps, &leftTime, &rightTime);

        // Check if enough time has passed
        uint8_t leftMoved = 0;
        uint8_t rightMoved = 0;
        if( (leftTime != prevLeftTime) || leftRPM == 0 ){leftMoved = 1;}
        if( (rightTime != prevRightTime) || rightRPM == 0 ){rightMoved = 1;}

        // If enough time has passed, continue with the PI controller
        if(leftMoved && rightMoved){
            // Calculate the current RPM values
            leftActRPM = COMPUTE_RPM(leftSteps, prevLeftSteps, leftTime, prevLeftTime);
            rightActRPM = COMPUTE_RPM(rightSteps, prevRightSteps, rightTime, prevRightTime);

            // Compute error
            errorL = leftRPM - leftActRPM;
            errorR = rightRPM - rightActRPM;

            // Compute integral terms and limit to prevent wind up
            leftIntegral += KI_FORWARD(errorL);
            rightIntegral += KI_FORWARD(errorR);
            if(leftIntegral > INTEGRAL_MAX){leftIntegral = INTEGRAL_MAX;}
            if(leftIntegral < INTEGRAL_MIN){leftIntegral = INTEGRAL_MIN;}
            if(rightIntegral > INTEGRAL_MAX){rightIntegral = INTEGRAL_MAX;}
            if(rightIntegral < INTEGRAL_MIN){rightIntegral = INTEGRAL_MIN;}

            // Set the update for PWMs
            updateLeft += KP_FORWARD(errorL) + leftIntegral;
            updateRight += KP_FORWARD(errorR) + rightIntegral;

            // Limit the update values to min and max duty cycles
            if(updateLeft < MIN_DUTY_CYCLE){updateLeft = MIN_DUTY_CYCLE;}
            if(updateLeft > MAX_DUTY_CYCLE){updateLeft = MAX_DUTY_CYCLE;}
            if(updateRight < MIN_DUTY_CYCLE){updateRight = MIN_DUTY_CYCLE;}
            if(updateRight > MAX_DUTY_CYCLE){updateRight = MAX_DUTY_CYCLE;}

            // Set the new duty cycles
            Motor_Forward(updateLeft, updateRight);

            // Set all previous values
            prevLeftSteps = leftSteps;
            prevRightSteps = rightSteps;
            prevLeftTime = leftTime;
            prevRightTime = rightTime;
        }

        Clock_Delay1ms(FORWARD_DELAY_MS);
    }

    // Active braking
    Motor_Backward(updateLeft, updateRight);
    Front_Lights_OFF();
    Back_Lights_ON();
    Clock_Delay1ms(ACTIVE_BRAKING_DELAY_MS);
    Motor_Stop();
    Back_Lights_OFF();
}
