// An API providing robot movement functions with precision using the tachometer
// By John Tadrous
// Created 02/14/2023
#include "Precision_Moves.h"


// This function turns the robot right for a given number of steps (desiredSteps) to realize
// a certain angular turn about the center of graviy. The left and right wheels needs to turn at the
// same speed in opposite directions
// Input speed is the PWM duty cycle from 0 to 14999
ret_t Motor_Precision_Right(int16_t speed, int32_t desiredSteps, uint8_t distanceInterrupt){
    // Check for invalid inputs
    if(speed < MIN_DUTY_CYCLE){return DRIVING_INTERRUPTED;}
    if(speed > MAX_DUTY_CYCLE){return DRIVING_INTERRUPTED;}

    // Whether or not the robot reached the destination
    ret_t hasReached = REACHED_DESTINATION;

    // Containers for each distance sensor's distance
    uint32_t leftDist, centerDist, rightDist;

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

    // Turn on the lights
    Front_Lights_ON();

    // Proportional controller
    while( ((leftSteps - leftInitSteps) < desiredSteps) && ((rightInitSteps - rightSteps) < desiredSteps) ){
        // If interrupt is turned on
        if(distanceInterrupt == DO_INTERRUPT){
            // Get distances from each sensor
            Distance_GetDistances(&leftDist, &centerDist, &rightDist);

            // If the left distance sensor is open, stop
            if(leftDist > MAX_DISTANCE_MM && centerDist > MAX_DISTANCE_MM){
                hasReached = DRIVING_INTERRUPTED;
                break;
            }
        }

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

    return hasReached;
}


// This function turns the robot left for a given number of steps (desiredSteps) to realize
// a certain angular turn about the center of graviy. The left and right wheels needs to turn at the
// same speed in opposite directions
// Input speed is the PWM duty cycle from 0 to 14999
ret_t Motor_Precision_Left(int16_t speed, int32_t desiredSteps, uint8_t distanceInterrupt){
    // Check for invalid inputs
    if(speed < MIN_DUTY_CYCLE){return DRIVING_INTERRUPTED;}
    if(speed > MAX_DUTY_CYCLE){return DRIVING_INTERRUPTED;}

    // Whether or not the robot reached the destination
    ret_t hasReached = REACHED_DESTINATION;

    // Containers for each distance sensor's distance
    uint32_t leftDist, centerDist, rightDist;

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

    // Turn on the lights
    Front_Lights_ON();

    // Proportional controller
    while( ((rightSteps - rightInitSteps) < desiredSteps) && ((leftInitSteps - leftSteps) < desiredSteps) ){
        // If interrupt is turned on
        if(distanceInterrupt == DO_INTERRUPT){
            // Get distances from each sensor
            Distance_GetDistances(&leftDist, &centerDist, &rightDist);

            // If the left distance sensor is open, stop
            if(rightDist > MAX_DISTANCE_MM && centerDist > MAX_DISTANCE_MM){
                hasReached = DRIVING_INTERRUPTED;
                break;
            }
        }

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

    return hasReached;
}


// This function utilizes the tachometer and a PI controller to maintain  wheel
// speed at desired rpm. It returns whenever left or right wheel reaches its target steps
// RPM input is given in 0.1 RPM
ret_t Motor_Forward_RPM(uint16_t leftRPM, uint16_t rightRPM, int32_t desiredLSteps, int32_t desiredRSteps){
    // Whether or not the robot reached the destination
    ret_t hasReached = REACHED_DESTINATION;

    // Containers for each distance sensor's distance
    uint32_t leftDist, centerDist, rightDist;

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

    // Turn on the front lights
    Front_Lights_ON();

    // Proportional Integral Controller
    while( ((leftSteps - leftInitSteps) < desiredLSteps) && ((rightSteps - rightInitSteps) < desiredRSteps) ){
        // Get distances from each sensor
        Distance_GetDistances(&leftDist, &centerDist, &rightDist);

        // Check if any of the sensors are within the minimum distance
        if(leftDist < MIN_DISTANCE_MM/2 || centerDist < MIN_DISTANCE_MM || rightDist < MIN_DISTANCE_MM/2){
            // Object detected
            hasReached = DRIVING_INTERRUPTED;
            break;
        }

        // Get the current steps and time
        Tachometer_Get_SpaceTime(&leftSteps, &rightSteps, &leftTime, &rightTime);

        // Check if enough time has passed
        uint8_t leftMoved = 0;
        uint8_t rightMoved = 0;
        if( (leftTime != prevLeftTime) || leftRPM == 0 ){leftMoved = 1;}
        if( (rightTime != prevRightTime) || rightRPM == 0 ){rightMoved = 1;}

        // If enough time has passed for the left side, continue with the PI controller
        if(leftMoved){
            // Calculate the current RPM values and error
            leftActRPM = COMPUTE_RPM(leftSteps, prevLeftSteps, leftTime, prevLeftTime);
            errorL = leftRPM - leftActRPM;

            // Compute integral terms and limit to prevent wind up
            leftIntegral += KI_FORWARD(errorL);
            if(leftIntegral > INTEGRAL_MAX){leftIntegral = INTEGRAL_MAX;}
            if(leftIntegral < INTEGRAL_MIN){leftIntegral = INTEGRAL_MIN;}

            // Set the update for PWM and limit
            updateLeft += KP_FORWARD(errorL) + leftIntegral;
            if(updateLeft < MIN_DUTY_CYCLE){updateLeft = MIN_DUTY_CYCLE;}
            if(updateLeft > MAX_DUTY_CYCLE){updateLeft = MAX_DUTY_CYCLE;}

            // Set all previous values for the left side
            prevLeftSteps = leftSteps;
            prevLeftTime = leftTime;
        }

        // If enough time has passed for the right side, continue with the PI controller
        if(rightMoved){
            // Calculate the current RPM values and compute error
            rightActRPM = COMPUTE_RPM(rightSteps, prevRightSteps, rightTime, prevRightTime);
            errorR = rightRPM - rightActRPM;

            // Compute integral terms and limit to prevent wind up
            rightIntegral += KI_FORWARD(errorR);
            if(rightIntegral > INTEGRAL_MAX){rightIntegral = INTEGRAL_MAX;}
            if(rightIntegral < INTEGRAL_MIN){rightIntegral = INTEGRAL_MIN;}

            // Set the update for PWM and limit
            updateRight += KP_FORWARD(errorR) + rightIntegral;
            if(updateRight < MIN_DUTY_CYCLE){updateRight = MIN_DUTY_CYCLE;}
            if(updateRight > MAX_DUTY_CYCLE){updateRight = MAX_DUTY_CYCLE;}

            // Set all previous values for the right side
            prevRightSteps = rightSteps;
            prevRightTime = rightTime;
        }

        // Set the new duty cycles
        Motor_Forward(updateLeft, updateRight);

        Clock_Delay1ms(FORWARD_DELAY_MS);
    }

    // Active braking
    Motor_Backward(updateLeft, updateRight);
    Front_Lights_OFF();
    Back_Lights_ON();
    Clock_Delay1ms(ACTIVE_BRAKING_DELAY_MS);
    Motor_Stop();
    Back_Lights_OFF();

    return hasReached;
}
