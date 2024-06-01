#include "Odometry.h"


// ---------- Odometry_DriveForward ----------
// Drives forward until reaching the destination or an object is detected in the path
// Inputs: Coordinates* cur  - pointer to the robot's current coordinates
//         const Coordinates* dest - pointer to the destination's coordinates
// Output: ret_t - whether the robot reached the destination or detected an object
ret_t Odometry_DriveForward(Coordinates* cur, const Coordinates* dest){
    // Assuming that the robot is already facing the correct angle
    // Calculate the distance and number of steps to travel to the destination
    int32_t dy = dest->yPos - cur->yPos;
    int32_t dx = dest->xPos - cur->xPos;
    int32_t distance = sqrt(dy*dy + dx*dx);
    int32_t desiredSteps = DISTANCE_TO_STEPS(distance);

    // Get original steps
    int32_t leftInitSteps, rightInitSteps;
    Tachometer_Get_Steps(&leftInitSteps, &rightInitSteps);

    // Drive forwards for until the destination has been reached or until blocked by an object
    ret_t hasReached = Motor_Forward_RPM(LEFT_RPM, RIGHT_RPM, desiredSteps, desiredSteps);

    // Get the robot's new number of steps
    int32_t leftSteps, rightSteps;
    Tachometer_Get_Steps(&leftSteps, &rightSteps);

    // Calculate the distance traveled by the robot
    int32_t stepsTraveled = min(leftSteps - leftInitSteps, rightSteps - rightInitSteps);
    distance = STEPS_TO_DISTANCE(stepsTraveled);

    // Update the robot's current x and y positions
    cur->xPos += (int32_t)( (float)distance * cosd(cur->heading) );
    cur->yPos += (int32_t)( (float)distance * sind(cur->heading) );

    // Return whether or not the robot reached the destination
    return hasReached;
}


// ---------- Odometry_CheckFinished ----------
// If the robot has reached the destination, does not return
// Inputs: ret_t - whether the robot reached the destination or detected an object
// Output: none
void Odometry_CheckFinished(ret_t reachedDest){
    // If the robot reached the destination, enter an infinite loop
    if(reachedDest == REACHED_DESTINATION){
        while(TRUE){
            Clock_Delay1ms(INFINITE_LOOP_DELAY);
        }
    }
}


// ---------- Odometry_PickSide ----------
// Determines which was to spin the robot to avoid an obstacle
// Inputs: const Coordinates* cur - robot's current coordinates
//         const Coordinates* cur - destination's coordinates
// Output: side_t - the direction to spin the robot
side_t Odometry_PickSide(const Coordinates* cur, const Coordinates* dest){
    uint32_t leftDist, centerDist, rightDist;
    int32_t alpha;
    side_t chosenSide;

    // Get the distances for the sensors
    Distance_GetDistances(&leftDist, &centerDist, &rightDist);

    // Boolean for if the left and right sensors are detecting open air (no object near)
    uint8_t leftOpen = leftDist >= SENSOR_THRESHOLD_MM;
    uint8_t rightOpen = rightDist >= SENSOR_THRESHOLD_MM;

    // If the left sensor is open, spin left
    if(leftOpen && !rightOpen){
        return LEFT_SIDE;
    }

    // If the right sensor is open, spin right
    if(!leftOpen && rightOpen){
        return RIGHT_SIDE;
    }

    // If both sensors are open, spin whichever way is closest to the destination
    if(leftOpen && rightOpen){
        // Calculate the angle to the destination and choose that direction
        alpha = Odometry_CalculateAlpha(cur, dest);
        chosenSide = RIGHT_SIDE;
        if(alpha >= 0){
            chosenSide = LEFT_SIDE;
        }

        return chosenSide;
    }

    // If neither sensor are open, the decision is made based on two options
    // if(!leftOpen && !rightOpen){
    chosenSide = RIGHT_SIDE;
#ifdef TURN_TO_DESTINATION
    // Turns towards the destination
    alpha = Odometry_CalculateAlpha(cur, dest);
    if(alpha >= 0){
        chosenSide = LEFT_SIDE;
    }
#else
    // Turns toward whatever sensor is the furthest (has to spin the least distance)
    if(leftDist >= rightDist){
        chosenSide = LEFT_SIDE;
    }
#endif

    return chosenSide;
}


// ---------- Odometry_Spin ----------
// Spins until the opposite sensor is open (or the max number of degrees)
// Inputs: Coordinates* cur - robot's current coordinates
//         side_t direction - direction to spin (left or right)
//         int32_t maxDegrees - maximum number of degrees the robot can spin
// Output: none
void Odometry_Spin(Coordinates* cur, side_t direction, int32_t maxDegrees){
    // Get original steps
    int32_t leftInitSteps, rightInitSteps;
    Tachometer_Get_Steps(&leftInitSteps, &rightInitSteps);

    // Spin in the same direction as the input
    if(direction == LEFT_SIDE){
        Motor_Precision_Left(SPIN_DUTY, ANGLE_TO_STEPS(maxDegrees), DO_INTERRUPT);
    } else /*if(direction == RIGHT_SIDE)*/{
        Motor_Precision_Right(SPIN_DUTY, ANGLE_TO_STEPS(maxDegrees), DO_INTERRUPT);
    }

    // Get the new steps
    int32_t leftSteps, rightSteps;
    Tachometer_Get_Steps(&leftSteps, &rightSteps);

    // Calculate the angle that the robot spun
    int32_t stepsTraveled = max(leftSteps - leftInitSteps, rightSteps - rightInitSteps);
    int32_t angle = STEPS_TO_ANGLE(stepsTraveled);

    // If the robot spun right, the angle is negative
    if(direction == RIGHT_SIDE){
        angle *= -1;
    }

    // Add the angle to the robot's current heading and make sure it is between 0 and 360
    cur->heading += angle;
    cur->heading %= DEGREES_PER_REVOLUTION;
}


// ---------- Odometry_Forward ----------
// Drives for a set distance to move out of the way of an obstacle
// Inputs: Coordinates* cur - robot's current coordinates
// Output: none
void Odometry_Forward(Coordinates* cur){
    // Get original steps
    int32_t leftInitSteps, rightInitSteps;
    Tachometer_Get_Steps(&leftInitSteps, &rightInitSteps);

    // Calculate the desired number of steps and then move forwards that distance
    int32_t desiredSteps = DISTANCE_TO_STEPS(DRIVE_FORWARD_MM);
    Motor_Forward_RPM(LEFT_RPM, RIGHT_RPM, desiredSteps, desiredSteps);

    // Get the new number of steps
    int32_t leftSteps, rightSteps;
    Tachometer_Get_Steps(&leftSteps, &rightSteps);

    // Calculate the distance traveled
    int32_t stepsTraveled = min(leftSteps - leftInitSteps, rightSteps - rightInitSteps);
    int32_t distance = STEPS_TO_DISTANCE(stepsTraveled);

    // Update the robot's current x and y positions
    cur->xPos += (int32_t)( (float)distance * cosd(cur->heading) );
    cur->yPos += (int32_t)( (float)distance * sind(cur->heading) );
}


// ---------- Odometry_CorrectSpin ----------
// Spins the robot towards the destination
// Inputs: Coordinates* cur - robot's current coordinates
//         const Coordinates* dest - destination's coordinates
// Output: none
void Odometry_CorrectSpin(Coordinates* cur, const Coordinates* dest){
    // Get original steps
    int32_t leftInitSteps, rightInitSteps;
    Tachometer_Get_Steps(&leftInitSteps, &rightInitSteps);

    // Calculate alpha
    int32_t alpha = Odometry_CalculateAlpha(cur, dest);

    if(alpha > 0){
        Motor_Precision_Left(SPIN_DUTY, ANGLE_TO_STEPS(alpha), NO_INTERRUPT);
    } else if(alpha < 0){
        Motor_Precision_Right(SPIN_DUTY, ANGLE_TO_STEPS(alpha * -1), NO_INTERRUPT);
    } else {
        return;
    }

    // Get the new number of steps
    int32_t leftSteps, rightSteps;
    Tachometer_Get_Steps(&leftSteps, &rightSteps);

    // Calculate the angle spun
    int32_t stepsTraveled = max(leftSteps - leftInitSteps, rightSteps - rightInitSteps);
    int32_t angle = STEPS_TO_ANGLE(stepsTraveled);

    // If alpha is negative, the robot spun to the right and the angle is negative
    if(alpha < 0){
        angle *= -1;
    }

    // Update the robot's heading
    cur->heading += angle;
    cur->heading %= DEGREES_PER_REVOLUTION;
}


// ---------- Odometry_CalculateAlpha ----------
// Calculates the angle between the robot's heading and the destination
// Inputs: const Coordinates* cur - robot's current coordinates
//         const Coordinates* dest - destination's coordinates
// Output: int32_t - alpha, angle between the robot's heading and the destination
int32_t Odometry_CalculateAlpha(const Coordinates* cur, const Coordinates* dest){
    // Difference between the destination and robot's x and y coordinates
    int32_t dy = dest->yPos - cur->yPos;
    int32_t dx = dest->xPos - cur->xPos;

    // Compute alpha
    int32_t angle;
    if(dx == 0){
        // Edge cases
        if(dy >= 0){
            angle = 90;
        } else {
            angle = -90;
        }
    } else {
        // Compute the angle between the x axis vector and robot destination vector
        angle = (int32_t)RAD_TO_DEG( (atan2((float)dy, (float)dx)) );
    }

    // atan2 only returns in quadrants 1 and 4, so check if the angle is in quadrants 2 and 3
    if(angle > 0 && dx < 0 && dy < 0){
        // In quadrant 1 but should be in quadrant 3
        angle += 180;
    } else if(angle < 0 && dy > 0 && dx < 0){
        // In quadrant 4 but should be in quadrant 2
        angle += 180;
    }

    // alpha = angle between the x-axis and the robot destination vector - robot's heading
    return (angle - cur->heading) % DEGREES_PER_REVOLUTION;
}


// ---------- min ----------
// returns the minimum of the two inputs
// Inputs: int32_t - num1
//         int32_t - num2
// Output: int32_t - minimum of num1 and num2
int32_t min(int32_t num1, int32_t num2){
    if(num1 < num2)
        return num1;

    return num2;
}


// ---------- max ----------
// returns the maximum of the two inputs
// Inputs: int32_t - num1
//         int32_t - num2
// Output: int32_t - maximum of num1 and num2
int32_t max(int32_t num1, int32_t num2){
    if(num1 < num2)
        return num2;

    return num1;
}
