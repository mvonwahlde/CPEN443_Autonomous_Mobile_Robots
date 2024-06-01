/*
 * Precision_Moves.h
 *
 *  Created on: Feb 14, 2023
 *      Author: John Tadrous
 */

#ifndef PRECISION_MOVES_H_
#define PRECISION_MOVES_H_


#include <stdio.h>
#include "msp.h"
#include "Clock.h"
#include "Motor.h"
#include "Tachometer.h"
#include "RobotLights.h"
#include "Distance.h"


// New type for returns
typedef int32_t ret_t;
#define REACHED_DESTINATION 1 /* Robot reached destination    */
#define DRIVING_INTERRUPTED 0 /* Detected obstacle in the way */


// --------------------- Definitions ---------------------

#define PI                     3.14159265f  /* Approximate for pi to 8 decimal places */
#define CM_TO_MM(cm)           cm*10        /* Converts centimeters to millimeters    */
#define CIRCUMFERENCE(radius)  2*radius*PI  /* Calculates diameter from a radius      */

#define WIDTH_MM               140          /* Width of the robot (mm)                */
#define DIAMETER_MM            70           /* Diameter of the robot's wheels (mm)    */
#define PULSES_PER_REV         360          /* Number of spokes on tachometer wheel   */
#define DEGREES_PER_REVOLUTION 360          /* Number of degrees per revolution       */

#define ANGLE_TO_STEPS(angle)       (int32_t)((angle)*WIDTH_MM/DIAMETER_MM)                            /* Converts angle (degrees) to steps */
#define DISTANCE_TO_STEPS_FL(dist)  ((float)dist * (float)PULSES_PER_REV / (PI * (float)DIAMETER_MM))  /* Distance (mm) to steps (float)    */
#define DISTANCE_TO_STEPS(dist)     (int32_t)(DISTANCE_TO_STEPS_FL(dist))                              /* Distance (mm) to steps (int)      */
#define STEPS_TO_ANGLE(steps)       (int32_t)((steps)*DIAMETER_MM/WIDTH_MM)                            /* Steps(int) to distance (mm)       */
#define STEPS_TO_DISTANCE_FL(steps) ((float)steps * PI * (float)DIAMETER_MM / ((float)PULSES_PER_REV)) /* Steps (float) to distance (mm)    */
#define STEPS_TO_DISTANCE(steps)    (int32_t)(STEPS_TO_DISTANCE_FL(steps))                             /* Steps (int) to distance (mm)      */

#define KP_SPIN(error)    (int32_t)(error*20)     /* Proportional term for spinning controller    */
#define KP_FORWARD(error) (int32_t)(error)       /* Proportional term for forward controller     */
#define KI_FORWARD(error) (int32_t)(error/5)     /* Integral term for the forward controller     */

#define INTEGRAL_MIN           -200              /* Minimum integral term (to prevent windup)    */
#define INTEGRAL_MAX            200              /* Maxumum integral term (to prevent windup)    */

#define MIN_DUTY_CYCLE          0                /* Minimum duty cycle for the wheel motors      */
#define MAX_DUTY_CYCLE          14998            /* Maximum duty cycle for the wheel motors      */

#define SPIN_DELAY_MS           20               /* Delay between spinning controller iterations */
#define FORWARD_DELAY_MS        60               /* Delay between forward controller iterations  */
#define ACTIVE_BRAKING_DELAY_MS 60               /* Delay for the length of active braking       */

#define RPM_TO_DUTY_EST(rpm)    (int32_t)(6*rpm) /* Estimate for the starting duty cycle of the forward controller */

#define COMPUTE_RPM(steps, prevSteps, time, prevTime) (int32_t)(20000000 * (steps - prevSteps) / (time - prevTime + 1)) /* Compute RPM from steps and times */

#define NO_INTERRUPT 0 /* Spin function should not interrupt when the opposite side is open */
#define DO_INTERRUPT    1 /* Spin function should interrupt when the opposite side is open     */

#define MIN_DISTANCE_MM     150 /* Minimum distance from a sensor to an object         */
#define MAX_DISTANCE_MM     200 /* Maximum distance before the sensor is declared open */


// --------------------- Function Prototypes ---------------------
ret_t Motor_Precision_Right(int16_t speed, int32_t desiredSteps, uint8_t distanceInterrupt);
ret_t Motor_Precision_Left(int16_t speed, int32_t desiredSteps, uint8_t distanceInterrupt);
ret_t Motor_Forward_RPM(uint16_t leftRPM, uint16_t rightRPM, int32_t desiredLSteps, int32_t desiredRSteps);


#endif /* PRECISION_MOVES_H_ */
