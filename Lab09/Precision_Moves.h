/*
 * Precision_Moves.h
 *
 *  Created on: Feb 14, 2023
 *      Author: John Tadrous
 */

#ifndef PRECISION_MOVES_H_
#define PRECISION_MOVES_H_


// --------------------- Definitions ---------------------

#define PI                    3.14159265f  /* Approximate for pi to 8 decimal places */
#define CM_TO_MM(cm)          cm*10        /* Converts centimeters to millimeters    */
#define CIRCUMFERENCE(radius) 2*radius*PI  /* Calculates diameter from a radius      */

#define WIDTH_MM              140          /* Width of the robot (mm)                */
#define DIAMETER_MM           70           /* Diameter of the robot's wheels (mm)    */
#define PULSES_PER_REV        360          /* Number of spokes on tachometer wheel   */

#define ANGLE_TO_STEPS(angle)      (int32_t)((angle)*WIDTH_MM/DIAMETER_MM)                                    /* Converts angle (degrees) to steps */
#define DISTANCE_TO_STEPS_FL(dist) (float)((float)dist * (float)PULSES_PER_REV / (PI * (float)DIAMETER_MM)) /* Distance (mm) to steps (float)    */
#define DISTANCE_TO_STEPS(dist)    (int32_t)(DISTANCE_TO_STEPS_FL(dist))                                    /* Distance (mm) to steps (int)      */

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

#define MIN_SPEED               350                 /* Minimum speed for the circle (0.1 RPM) */
#define MIN_RADIUS              WIDTH_MM/20         /* Minimum radius for the circle (cm)     */

#define INNER_RADIUS(radius) (radius - WIDTH_MM/2)  /* Calculates the inner wheel's radius (mm) */
#define OUTER_RADIUS(radius) (radius + WIDTH_MM/2)  /* Calculates the outer wheel's radius (mm) */

#define MIN_SIDE_LENGTH         20                  /* Min side length of the star (cm)                */
#define MIN_STAR_SPEED          400                 /* Min speed for the straight sections of the star */
#define NUM_STAR_POINTS         5                   /* Total number of points on the star shape        */
#define STAR_SPINNING_SPEED     3000                /* Spinning speed for turns on the star shape      */
#define STAR_DELAY_MS           250                 /* Delay between each action of the star shape     */
#define STAR_INTERIOR_ANGLE     36                  /* The interior angle of the star (degrees)        */
#define STAR_EXTERIOR_ANGLE     108                 /* The exterior angle of the star (degrees)        */


// --------------------- Function Prototypes ---------------------
void Motor_Precision_CircleCCW(uint16_t speed, uint16_t radius);
void Motor_Precision_CircleCW(uint16_t speed, uint16_t radius);
void Motor_Precision_StarCCW(uint16_t speed, uint16_t sideLength);
void Motor_Precision_Right(int16_t speed, int32_t desiredSteps);
void Motor_Precision_Left(int16_t speed, int32_t desiredSteps);
void Motor_Forward_RPM(uint16_t leftRPM, uint16_t rightRPM, int32_t desiredLSteps, int32_t desiredRSteps);


#endif /* PRECISION_MOVES_H_ */
