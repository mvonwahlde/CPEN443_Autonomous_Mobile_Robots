#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <math.h>
#include "msp.h"
#include "Clock.h"
#include "Motor.h"
#include "Tachometer.h"
#include "Precision_Moves.h"
#include "RobotLights.h"
#include "Distance.h"


#define TURN_TO_DESTINATION /* Comment to turn to the whatever side has the smallest distance when avoiding an obstacle */

#define LEFT_SIDE  0 /* For the type side_t */
#define RIGHT_SIDE 1 /* For the type side_t */

#define TRUE 1 /* Boolean value for true. Used for infinite loops */
#define INFINITE_LOOP_DELAY 1000 /* Delay in ms between each iteration of an infinite loop */

#define LEFT_RPM   650  /* The RPM of the left motor when driving forward */
#define RIGHT_RPM  650  /* The RPM of the right motor when driving forward */
#define SPIN_DUTY  2500 /* The duty cycle of both motors when spinning */

#define cosd(angle) (cos((float)angle * PI / 180.0)) /* Returns the cosine of the angle (degrees)*/
#define sind(angle) (sin((float)angle * PI / 180.0)) /* Returns the sine of the angle (degrees) */
#define RAD_TO_DEG(rads) (rads * 180 / PI) /* Radians to degrees conversion */

#define DRIVE_FORWARD_MM    400 /* Distance to drive forwards before correcting the spin */
#define SENSOR_THRESHOLD_MM 300 /* The threshold before a side is considered open */


// Left or right side
typedef uint32_t side_t;

// X position, y position, and heading
typedef struct Coordinates{
    int32_t xPos;
    int32_t yPos;
    int32_t heading;
} Coordinates;


// Function Prototypes
ret_t Odometry_DriveForward(Coordinates* cur, const Coordinates* dest);
void Odometry_CheckFinished(ret_t reachedDest);
side_t Odometry_PickSide(const Coordinates* cur, const Coordinates* dest);
void Odometry_Spin(Coordinates* cur, side_t direction, int32_t maxDegrees);
void Odometry_Forward(Coordinates* cur);
void Odometry_CorrectSpin(Coordinates* cur, const Coordinates* dest);
int32_t Odometry_CalculateAlpha(const Coordinates* cur, const Coordinates* dest);
int32_t min(int32_t num1, int32_t num2);
int32_t max(int32_t num1, int32_t num2);

#endif
