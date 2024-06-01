#include "msp.h"
#include "Clock.h"
#include "Reflectance.h"
#include "RobotLights.h"
#include "Motor.h"


#define SPEED     ((uint16_t) 2000)  /* Base speed for the motors (duty cycle out of 15000) */
#define TIME      ((uint32_t) 1000)  /* Time for the reflectance sensor to wait (usec)      */

// States (dependent on the position of the robot)
#define Center    &fsm[0]  /* Aligned with center of the line */
#define Left      &fsm[1]  /* On the left side of the line    */
#define Right     &fsm[2]  /* On the right side of the line   */
#define OffLeft   &fsm[3]  /* Off the line to the left side   */
#define OffRight  &fsm[4]  /* Off the line to the right side  */
#define OffFront  &fsm[5]  /* Off the line to the front       */


// Linked data structure for each state
struct State {
    uint32_t out; // 4-bit output (left backwards, right backwards, left forwards, right forwards)
    uint32_t delay; // time to delay in 1ms
    const struct State *next[4]; // Next if 2-bit input is 0-3
};

typedef const struct State State_t;


// States are locations relative to the line
State_t fsm[6] = {{0x03 /* Forward      */, 20, { OffFront, Left, Right, Center }},  // Center
                  {0x02 /* Slight right */, 20, { OffLeft,  Left, Right, Center }},  // Left
                  {0x01 /* Slight left  */, 20, { OffRight, Left, Right, Center }},  // Right
                  {0x06 /* Spin right   */, 20, { OffLeft,  Left, Right, Center }},  // OffLeft
                  {0x09 /* Spin left    */, 20, { OffRight, Left, Right, Center }},  // OffRight
                  {0x0C /* Backwards    */, 20, { OffFront, Left, Right, Center }}}; // OffFront

State_t *Spt; // pointer to the current state


// Program entry point
void main(void) {
    // Stop watchdog timer and initialize clock
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;
	Clock_Init48MHz();

	// Containers
	uint8_t dataCenter;
	uint8_t rightData, leftData;
	uint8_t leftBack, rightBack;

	// Initializations
	Reflectance_Init();
	Motor_Init();
	MvtLED_Init();

	// Start in the center state
	Spt = Center;

	// Main loop
	while (1) {
	    // Extract state output data for the right wheel
	    rightData = (Spt->out) & 0x05;
	    rightBack = 0;

	    // Check if right wheel needs to turn backwards
	    if(rightData > 0x03){
	        rightBack = 1;
	        rightData = 0x01;
	    } else {
	        // Else, right wheel is moving forwards (or stopped)
	        rightData = rightData & 0x01;
	    }

	    // Extract state output data for the left wheel
	    leftData = (Spt->out) & 0x0A;
	    leftBack = 0;

	    // Check if left wheel needs to turn backwards
	    if(leftData > 0x03){
	        leftBack = 1;
	        leftData = 0x01;
	    } else {
	        // Else, left wheel is moving forwards (or stopped)
	        leftData = (leftData >> 1) & 0x01;
	    }

	    // Check which motor function to call
	    if(rightBack == 1 && leftBack == 1){
	        // Both wheels backwards
	        Motor_Backward(leftData*SPEED, rightData*SPEED);
	        Back_Lights_ON();
	        Front_Lights_OFF();
	    } else if(rightBack == 1 && leftBack == 0){
	        // Right wheel back, left wheel forward (spinning right)
	        Motor_Right(leftData*SPEED, rightData*SPEED);
	        Back_Lights_ON();
	        Front_Lights_ON();
	    } else if(leftBack == 1 && rightBack == 0){
	        // Left wheel back, right wheel forward (spinning left)
	        Motor_Left(leftData*SPEED, rightData*SPEED);
	        Back_Lights_ON();
	        Front_Lights_ON();
	    } else {
	        // Both wheels forward
	        Motor_Forward(leftData*SPEED, rightData*SPEED);
	        Back_Lights_OFF();
	        Front_Lights_ON();
	    }

	    // Stay in that state for specified delay
	    Clock_Delay1ms(Spt->delay);

	    // Get input from central sensors
	    dataCenter = Reflectance_Center(TIME);

	    // Set the next state
	    Spt = Spt->next[dataCenter];
	}
}
