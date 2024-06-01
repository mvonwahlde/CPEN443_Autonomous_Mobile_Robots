#include "msp.h"
#include "Clock.h"
#include "Motor.h"
#include "PWM.h"
#include "Reflectance.h"
#include "RobotLights.h"


/**
 * main.c
 */
void main(void){
    // Stop watchdog timer and initialize the clock
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;
	Clock_Init48MHz();

	// Initialize components
	Reflectance_Init();
	Motor_Init();
	MvtLED_Init();

	// Turn on the motors
	Motor_Forward(5000, 5000);
	Front_Lights_ON();

	// Used to hold the reflectance data
	uint8_t RefData = 0x00;

	// Finds the line
	while(RefData == 0x00){
	    Clock_Delay1ms(10);
	    RefData = Reflectance_Read(1000);
	}

	// Stop the motor and turn off the front lights
	Motor_Stop();
	Front_Lights_OFF();

	// Wait 500ms to prevent jerking and then read the reflectance
	Clock_Delay1ms(500);
	RefData = Reflectance_Read(1000);

	// Back up and turn on the back lights
	Motor_Backward(1200, 1200);
	Back_Lights_ON();

	// Backs up to the line
	while(RefData == 0x00){
	    Clock_Delay1ms(10);
	    RefData = Reflectance_Read(1000);
	}

	// Stop the motors and turn on the back lights
	Motor_Stop();
	Back_Lights_OFF();

	// Wait forever
	while(1){
	    Clock_Delay1ms(100);
	}

}
