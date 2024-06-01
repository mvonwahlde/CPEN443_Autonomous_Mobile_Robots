#include "msp.h"
#include "Clock.h"
#include "ADC14.h"

#define NUM_SAMPLES 30

/**
 * main.c
 */
void main(void){
    // Stop watchdog timer
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;

	Clock_Init48MHz();
	ADC0_InitSWTriggerCh17_14_16();

	uint32_t leftDist, centerDist, rightDist;
	double leftAvg, centerAvg, rightAvg;

	while(1){

	    leftAvg = 0;
	    centerAvg = 0;
	    rightAvg = 0;

	    int i;
	    for(i = 0; i < NUM_SAMPLES; i++){
	        ADC_In17_14_16(&leftDist, &centerDist, &rightDist);
	        leftAvg += leftDist;
	        centerAvg += centerDist;
	        rightAvg += rightDist;
	        Clock_Delay1ms(10);
	    }

	    leftAvg /= NUM_SAMPLES;
	    centerAvg /= NUM_SAMPLES;
	    rightAvg /= NUM_SAMPLES;


	}
}
