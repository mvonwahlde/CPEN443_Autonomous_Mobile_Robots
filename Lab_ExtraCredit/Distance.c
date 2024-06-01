#include "Distance.h"


void Distance_GetDistances(uint32_t *leftDist, uint32_t *centerDist, uint32_t *rightDist){
    uint32_t leftADC, centerADC, rightADC;

    // Get distances from each sensor
    ADC_In17_14_16(&leftADC, &centerADC, &rightADC);
    Distance_ComputeDistances(leftADC, leftDist, centerADC, centerDist, rightADC, rightDist);
}

// Compute all distances from the center
// Inputs: ADC values and pointers to store distances
void Distance_ComputeDistances(uint32_t leftADC, uint32_t *leftDist, uint32_t centerADC,
                      uint32_t *centerDist, uint32_t rightADC, uint32_t *rightDist){
    *rightDist = Distance_ComputeDistance(leftADC, LEFT_DISTANCE_SENSOR); // Supposed to be flipped
    *centerDist = Distance_ComputeDistance(centerADC, CENTER_DISTANCE_SENSOR);
    *leftDist = Distance_ComputeDistance(rightADC, RIGHT_DISTANCE_SENSOR); // Supposed to be flipped
}


// Formulas to compute each distance
// Inputs: ADC values for the distance and the corresponding side
uint32_t Distance_ComputeDistance(uint32_t adcReading, char side){
    uint32_t distanceMM;

    if(side == RIGHT_DISTANCE_SENSOR){
        distanceMM = (uint32_t)(3.0*pow(10.0,6.0)*pow((double)adcReading,-1.116));
    } else if(side == CENTER_DISTANCE_SENSOR){
        distanceMM = (uint32_t)(6.0*pow(10.0,6.0)*pow((double)adcReading,-1.182));
    } else { // side == LEFT_DISTANCE_SENSOR
        distanceMM = (uint32_t)(3.0*pow(10.0,6.0)*pow((double)adcReading,-1.110));
    }

    return distanceMM;
}
