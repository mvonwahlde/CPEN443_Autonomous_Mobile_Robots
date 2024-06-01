#ifndef DISTANCE_H
#define DISTANCE_H

#include "ADC14.h"
#include <math.h>


// Characters to represent the distance sensors (each has a different formula)
#define RIGHT_DISTANCE_SENSOR    'r'
#define CENTER_DISTANCE_SENSOR   'c'
#define LEFT_DISTANCE_SENSOR     'l'

void Distance_GetDistances(uint32_t *leftDist, uint32_t *centerDist, uint32_t *rightDist);
void Distance_ComputeDistances(uint32_t leftADC, uint32_t *leftDist, uint32_t centerADC,
                      uint32_t *centerDist, uint32_t rightADC, uint32_t *rightDist);
uint32_t Distance_ComputeDistance(uint32_t adcReading, char side);


#endif
