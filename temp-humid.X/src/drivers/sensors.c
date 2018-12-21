/*
 * File:   sensors.c
 * Author: ricardo
 *
 * Created on November 21, 2018, 1:24 AM
 */


#include "xc.h"
#include "sensors.h"

#define TEMP_SENSOR_GAIN 2 // with 2k ohm resistor

// Convert sensor reading into Relative Humidity (RH%) 
//  using equation from Datasheet
// VOUT = (VSUPPLY)(0.00636(SENSOR RH) + 0.1515), 
//  typical at 25 degrees Celsius

float hih5030_get_sensor_rh(float voltage, float supply)
{
    return ((voltage / (.00636 * supply)) - 23.82);
}

// Get True Relative Humidity (RH%) compensated 
//  with Static Temperature or Measured Temperature
// TRUE RH = (SENSOR RH)/(1.0546 - 0.00216T), T in degrees Celsius

float hih5030_get_true_rh(float sensor_rh, float temperature)
{
    return sensor_rh / (1.0546 - (0.00216 * temperature));
}

// Get temperature in degree Celsius
// Temperature range 0.13 V = 130ºC -> *1000
// Temperature offset 0.248 V = -25ºC -> - 273

float ad592_get_temp_degree_c(float voltage)
{
    return (voltage/TEMP_SENSOR_GAIN)*1000 - 273;
}