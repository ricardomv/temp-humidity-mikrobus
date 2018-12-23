// Copyright (c) 2018 Ricardo Marcelino Vieira ricardo.vieira@tecnico.ulisboa.pt
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to
// the following conditions:
//
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
// LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
// OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

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
// Temperature range 0.13 V = 130�C -> *1000
// Temperature offset 0.248 V = -25�C -> - 273

float ad592_get_temp_degree_c(float voltage)
{
    return (voltage/TEMP_SENSOR_GAIN)*1000 - 273;
}
