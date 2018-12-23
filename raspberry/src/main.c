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

#include <stdio.h>
#include <bcm2835.h>
#include "sensors.h"
#include "mcp3201.h"

#define VREF_ADC              3.3  // V

#define TEMP_ADC_CS_PIN       RPI_V2_GPIO_P1_10
#define HUMID_ADC_CS_PIN      RPI_V2_GPIO_P1_12

int main(void) {
    struct mcp3201_spi_dev_t temp_adc_dev;
    struct mcp3201_spi_dev_t humid_adc_dev;
    float temp_degree_C = 0;
    float sensor_RH;
    unsigned int true_RH = 0;

    // If you call this, it will not actually access the GPIO
    // Use for testing
    // bcm2835_set_debug(1);

    if (!bcm2835_init())
    {
      printf("bcm2835_init failed. Are you running as root??\n");
      return 1;
    }
    if (!bcm2835_spi_begin())
    {
      printf("bcm2835_spi_begin failed. Are you running as root??\n");
      return 1;
    }

    temp_adc_dev.spi_num = 0;
    temp_adc_dev.cs_pin = TEMP_ADC_CS_PIN;
    temp_adc_dev.v_ref = VREF_ADC;

    mcp3201_init(&temp_adc_dev);

    humid_adc_dev.spi_num = 0;
    humid_adc_dev.cs_pin = HUMID_ADC_CS_PIN;
    humid_adc_dev.v_ref = VREF_ADC;

    mcp3201_init(&humid_adc_dev);

    temp_degree_C = ad592_get_temp_degree_c(
            mcp3201_get_voltage(&temp_adc_dev));
    sensor_RH = hih5030_get_sensor_rh(
            mcp3201_get_voltage(&humid_adc_dev), humid_adc_dev.v_ref);

    true_RH = hih5030_get_true_rh(sensor_RH, temp_degree_C);

    printf("%.1f %d\n",temp_degree_C, true_RH);

    bcm2835_spi_end();
    return 0;
}
