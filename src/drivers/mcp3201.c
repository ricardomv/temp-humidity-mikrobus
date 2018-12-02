/*
 * File:   mcp3201.c
 * Author: ricardo
 *
 * Created on November 17, 2018, 1:30 PM
 */


#include "xc.h"
#include <stdio.h>
#include "periph/spi.h"
#include "periph/gpio.h"
#include "mcp3201.h"

#define ADC_BITS        12
#define ADC_COUNTS      (1 << ADC_BITS)

unsigned int mcp3201_get_sample(struct mcp3201_spi_dev_t *dev)
{
    uint8_t rx_buffer[3] = {0};
    unsigned int result = 0;

    gpio_write(dev->cs_pin, 0);
    spi_transfer(dev->spi_num, NULL, rx_buffer, 2);
    gpio_write(dev->cs_pin, 1);

    result = rx_buffer[0];
    result = result << 8;
    result |= rx_buffer[1];
    result = result >> 1;
    result = result & 0x0fff;
    
    return result;
}

float mcp3201_get_voltage(struct mcp3201_spi_dev_t *dev) {
    return (mcp3201_get_sample(dev) / ADC_COUNTS) * dev->v_ref;
}
