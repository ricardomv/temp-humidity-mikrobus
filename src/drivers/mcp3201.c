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

struct mcp3201_spi_dev_t {
    unsigned int spi_num;
    unsigned int cs_pin;
};

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