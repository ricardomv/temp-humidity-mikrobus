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
#include "mcp3201.h"

#define ADC_BITS        12
#define ADC_COUNTS      ((1 << ADC_BITS) - 1)

void mcp3201_init(struct mcp3201_spi_dev_t *dev)
{

    bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE);
    bcm2835_gpio_fsel(dev->cs_pin, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_write(dev->cs_pin, HIGH);
}

unsigned int mcp3201_get_sample(struct mcp3201_spi_dev_t *dev)
{
    uint8_t tx_buffer[3] = {0xFF, 0xFF, 0xFF};
    uint8_t rx_buffer[3] = {0};
    unsigned int result = 0;
/*
    spi_set_baud(dev->spi_num, 1000000);  // datasheet says it can do 800 kHz at 2.7V

    gpio_write(dev->cs_pin, 0);
    spi_transfer(dev->spi_num, NULL, rx_buffer, 2);
    gpio_write(dev->cs_pin, 1);

    spi_set_baud(dev->spi_num, 0);
*/
    bcm2835_gpio_write(dev->cs_pin, LOW);
    bcm2835_spi_transfernb(tx_buffer, rx_buffer, 2);
    bcm2835_gpio_write(dev->cs_pin, HIGH);

    result = rx_buffer[0];
    result = result << 8;
    result |= rx_buffer[1];
    result = result >> 1;
    result = result & 0x0fff;

    return result;
}

float mcp3201_get_voltage(struct mcp3201_spi_dev_t *dev) {
    return ((float)mcp3201_get_sample(dev) / ADC_COUNTS) * dev->v_ref;
}
