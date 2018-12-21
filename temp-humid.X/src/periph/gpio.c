/*
 * Copyright (C) 2017  Francois Berder <fberder@outlook.fr>
 *
 * This file is part of pic24-framework.
 *
 * pic24-framework is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * pic24-framework is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with pic24-framework.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <xc.h>
#include "gpio.h"
#include "periph_conf.h"

    #define GPIO_INDEX(pin)     ((pin) & 0x0F)
    #define GPIO_PORT(pin)      (((pin) >> 4) & 0x0F)
    #define GPIO_MASK(pin)      (1U << GPIO_INDEX(pin))

void gpio_init_out(uint8_t pin, uint8_t val)
{
    unsigned int port = GPIO_PORT(pin);
    unsigned int mask = GPIO_MASK(pin);

    switch (port) {
    case PORT_A:
        ANSA &= ~mask;
        ODCA &= ~mask;
        if (val)
            LATA |= mask;
        else
            LATA &= ~mask;
        TRISA &= ~mask;
        break;
    case PORT_B:
        ANSB &= ~mask;
        ODCB &= ~mask;
        if (val)
            LATB |= mask;
        else
            LATB &= ~mask;
        TRISB &= ~mask;
        break;
    case PORT_C:
        ANSC &= ~mask;
        ODCC &= ~mask;
        if (val)
            LATC |= mask;
        else
            LATC &= ~mask;
        TRISC &= ~mask;
        break;
    case PORT_D:
        ANSD &= ~mask;
        ODCD &= ~mask;
        if (val)
            LATD |= mask;
        else
            LATD &= ~mask;
        TRISD &= ~mask;
        break;
    case PORT_E:
        ANSE &= ~mask;
        ODCE &= ~mask;
        if (val)
            LATE |= mask;
        else
            LATE &= ~mask;
        TRISE &= ~mask;
        break;
    case PORT_F:
        ANSF &= ~mask;
        ODCF &= ~mask;
        if (val)
            LATF |= mask;
        else
            LATF &= ~mask;
        TRISF &= ~mask;
        break;
    case PORT_G:
        ANSG &= ~mask;
        ODCG &= ~mask;
        if (val)
            LATG |= mask;
        else
            LATG &= ~mask;
        TRISG &= ~mask;
        break;
    }
}

void gpio_init_in(uint8_t pin)
{
    unsigned int port = GPIO_PORT(pin);
    unsigned int mask = GPIO_MASK(pin);

    switch (port) {
    case PORT_A:
        ANSA &= ~mask;
        TRISA |= mask;
        break;
    case PORT_B:
        ANSB &= ~mask;
        TRISB |= mask;
        break;
    case PORT_C:
        ANSC &= ~mask;
        TRISC |= mask;
        break;
    case PORT_D:
        ANSD &= ~mask;
        TRISD |= mask;
        break;
    case PORT_E:
        ANSE &= ~mask;
        TRISE |= mask;
        break;
    case PORT_F:
        ANSF &= ~mask;
        TRISF |= mask;
        break;
    case PORT_G:
        ANSG &= ~mask;
        TRISG |= mask;
        break;
    }
}

void gpio_write(uint8_t pin, uint8_t val)
{
    unsigned int port = GPIO_PORT(pin);
    unsigned int mask = GPIO_MASK(pin);

    switch (port) {
    case PORT_A:
        if (val)
            LATA |= mask;
        else
            LATA &= ~mask;
        break;
    case PORT_B:
        if (val)
            LATB |= mask;
        else
            LATB &= ~mask;
        break;
    case PORT_C:
        if (val)
            LATC |= mask;
        else
            LATC &= ~mask;
        break;
    case PORT_D:
        if (val)
            LATD |= mask;
        else
            LATD &= ~mask;
        break;
    case PORT_E:
        if (val)
            LATE |= mask;
        else
            LATE &= ~mask;
        break;
    case PORT_F:
        if (val)
            LATF |= mask;
        else
            LATF &= ~mask;
        break;
    case PORT_G:
        if (val)
            LATG |= mask;
        else
            LATG &= ~mask;
        break;
    }
}

int gpio_read(uint8_t pin)
{
    unsigned int port = GPIO_PORT(pin);
    unsigned int mask = GPIO_MASK(pin);

    switch (port) {
    case PORT_A:
        return PORTA & mask;
    case PORT_B:
        return PORTB & mask;
    case PORT_C:
        return PORTC & mask;
    case PORT_D:
        return PORTD & mask;
    case PORT_E:
        return PORTE & mask;
    case PORT_F:
        return PORTF & mask;
    case PORT_G:
        return PORTG & mask;
    default:
        return -1;
    }
}

void gpio_toggle(uint8_t pin)
{
    unsigned int port = GPIO_PORT(pin);
    unsigned int mask = GPIO_MASK(pin);

    switch (port) {
    case PORT_A:
        LATA ^= mask;
        break;
    case PORT_B:
        LATB ^= mask;
        break;
    case PORT_C:
        LATC ^= mask;
        break;
    case PORT_D:
        LATD ^= mask;
        break;
    case PORT_E:
        LATE ^= mask;
        break;
    case PORT_F:
        LATF ^= mask;
        break;
    case PORT_G:
        LATG ^= mask;
        break;
    }
}
