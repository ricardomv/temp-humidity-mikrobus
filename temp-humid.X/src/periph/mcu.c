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

// FSEC
#pragma config BWRP = OFF    // Boot Segment Write-Protect bit->Boot Segment may be written
#pragma config BSS = DISABLED    // Boot Segment Code-Protect Level bits->No Protection (other than BWRP)
#pragma config BSEN = OFF    // Boot Segment Control bit->No Boot Segment
#pragma config GWRP = OFF    // General Segment Write-Protect bit->General Segment may be written
#pragma config GSS = DISABLED    // General Segment Code-Protect Level bits->No Protection (other than GWRP)
#pragma config CWRP = OFF    // Configuration Segment Write-Protect bit->Configuration Segment may be written
#pragma config CSS = DISABLED    // Configuration Segment Code-Protect Level bits->No Protection (other than CWRP)
#pragma config AIVTDIS = OFF    // Alternate Interrupt Vector Table bit->Disabled AIVT

// FBSLIM
#pragma config BSLIM = 8191    // Boot Segment Flash Page Address Limit bits->

// FOSCSEL
#pragma config FNOSC = DCO    // Oscillator Source Selection->Internal Fast RC (FRC)
#pragma config PLLMODE = DISABLED    // PLL Mode Selection->No PLL used; PLLEN bit is not available
#pragma config IESO = ON    // Two-speed Oscillator Start-up Enable bit->Start up device with FRC, then switch to user-selected oscillator source

// FOSC
#pragma config POSCMD = NONE    // Primary Oscillator Mode Select bits->Primary Oscillator disabled
#pragma config OSCIOFCN = ON    // OSC2 Pin Function bit->OSC2 is clock output
#pragma config SOSCSEL = ON    // SOSC Power Selection Configuration bits->Digital (SCLKI) mode
#pragma config PLLSS = PLL_PRI    // PLL Secondary Selection Configuration bit->PLL is fed by the Primary oscillator
#pragma config IOL1WAY = ON    // Peripheral pin select configuration bit->Allow only one reconfiguration
#pragma config FCKSM = CSDCMD    // Clock Switching Mode bits->Both Clock switching and Fail-safe Clock Monitor are disabled

// FWDT
#pragma config WDTPS = PS32768    // Watchdog Timer Postscaler bits->1:32768
#pragma config FWPSA = PR128    // Watchdog Timer Prescaler bit->1:128
#pragma config FWDTEN = ON_SWDTEN    // Watchdog Timer Enable bits->WDT Enabled/Disabled (controlled using SWDTEN bit)
#pragma config WINDIS = OFF    // Watchdog Timer Window Enable bit->Watchdog Timer in Non-Window mode
#pragma config WDTWIN = WIN25    // Watchdog Timer Window Select bits->WDT Window is 25% of WDT period
#pragma config WDTCMX = WDTCLK    // WDT MUX Source Select bits->WDT clock source is determined by the WDTCLK Configuration bits
#pragma config WDTCLK = LPRC    // WDT Clock Source Select bits->WDT uses LPRC

// FPOR
#pragma config BOREN = ON    // Brown Out Enable bit->Brown Out Enable Bit
#pragma config LPCFG = OFF    // Low power regulator control->No Retention Sleep
#pragma config DNVPEN = ENABLE    // Downside Voltage Protection Enable bit->Downside protection enabled using ZPBOR when BOR is inactive

// FICD
#pragma config ICS = PGD1    // ICD Communication Channel Select bits->Communicate on PGEC1 and PGED1
#pragma config JTAGEN = OFF    // JTAG Enable bit->JTAG is disabled
#pragma config BTSWP = OFF    // BOOTSWP Disable->BOOTSWP instruction disabled

// FDEVOPT1
#pragma config ALTCMPI = DISABLE    // Alternate Comparator Input Enable bit->C1INC, C2INC, and C3INC are on their standard pin locations
#pragma config TMPRPIN = OFF    // Tamper Pin Enable bit->TMPRN pin function is disabled
#pragma config SOSCHP = ON    // SOSC High Power Enable bit (valid only when SOSCSEL = 1->Enable SOSC high power mode (default)
#pragma config ALTVREF = ALTREFEN    // Alternate Voltage Reference Location Enable bit->VREF+ and CVREF+ on RA10, VREF- and CVREF- on RA9

// FBOOT
#pragma config BTMODE = SINGLE    // Boot Mode Configuration bits->Device is in Single Boot (legacy) mode

#include <xc.h>
#include "core_timer.h"
#include "mcu.h"

static uint32_t system_clock;

void __attribute__((interrupt, no_auto_psv, noreturn)) _OscillatorFail(void)
{
    INTCON1 &= ~_INTCON1_OSCFAIL_MASK;
    while (1)
        ;
}

void __attribute__((interrupt, no_auto_psv, noreturn)) _AddressError(void)
{
    INTCON1 &= ~_INTCON1_ADDRERR_MASK;
    while (1)
        ;
}

void __attribute__((interrupt, no_auto_psv, noreturn)) _StackError(void)
{
    INTCON1 &= ~_INTCON1_STKERR_MASK;
    while (1)
        ;
}

void __attribute__((interrupt, no_auto_psv, noreturn)) _MathError(void)
{
    INTCON1 &= ~_INTCON1_MATHERR_MASK;
    while (1)
        ;
}

void mcu_set_system_clock(uint32_t _system_clock)
{
    system_clock = _system_clock;
}

uint32_t mcu_get_system_clock(void)
{
    return system_clock;
}

void mcu_enable_interrupts(void)
{
    SET_CPU_IPL(0);
}

void mcu_disable_interrupts(void)
{
    SET_CPU_IPL(7);
}

unsigned int mcu_save_context(void)
{
    unsigned int ctx;
    SET_AND_SAVE_CPU_IPL(ctx, 7);
    return ctx;
}

void mcu_restore_context(unsigned int ctx)
{
    RESTORE_CPU_IPL(ctx);
}

uint16_t mcu_get_id(void)
{
    volatile uint16_t *devid = (volatile uint16_t *)0xFF0000;

    return *devid;
}

void mcu_delay(uint32_t ticks)
{
    uint32_t now = core_timer_get_ticks();
    while (core_timer_get_ticks() - now < ticks)
        ;
}

void mcu_delay_sec(uint32_t secs)
{
    uint32_t i;

    for (i = 0; i < secs; ++i)
        mcu_delay(TICKS_PER_SEC);
}

void mcu_reset(void)
{
    __asm__ __volatile__ ("reset");
}

enum reset_cause mcu_get_reset_cause(void)
{
    static enum reset_cause cause;

    if (cause)
        return cause;

    if (RCON & _RCON_WDTO_MASK)
        cause = WATCHDOG_RESET;
    else if (RCON & _RCON_SWR_MASK)
        cause = SOFT_RESET;
    else if (RCON & _RCON_EXTR_MASK)
        cause = EXT_RESET;
    else if (RCON & _RCON_POR_MASK)
        cause = POR_RESET;
    else if (RCON & _RCON_BOR_MASK)
        cause = BOR_RESET;

    RCON &= ~(_RCON_EXTR_MASK | _RCON_SWR_MASK | _RCON_WDTO_MASK |
              _RCON_BOR_MASK | _RCON_POR_MASK);

    return cause;
}

void mcu_idle(void)
{
    __asm__ __volatile__ ("pwrsav #1");
}

void mcu_sleep(void)
{
    __asm__ __volatile__ ("pwrsav #0");
}
