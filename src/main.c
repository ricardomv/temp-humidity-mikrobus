/*
 * File:   main.c
 * Author: ricardo
 *
 * Created on November 15, 2018, 11:05 PM
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
#pragma config FNOSC = FRC    // Oscillator Source Selection->Internal Fast RC (FRC)
#pragma config PLLMODE = DISABLED    // PLL Mode Selection->No PLL used; PLLEN bit is not available
#pragma config IESO = ON    // Two-speed Oscillator Start-up Enable bit->Start up device with FRC, then switch to user-selected oscillator source

// FOSC
#pragma config POSCMD = NONE    // Primary Oscillator Mode Select bits->Primary Oscillator disabled
#pragma config OSCIOFCN = OFF    // OSC2 Pin Function bit->OSC2 is clock output
#pragma config SOSCSEL = OFF    // SOSC Power Selection Configuration bits->Digital (SCLKI) mode
#pragma config PLLSS = PLL_PRI    // PLL Secondary Selection Configuration bit->PLL is fed by the Primary oscillator
#pragma config IOL1WAY = ON    // Peripheral pin select configuration bit->Allow only one reconfiguration
#pragma config FCKSM = CSDCMD    // Clock Switching Mode bits->Both Clock switching and Fail-safe Clock Monitor are disabled

// FWDT
#pragma config WDTPS = PS32768    // Watchdog Timer Postscaler bits->1:32768
#pragma config FWPSA = PR128    // Watchdog Timer Prescaler bit->1:128
#pragma config FWDTEN = OFF    // Watchdog Timer Enable bits->WDT and SWDTEN disabled
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

#include "xc.h"
#include <stdio.h>
#include <string.h>
#include "periph/spi.h"
#include "periph/mcu.h"
#include "periph/gpio.h"
#include "drivers/sdcard.h"
#include "drivers/sdcard_cache.h"
#include "drivers/mbr.h"
#include "drivers/lcd.h"
#include "drivers/mcp3201.h"
#include "fat16/fat16.h"
#include "periph/core_timer.h"
#include "periph/periph_conf.h"
#include "periph/timer1.h"
#include "periph/timer.h"
#include "periph/watchdog.h"
#include "periph/uart.h"
#include "utils.h"

#define SAMPLE_CNT 100000

#ifndef FIRMWARE_VERSION
#define FIRMWARE_VERSION "dev"
#endif

static const char *welcome_msg = "Proj SEP firmware " FIRMWARE_VERSION
                                 " - " __DATE__ " " __TIME__
                                 "\n";

/* Pins for SPI communication */
#define SCK_PIN               GPIO_PIN(PORT_G, 6)
#define MISO_PIN              GPIO_PIN(PORT_G, 7)
#define MOSI_PIN              GPIO_PIN(PORT_G, 8)

/* Pins for SD card */
#define SD_CS_PIN             GPIO_PIN(PORT_G, 9)

/* Pins for ADCs */
#define TEMP_ADC_CS_PIN       GPIO_PIN(PORT_D, 0)  // pin 
#define HUMID_ADC_CS_PIN      GPIO_PIN(PORT_D, 0)  // pin 72

#define LED_D3_PIN            GPIO_PIN(PORT_A, 0)
#define LED_D4_PIN            GPIO_PIN(PORT_A, 1)
#define LED_D5_PIN            GPIO_PIN(PORT_A, 2)
#define LED_D6_PIN            GPIO_PIN(PORT_A, 3)
#define LED_D7_PIN            GPIO_PIN(PORT_A, 4)
#define LED_D8_PIN            GPIO_PIN(PORT_A, 5)
#define LED_D9_PIN            GPIO_PIN(PORT_A, 6)
#define LED_D10_PIN           GPIO_PIN(PORT_A, 7)

/* Pins for UART interface */
#define UART_TX_PIN          (GPIO_PIN(PORT_F, 5))
#define UART_RX_PIN          (GPIO_PIN(PORT_F, 4))

static struct storage_dev_t dev = {
    sdcard_cache_read,
    sdcard_cache_read_byte,
    sdcard_cache_write,
    sdcard_cache_seek
};

struct sdcard_spi_dev_t sdcard_dev;
struct mcp3201_spi_dev_t temp_adc_dev;
struct mcp3201_spi_dev_t humid_adc_dev;

unsigned long int sample_counter = 0;
int sample_fd;

void timer2_callback(void)
{
    gpio_toggle(LED_D3_PIN);
    LCD_ClearScreen();
    LCD_PutLongInt(sample_counter);
}

float calc_temp_degree_c(int N)
{
    // TODO: use propper calculations
    return N*50/4095;
}

unsigned int calc_humidity_percent(int N, float temp)
{
    // TODO: use propper calculations
    return (unsigned int)N/40.95;
}

void timer3_callback(void)
{
    int ret;
    char buffer[128];
    unsigned int len = 0;
    float temp = calc_temp_degree_c(mcp3201_get_sample(&temp_adc_dev));
    unsigned int humidity = calc_humidity_percent(mcp3201_get_sample(&humid_adc_dev), temp);
    sprintf(buffer, "2018/11/20 %fºC %d%%\n", temp, humidity);
    len = strlen(buffer);
    ret = fat16_write(sample_fd, buffer, len);
    if (ret < 0 || (unsigned int)ret != len) {
        fat16_close(sample_fd);
        DBG_error("failed to write to file");
    }
    if(sample_counter++ > SAMPLE_CNT)
        timer_stop(TIMER_3);
}

void configure_periph()
{
    mcu_set_system_clock(8000000LU);
    watchdog_disable();
    
    /* Configure timer 1 to enable mcu_delay, 1 tick = 1ms */
    timer1_power_up();
    timer1_configure(TIMER1_PRESCALER_1, 4000, 1);
    timer1_start();
    
    /* UART1 */
    gpio_init_out(UART_TX_PIN, 1);
    gpio_init_in(UART_RX_PIN);
    RPOR8bits.RP17R = 3; //RP17  tx
    //RPINR18bits.U1RXR = 10;  not sure this is correct
    
    /* Configure timer 2 to blink LED */
    timer_power_up(TIMER_2);
    timer_configure(TIMER_2, TIMER2_PRESCALER_64, 15000, 1);
    
    /* Configure timer 3 to aquire sample */
    timer_power_up(TIMER_3);
    timer_configure(TIMER_3, TIMER3_PRESCALER_64, 50, 1);
    
    LCD_Initialize();
    DBG("LCD Initialized");
    
    /* SPI2 */
    gpio_init_out(MOSI_PIN, 0);
    gpio_init_in(MISO_PIN);
    gpio_init_out(SCK_PIN, 0);
    RPOR9bits.RP19R = 0x000a;       /* SPI1 - MOSI */
    RPINR22bits.SDI2R = 0x001a;     /* SPI1 - MISO */
    RPOR10bits.RP21R = 0x000b;        /* SPI1 - SCK */
    gpio_init_out(SD_CS_PIN, 1);
    gpio_init_out(TEMP_ADC_CS_PIN, 1);
    gpio_init_out(HUMID_ADC_CS_PIN, 1);
    
    gpio_init_out(LED_D3_PIN, 0);

    /* Configure spi */
    spi_power_up(SPI_2);
    spi_configure(SPI_2, 400000, SPI_MODE_0);
    spi_enable(SPI_2);
    
    /* Configure uart */
    uart_power_up(UART_1);
    uart_configure(UART_1, UART_BD_9600);
    uart_enable(UART_1);
}

int main(void) {
    uint32_t partition_offset;
    
    configure_periph();

    printf("\033[2J");
    printf(welcome_msg);
    
    print_reset_cause();

    temp_adc_dev.spi_num = SPI_2;
    temp_adc_dev.cs_pin = TEMP_ADC_CS_PIN;

    humid_adc_dev.spi_num = SPI_2;
    humid_adc_dev.cs_pin = HUMID_ADC_CS_PIN;

    sdcard_dev.spi_num = SPI_2;
    sdcard_dev.cs_pin = SD_CS_PIN;

    if (sdcard_init(&sdcard_dev))
        DBG_error("sdcard_init failed");

    sdcard_cache_init(sdcard_dev);

    partition_offset = find_fat16_partition(&sdcard_dev);
    if (partition_offset > 0) {
        /* Convert partition_offset from block to byte */
        partition_offset *= SDCARD_BLOCK_LENGTH;
        fat16_init(dev, partition_offset);
    } else {
        DBG_error( "Reading uSD card" ) ;
    }
    
    sample_fd = fat16_open("/SAMPLES.TXT", 'w');
    if (sample_fd < 0)
        DBG_error("failed to open file for write");

    /* Start statistics timer*/
    timer_start(TIMER_2);
    /* Start data acquisition*/
    timer_start(TIMER_3);
    
    /* Wait until data acquisition is done */
    while(timer_is_running(TIMER_3));
    
    fat16_close(sample_fd);
    sdcard_cache_flush();
    
    printf("DONE");
    LCD_PutString("DONE", 4);

    mcu_sleep();

    return 0;
}
