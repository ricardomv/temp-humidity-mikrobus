/*
 * File:   main.c
 * Author: ricardo
 *
 * Created on November 15, 2018, 11:05 PM
 */

#pragma config WDTPS = PS1024 /* Set watchdog prescaler */
#pragma config FWDTEN = SWON  /* Allow watchdog to be disabled by software */

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

#define SAMPLE_CNT 204800000 /* @ Fs=2370.3 Hz = 24h */

#ifndef FIRMWARE_VERSION
#define FIRMWARE_VERSION "dev"
#endif

static const char *welcome_msg = "Proj SEP firmware " FIRMWARE_VERSION
                                 " - " __DATE__ " " __TIME__
                                 "\n";

/* Pins for SPI communication */
#define SCK_PIN         GPIO_PIN(PORT_G, 6)
#define MISO_PIN        GPIO_PIN(PORT_G, 7)
#define MOSI_PIN        GPIO_PIN(PORT_G, 8)

/* Pins for SD card */
#define SD_CS_PIN          GPIO_PIN(PORT_G, 9)

/* Pins for ADC */
#define ADC_CS_PIN          GPIO_PIN(PORT_G, 14)

#define LED_D3_PIN          GPIO_PIN(PORT_A, 0)
#define LED_D4_PIN          GPIO_PIN(PORT_A, 1)
#define LED_D5_PIN          GPIO_PIN(PORT_A, 2)
#define LED_D6_PIN          GPIO_PIN(PORT_A, 3)
#define LED_D7_PIN          GPIO_PIN(PORT_A, 4)
#define LED_D8_PIN          GPIO_PIN(PORT_A, 5)
#define LED_D9_PIN          GPIO_PIN(PORT_A, 6)
#define LED_D10_PIN          GPIO_PIN(PORT_A, 7)

/* Pins for UART interface */
#define UART_TX_PIN     (GPIO_PIN(PORT_F, 5))
#define UART_RX_PIN     (GPIO_PIN(PORT_F, 4))

static struct storage_dev_t dev = {
    sdcard_cache_read,
    sdcard_cache_read_byte,
    sdcard_cache_write,
    sdcard_cache_seek
};

static void DBG(const char *reason)
{
    //printf("%s\n", reason);
}

static void DBG_stop(const char *reason)
{
    LCD_ClearScreen();
    printf("%s\n", reason);
    while (1);
}

static void DBG_error(const char *reason)
{
    LCD_ClearScreen();
    printf("Error: %s.\n", reason);
    while (1);
}

/**
 * @brief Try to find a FAT16 partition on the SD card
 *
 * @return First sector of the FAT16 partition, 0 if no
 *         partition was found.
 */
static uint32_t find_fat16_partition(struct sdcard_spi_dev_t *dev)
{
    unsigned int i;
    uint32_t first_sector = 0;

    DBG("Reading MBR...\n");
    mbr_read_partition_table(dev);

    DBG("Looking for a FAT16 partition...\n");
    for (i = 0; i < PARTITION_ENTRY_COUNT; ++i) {
        struct partition_info_t p = mbr_get_partition_info(i);
        if ((p.status == BOOTABLE_PARTITION || p.status == INACTIVE_PARTITION)
        &&  p.type == FAT16_PARTITION_TYPE) {
            //uint32_t size_100kB = p.size / 100000; /* size in 100kB unit */
            //printf("Found FAT16 partition at entry %u\n", i);
            //printf("\tstart_sector: %lu\n", p.start_sector);
            //printf("\tsize: %lu bytes (%lu.%lu MB)\n", p.size, size_100kB / 10, size_100kB % 10);
            first_sector = p.start_sector;
            break;
        }
    }
    if (i == PARTITION_ENTRY_COUNT)
        DBG_error("Failed to found a FAT16 partition\n");

    return first_sector;
}

struct sdcard_spi_dev_t sdcard_dev;
struct mcp3201_spi_dev_t adc_dev;
unsigned long int sample_counter = 0;
int sample_fd;

void timer2_callback(void)
{
    gpio_toggle(LED_D3_PIN);
    LCD_ClearScreen();
    LCD_PutInt(sample_counter);
}

void timer3_callback(void)
{
    int ret;
    char buffer[128];
    unsigned int len = 0;
    unsigned int result = mcp3201_get_sample(&adc_dev);
    //printf("%ld\n\n", i);
    sprintf(buffer, "%d\n", result);
    len = strlen(buffer);
    ret = fat16_write(sample_fd, buffer, len);
    if (ret < 0 || (unsigned int)ret != len) {
        fat16_close(sample_fd);
        DBG_error("failed to write to file\n");
    }
    if(sample_counter++ > SAMPLE_CNT)
        timer_stop(TIMER_3);
}

void print_reset_cause()
{
    switch (mcu_get_reset_cause()) {
        case BOR_RESET:
            DBG_stop("Brownout caused reset");
            break;
        case SOFT_RESET:
            DBG_stop("SOFT_RESET");
            break;
        case WATCHDOG_RESET:
            DBG_stop("Watchdog caused reset");
            break;
        default:
            break;
    }
}

void configure_periph()
{
    mcu_set_system_clock(8000000LU);
    watchdog_disable();
    
    /* Configure timer 1 to enable mcu_delay, 1 tick = 1ms */
    timer1_power_up();
    timer1_configure(TIMER1_PRESCALER_1, 4000, 1);
    timer1_start();
    
    /* Configure timer 2 to blink LED */
    timer_power_up(TIMER_2);
    timer_configure(TIMER_2, TIMER2_PRESCALER_64, 15000, 1);
    timer_start(TIMER_2);
    
    /* Configure timer 3 to aquire sample */
    timer_power_up(TIMER_3);
    timer_configure(TIMER_3, TIMER3_PRESCALER_64, 50, 1);
    
    LCD_Initialize();
    DBG("LCD Initialized");
    
    /* SPI2 */
    gpio_init_out(MOSI_PIN, 0);
    gpio_init_in(MISO_PIN);
    gpio_init_out(SCK_PIN, 0);
    gpio_init_out(SD_CS_PIN, 1);
    gpio_init_out(ADC_CS_PIN, 1);
    RPOR9bits.RP19R = 0x000a;       /* SPI1 - MOSI */
    RPINR22bits.SDI2R = 0x001a;     /* SPI1 - MISO */
    RPOR10bits.RP21R = 0x000b;        /* SPI1 - SCK */
    
    gpio_init_out(LED_D3_PIN, 0);

    spi_power_up(SPI_2);
    spi_configure(SPI_2, 400000, SPI_MODE_0);
    spi_enable(SPI_2);
}

int main(void) {
    uint32_t partition_offset;
    
    configure_periph();

    printf(welcome_msg);
    
    print_reset_cause();

    adc_dev.spi_num = SPI_2;
    adc_dev.cs_pin = ADC_CS_PIN;

    sdcard_dev.spi_num = SPI_2;
    sdcard_dev.cs_pin = SD_CS_PIN;

    if (sdcard_init(&sdcard_dev))
        DBG_error("sdcard_init failed\n");

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
        DBG_error("failed to open file for write\n");

    /* Start data acquisition*/
    timer_start(TIMER_3);
    
    /* Wait until data acquisition is done */
    while(timer_is_running(TIMER_3));
    
    fat16_close(sample_fd);
    sdcard_cache_flush();
    
    printf("DONE");

    mcu_sleep();

    return 0;
}
