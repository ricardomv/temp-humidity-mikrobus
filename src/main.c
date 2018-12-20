/*
 * File:   main.c
 * Author: ricardo
 *
 * Created on November 15, 2018, 11:05 PM
 */

#include "xc.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "periph/spi.h"
#include "periph/mcu.h"
#include "periph/gpio.h"
#include "periph/core_timer.h"
#include "periph/periph_conf.h"
#include "periph/timer1.h"
#include "periph/timer.h"
#include "periph/watchdog.h"
#include "periph/uart.h"
#include "periph/rtcc.h"

#include "drivers/sdcard.h"
#include "drivers/sdcard_cache.h"
#include "drivers/mbr.h"
#include "drivers/lcd.h"
#include "drivers/mcp3201.h"
#include "drivers/sensors.h"

#include "fat16/fat16.h"

#include "utils.h"
#include "explorer1632.h"

#define VREF_ADC              3.3  // V

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
#define TEMP_ADC_CS_PIN       GPIO_PIN(PORT_C, 4)  // pin 52 pim / 95 device / RX mikrobus
#define HUMID_ADC_CS_PIN      GPIO_PIN(PORT_D, 0)  // pin 72 / PWM mikrobus

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

unsigned int sampling = 1;
int sample_fd = 255;
float temp_degree_C = 0;
unsigned int true_RH = 0;

struct sdcard_cache_stats_t stats;

struct tm current_time;

uint32_t millis;
uint32_t aquisition_secs = 86400;

void timer2_callback(void)
{
    char date_str[10];
    char write_size[7] = {0};
    gpio_toggle(LED_D3_PIN);
    LCD_ClearScreen();
    LCD_PutString("T:", 3);
    LCD_PutInt(temp_degree_C);
    LCD_PutChar('.');
    LCD_PutInt(abs((int)(temp_degree_C*10)%10));
    LCD_PutString("\xDF""C H:", 5);
    LCD_PutInt(true_RH);
    LCD_PutString("%\n\r",3);
    strftime(date_str, sizeof(date_str), "%T", &current_time);
    LCD_PutString(date_str, sizeof(date_str));
    stats = sdcard_cache_get_stats();
    LCD_PutString(" ",1);
    readable_fs(1024*stats.write_success/2, write_size);
    LCD_PutString(write_size, sizeof(write_size));
}

void data_aquisition(void)
{
    int ret;
    char buffer[30];
    char date_str[20];
    unsigned int len = 0;
    float sensor_RH;

    RTCC_TimeGet(&current_time);

    temp_degree_C = ad592_get_temp_degree_c(
            mcp3201_get_voltage(&temp_adc_dev));
    sensor_RH = hih5030_get_sensor_rh(
            mcp3201_get_voltage(&humid_adc_dev), humid_adc_dev.v_ref);

    true_RH = hih5030_get_true_rh(sensor_RH, temp_degree_C);

    strftime(date_str, sizeof(date_str), "%T", &current_time);

    sprintf(buffer, "%s.%lu %.1f %d\n", date_str,
                                        core_timer_get_ticks() - millis,
                                        (double)temp_degree_C,
                                        true_RH);

    if (sample_fd != 255) {
        len = strlen(buffer);
        ret = fat16_write(sample_fd, buffer, len);
        if (ret < 0 || (unsigned int)ret != len) {
            fat16_close(sample_fd);
            DBG_error("failed to write to file\n");
        }
    }
    else
        printf("%s", buffer);
}

void rtcc_alarm_callback(void)
{
    gpio_toggle(LED_D4_PIN);
    if(--aquisition_secs == 0) sampling = 0;
    millis = core_timer_get_ticks();
}

void configure_periph()
{
    mcu_set_system_clock(30000000LU);
    watchdog_disable();

    // DCOFSEL 30 MHz;
    DCOCONbits.DCOFSEL = 0xF;

    /* Configure timer 1 to enable mcu_delay, 1 tick = 1ms */
    timer1_power_up();
    timer1_configure(TIMER1_PRESCALER_1, 14565, 1);
    timer1_start();
    
    /* UART1 */
    gpio_init_out(UART_TX_PIN, 1);
    gpio_init_in(UART_RX_PIN);
    RPOR8bits.RP17R = 3;    // RP17 tx
    RPINR18bits.U1RXR = 10; // RP10 rx
    
    /* Configure timer 2 to blink LED */
    timer_power_up(TIMER_2);
    timer_configure(TIMER_2, TIMER2_PRESCALER_256, 15000, 1);

    /* Configure rtcc */
    RTCC_Initialize();
    
    LCD_Initialize();
    DBG("LCD Initialized\n");
    
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
    gpio_init_out(LED_D4_PIN, 0);
    gpio_init_in(BUTTON_S3_PIN);

    /* Configure spi */
    spi_power_up(SPI_2);
    spi_configure(SPI_2, 400000, SPI_MODE_0);
    spi_enable(SPI_2);
    
    /* Configure uart */
    uart_power_up(UART_1);
    if(uart_configure(UART_1, UART_BD_115200))
        DBG_error("Failed to configure UART\n");
    uart_enable(UART_1);
}

#define DATE_CMD "SET_DATE"
#define DATE_CMD_LEN 8

void sync_time(void)
{
    struct tm *time_info;
    char cmd[9] = {0};
    int count;
    time_t rawtime;

    do {
        count = uart_read_noblock(UART_1, cmd, DATE_CMD_LEN);
    }
    while(count == 0 && gpio_read(BUTTON_S3_PIN));

    if (!count || cmd[0] == '\r')
        return;

    if (DATE_CMD_LEN - count > 0)
        uart_read(UART_1, cmd+count, DATE_CMD_LEN - count);

    if (strncmp(cmd, DATE_CMD, DATE_CMD_LEN))
        DBG_error( "Setting rtcc time\n" );

    uart_read(UART_1, &rawtime, sizeof(rawtime));
    time_info = localtime(&rawtime);
    RTCC_TimeSet(time_info);

    printf ("Current local time and date: %s", asctime(time_info));
    strftime(cmd, sizeof(cmd), " %H:%M", time_info);
    LCD_PutString(cmd, 6);
}

int main(void) {
    uint32_t partition_offset;
    struct tm current_time = {0};
    char filename[14] = "/SAMPLES.TXT\0";
    
    configure_periph();

    printf("\033[2J"); // Clear console
    printf(welcome_msg);
    
    print_reset_cause();
    
    LCD_ClearScreen();
    LCD_PutString("Press S3 to \n\rstart/stop", 24);

    sync_time();

    temp_adc_dev.spi_num = SPI_2;
    temp_adc_dev.cs_pin = TEMP_ADC_CS_PIN;
    temp_adc_dev.v_ref = VREF_ADC;

    humid_adc_dev.spi_num = SPI_2;
    humid_adc_dev.cs_pin = HUMID_ADC_CS_PIN;
    humid_adc_dev.v_ref = VREF_ADC;

    sdcard_dev.spi_num = SPI_2;
    sdcard_dev.cs_pin = SD_CS_PIN;
    
    while(gpio_read(BUTTON_S3_PIN));
    while(!gpio_read(BUTTON_S3_PIN)); // wait for button release
    
    RTCC_TimeGet(&current_time);
    if (current_time.tm_year != 100)
        strftime(filename, sizeof(filename), "/%Y%m%d.TXT", &current_time);

    if (!sdcard_init(&sdcard_dev))
        DBG("Failed SD card initialization\n");
    sdcard_cache_init(sdcard_dev);

    partition_offset = find_fat16_partition(&sdcard_dev);
    if (partition_offset > 0) {
        /* Convert partition_offset from block to byte */
        partition_offset *= SDCARD_BLOCK_LENGTH;
        fat16_init(dev, partition_offset);

        sample_fd = fat16_open(filename, 'w');
        if (sample_fd < 0)
            DBG_error("Failed to open file \"%s\" for write\n", filename);
    }
    else
        DBG( "Reading SD card\n" ) ;
    
    /* Start statistics timer*/
    timer_start(TIMER_2);
    
    RTCC_AlarmSet(&current_time, EVERY_SECOND, 0, true);
    IEC3bits.RTCIE = 1;     // Enable alarm interrupt

    /* Start data acquisition*/
    while(sampling && gpio_read(BUTTON_S3_PIN)) data_aquisition();
    
    fat16_close(sample_fd);
    sdcard_cache_flush();
    
    stats = sdcard_cache_get_stats();
    printf("\nStatistics of the SD card cache:\n");
    printf("Evictions: %lu\n", stats.evictions);
    printf("Write success: %lu\n", stats.write_success);
    printf("Read success: %lu\n", stats.read_success);
    printf("Write error: %lu\n", stats.write_error);
    printf("Read error: %lu\n", stats.read_error);

    printf("Entering mcu sleep mode.\n");
    LCD_PutString("\n\rDONE            ", 18);

    IEC3bits.RTCIE = 0;     // Enable alarm interrupt

    mcu_sleep();

    return 0;
}
