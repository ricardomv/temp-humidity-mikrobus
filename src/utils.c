/*
 * File:   utils.c
 * Author: ricardo
 *
 * Created on November 20, 2018, 12:10 AM
 */


#include "xc.h"
#include "utils.h"
#include "drivers/mbr.h"
#include "periph/mcu.h"

/**
 * @brief Try to find a FAT16 partition on the SD card
 *
 * @return First sector of the FAT16 partition, 0 if no
 *         partition was found.
 */

uint32_t find_fat16_partition(struct sdcard_spi_dev_t *dev)
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
#ifndef NDEBUG
            uint32_t size_100kB = p.size / 100000; /* size in 100kB unit */
            DBG("Found FAT16 partition at entry %u\n", i);
            DBG("\tstart_sector: %lu\n", p.start_sector);
            DBG("\tsize: %lu bytes (%lu.%lu MB)\n", p.size, size_100kB / 10, size_100kB % 10);
#endif
            first_sector = p.start_sector;
            break;
        }
    }
    if (i == PARTITION_ENTRY_COUNT)
        DBG("Failed to found a FAT16 partition\n");

    return first_sector;
}

void print_reset_cause(void)
{
    switch (mcu_get_reset_cause()) {
        case BOR_RESET:
            DBG_stop("Brownout caused reset\n");
            break;
        case SOFT_RESET:
            DBG_stop("SOFT_RESET\n");
            break;
        case WATCHDOG_RESET:
            DBG_stop("Watchdog caused reset\n");
            break;
        default:
            break;
    }
}

char* readable_fs(double size/*in bytes*/, char *buf) {
    int i = 0;
    const char* units[] = {"B", "kB", "MB", "GB", "TB", "PB", "EB", "ZB", "YB"};
    while (size > 1024) {
        size /= 1024;
        i++;
    }
    sprintf(buf, "%.*f%s", i, size, units[i]);
    return buf;
}
