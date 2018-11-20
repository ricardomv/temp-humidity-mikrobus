/*
 * File:   utils.c
 * Author: ricardo
 *
 * Created on November 20, 2018, 12:10 AM
 */


#include "xc.h"
#include "utils.h"
#include "drivers/mbr.h"

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

    DBG("Reading MBR...");
    mbr_read_partition_table(dev);

    DBG("Looking for a FAT16 partition...");
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
        DBG("Failed to found a FAT16 partition");

    return first_sector;
}

