/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-01-26     armink       the first version
 */

#include <fal.h>

#include <at32f435_437.h>

#if defined (AT32F437xM) || defined (AT32F435xM) || defined (AT32F437xD) || defined (AT32F435xD) 
#define SECTOR_SIZE                      4096   /* this parameter depends on the specific model of the chip */
#else
#define SECTOR_SIZE                      2048   /* this parameter depends on the specific model of the chip */
#endif

/**
 * Get the sector of a given address
 *
 * @param address flash address
 *
 * @return The sector of a given address
 */
static rt_uint32_t stm32_get_sector(rt_uint32_t address)
{
		rt_uint32_t offset_addr = address - FLASH_BASE;
		rt_uint32_t sector = offset_addr / SECTOR_SIZE;
    rt_uint32_t sector_addr = sector * SECTOR_SIZE + FLASH_BASE;

    return sector_addr;
}

static int init(void)
{
    /* do nothing now */
}

static int read(long offset, rt_uint8_t *buf, rt_size_t size)
{
    rt_size_t i;
    rt_uint32_t addr = at32_onchip_flash.addr + offset;
    for (i = 0; i < size; i++, addr++, buf++)
    {
        *buf = *(rt_uint8_t *) addr;
    }

    return size;
}

static int write(long offset, const rt_uint8_t *buf, rt_size_t size)
{
    rt_size_t i;
    rt_uint32_t read_data;
    rt_uint32_t addr = at32_onchip_flash.addr + offset;

    flash_unlock();
    flash_flag_clear(FLASH_ODF_FLAG | FLASH_PRGMERR_FLAG | FLASH_EPPERR_FLAG);
    for (i = 0; i < size; i++, buf++, addr++)
    {
        /* write data */
        flash_byte_program(addr, *buf);
        read_data = *(rt_uint8_t *) addr;
        /* check data */
        if (read_data != *buf)
        {
            return -1;
        }
    }
    flash_lock();

    return size;
}

static int erase(long offset, rt_size_t size)
{
    flash_status_type flash_status;
    rt_size_t erased_size = 0;
    rt_uint32_t cur_erase_sector;
    rt_uint32_t addr = at32_onchip_flash.addr + offset;

    /* start erase */
    flash_unlock();
    flash_flag_clear(FLASH_ODF_FLAG | FLASH_PRGMERR_FLAG | FLASH_EPPERR_FLAG);
    /* it will stop when erased size is greater than setting size */
    while (erased_size < size)
    {
        cur_erase_sector = stm32_get_sector(addr + erased_size);
        flash_status = flash_sector_erase(cur_erase_sector);
        if (flash_status != FLASH_OPERATE_DONE)
        {
            return -1;
        }
        erased_size += SECTOR_SIZE;
    }
    flash_lock();

    return size;
}

const struct fal_flash_dev at32_onchip_flash =
{
    .name       = "at32_onchip",
    .addr       = AT32_FLASH_START_ADRESS,
    .len        = AT32_FLASH_SIZE,
    .blk_size   = 16*1024,
    .ops        = {init, read, write, erase},
    .write_gran = 8
};

