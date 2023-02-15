/*
 * Copyright (c) 2022 Telink Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdint.h>
#include <zephyr/sw_isr_table.h>
#include <bootutil/bootutil_log.h>
#include <flash.h>

BOOT_LOG_MODULE_DECLARE(mcuboot);

#define PLIC_PRIO (0xe4000000)
#define PLIC_IRQS (CONFIG_NUM_IRQS - CONFIG_2ND_LVL_ISR_TBL_OFFSET)

#define MATTER_IMAGE_START_ADDRESS (0x20000000 + 0xF0000) // slot1 (0xF0000~0x1D0000)
#define FLASH_ADDR_OF_SW_FLAG (0x20000000 + 0x1FD000)
#define ZIGBEE_IMAGE_START_ADDRESS (0x20000000 + 0x10000) // slot0 (0x10000~0xF0000)

static void restore_all_irq_priorities(void)
{
    volatile uint32_t *prio = (volatile uint32_t *)PLIC_PRIO;
    int i;

    /* Set priority of each interrupt line back to 1 */
    /* Interrupt source 0 does not exist*/
    for (i = 1; i < PLIC_IRQS; i++)
    {
        *prio = 1U;
        prio++;
    }
}

void boot_zigbee_image(void)
{
    void *start;
    void *flag;
    uint8_t *flag_start;
    uint8_t *byte;

    BOOT_LOG_INF("boot_zigbee_image");

    flag = (void *)(FLASH_ADDR_OF_SW_FLAG);
    flag_start = (uint8_t *)flag;

    BOOT_LOG_INF("Flag bytes:");
    for (int i = 0; i < 4; i++)
    {
        BOOT_LOG_INF("Byte %d of flag (%p) = 0x%x", i, (flag_start + i), *(flag_start + i));
    }

    /*
        flag for zigbee or matter:
        SW_FLG_ZB_JOIN_SUCCESS = 0x5A5A,    // boot Zigbee
        SW_FLG_ZB_JOIN_FAIL = 0x7A7A,       // boot Matter
        SW_FLG_ZB_LEAVE = 0x5050,           // boot Matter
        SW_FLG_IDLE = 0xFFFF,               // boot Zigbee
    */
    if (*(flag_start + 0) == 0xFF && *(flag_start + 1) == 0xFF && *(flag_start + 2) == 0xFF && *(flag_start + 3) == 0xFF)
    {
        start = (void *)(ZIGBEE_IMAGE_START_ADDRESS);
        BOOT_LOG_INF("Flag: SW_FLG_IDLE");
    }
    else if (*(flag_start + 0) == 0x05 && *(flag_start + 1) == 0x0A && *(flag_start + 2) == 0x05 && *(flag_start + 3) == 0x0A)
    {
        start = (void *)(ZIGBEE_IMAGE_START_ADDRESS);
        BOOT_LOG_INF("Flag: SW_FLG_ZB_JOIN_SUCCESS");
    }
    else if (*(flag_start + 0) == 0x05 && *(flag_start + 1) == 0x00 && *(flag_start + 2) == 0x05 && *(flag_start + 3) == 0x00)
    {
        start = (void *)(MATTER_IMAGE_START_ADDRESS);
        BOOT_LOG_INF("Flag: SW_FLG_ZB_LEAVE");
    }
    else if (*(flag_start + 0) == 0x07 && *(flag_start + 1) == 0x0A && *(flag_start + 2) == 0x07 && *(flag_start + 3) == 0x0A)
    {
        start = (void *)(MATTER_IMAGE_START_ADDRESS);
        BOOT_LOG_INF("Flag: SW_FLG_ZB_JOIN_FAIL");
    }
    else
    {
        BOOT_LOG_INF("WRONG FLAG!");
        while (1)
        {
        }
    }

    byte = (uint8_t *)start;

    BOOT_LOG_INF("Image starting bytes:");
    for (int i = 0; i < 4; i++)
    {
        BOOT_LOG_INF("Byte %d (%p) = 0x%x", i, (byte + i), *(byte + i));
    }

    BOOT_LOG_INF("restore_all_irq_priorities()");
    restore_all_irq_priorities();

    if (start == (void *)(MATTER_IMAGE_START_ADDRESS))
    {
        BOOT_LOG_INF("booting Matter App from default slot0, its starting address may be %p...", start);
    }
    else if (start == (void *)(ZIGBEE_IMAGE_START_ADDRESS))
    {
        BOOT_LOG_INF("booting Zigbee App from %p...", start);
        /* Jump to entry point of the App image*/
        ((void (*)(void))start)();
    }
    else
    {
        BOOT_LOG_INF("booting from default slot in mcuboot");
    }
}
