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

#define ZIGBEE_IMAGE_START_ADDRESS (0x20000000 + 0x10000)

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
    uint8_t *byte;

    BOOT_LOG_INF("boot_zigbee_image");

    start = (void *)(ZIGBEE_IMAGE_START_ADDRESS);
    byte = (uint8_t *)start;

    BOOT_LOG_INF("1st byte = 0x%x", *(byte + 0));
    BOOT_LOG_INF("2nd byte = 0x%x", *(byte + 1));
    BOOT_LOG_INF("3rd byte = 0x%x", *(byte + 2));
    BOOT_LOG_INF("4th byte = 0x%x", *(byte + 3));

    BOOT_LOG_INF("restore_all_irq_priorities()");
    restore_all_irq_priorities();

    BOOT_LOG_INF("booting...");
    /* Jump to entry point of the Zigbee image*/
    ((void (*)(void))start)();
}
