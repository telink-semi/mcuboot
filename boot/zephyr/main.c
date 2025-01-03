/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 * Copyright (c) 2020 Arm Limited
 * Copyright (c) 2021-2023 Nordic Semiconductor ASA
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <assert.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/usb/usb_device.h>
#include <soc.h>
#include <zephyr/linker/linker-defs.h>

#if defined(CONFIG_CPU_AARCH32_CORTEX_A) || defined(CONFIG_CPU_AARCH32_CORTEX_R)
#include <zephyr/arch/arm/aarch32/cortex_a_r/cmsis.h>
#elif defined(CONFIG_CPU_CORTEX_M)
#include <zephyr/arch/arm/aarch32/cortex_m/cmsis.h>
#endif

#include "target.h"

#include "bootutil/bootutil_log.h"
#include "bootutil/image.h"
#include "bootutil/bootutil.h"
#include "bootutil/fault_injection_hardening.h"
#include "bootutil/mcuboot_status.h"
#include "flash_map_backend/flash_map_backend.h"

#ifdef CONFIG_MCUBOOT_SERIAL
#include "boot_serial/boot_serial.h"
#include "serial_adapter/serial_adapter.h"

const struct boot_uart_funcs boot_funcs = {
    .read = console_read,
    .write = console_write
};
#endif

#ifdef CONFIG_BOOT_SERIAL_BOOT_MODE
#include <zephyr/retention/bootmode.h>
#endif

#if defined(CONFIG_BOOT_USB_DFU_WAIT) || defined(CONFIG_BOOT_USB_DFU_GPIO)
#include <zephyr/usb/class/usb_dfu.h>
#endif

#if CONFIG_MCUBOOT_CLEANUP_ARM_CORE
#include <arm_cleanup.h>
#endif

#ifdef CONFIG_BOOT_SERIAL_PIN_RESET
#include <zephyr/drivers/hwinfo.h>
#endif

/* CONFIG_LOG_MINIMAL is the legacy Kconfig property,
 * replaced by CONFIG_LOG_MODE_MINIMAL.
 */
#if (defined(CONFIG_LOG_MODE_MINIMAL) || defined(CONFIG_LOG_MINIMAL))
#define ZEPHYR_LOG_MODE_MINIMAL 1
#endif

/* CONFIG_LOG_IMMEDIATE is the legacy Kconfig property,
 * replaced by CONFIG_LOG_MODE_IMMEDIATE.
 */
#if (defined(CONFIG_LOG_MODE_IMMEDIATE) || defined(CONFIG_LOG_IMMEDIATE))
#define ZEPHYR_LOG_MODE_IMMEDIATE 1
#endif

#if defined(CONFIG_LOG) && !defined(ZEPHYR_LOG_MODE_IMMEDIATE) && \
    !defined(ZEPHYR_LOG_MODE_MINIMAL)
#ifdef CONFIG_LOG_PROCESS_THREAD
#warning "The log internal thread for log processing can't transfer the log"\
         "well for MCUBoot."
#else
#include <zephyr/logging/log_ctrl.h>

#define BOOT_LOG_PROCESSING_INTERVAL K_MSEC(30) /* [ms] */

/* log are processing in custom routine */
K_THREAD_STACK_DEFINE(boot_log_stack, CONFIG_MCUBOOT_LOG_THREAD_STACK_SIZE);
struct k_thread boot_log_thread;
volatile bool boot_log_stop = false;
K_SEM_DEFINE(boot_log_sem, 1, 1);

/* log processing need to be initalized by the application */
#define ZEPHYR_BOOT_LOG_START() zephyr_boot_log_start()
#define ZEPHYR_BOOT_LOG_STOP() zephyr_boot_log_stop()
#endif /* CONFIG_LOG_PROCESS_THREAD */
#else
/* synchronous log mode doesn't need to be initalized by the application */
#define ZEPHYR_BOOT_LOG_START() do { } while (false)
#define ZEPHYR_BOOT_LOG_STOP() do { } while (false)
#endif /* defined(CONFIG_LOG) && !defined(ZEPHYR_LOG_MODE_IMMEDIATE) && \
        * !defined(ZEPHYR_LOG_MODE_MINIMAL)
	*/

#ifdef CONFIG_SOC_FAMILY_NRF
#include <helpers/nrfx_reset_reason.h>

static inline bool boot_skip_serial_recovery()
{
    uint32_t rr = nrfx_reset_reason_get();

    return !(rr == 0 || (rr & NRFX_RESET_REASON_RESETPIN_MASK));
}
#else
static inline bool boot_skip_serial_recovery()
{
    return false;
}
#endif

BOOT_LOG_MODULE_REGISTER(mcuboot);

/* Validate serial recovery configuration */
#ifdef CONFIG_MCUBOOT_SERIAL
#if !defined(CONFIG_BOOT_SERIAL_ENTRANCE_GPIO) && \
    !defined(CONFIG_BOOT_SERIAL_WAIT_FOR_DFU) && \
    !defined(CONFIG_BOOT_SERIAL_BOOT_MODE) && \
    !defined(CONFIG_BOOT_SERIAL_NO_APPLICATION) && \
    !defined(CONFIG_BOOT_SERIAL_PIN_RESET)
#error "Serial recovery selected without an entrance mode set"
#endif
#endif

#ifdef CONFIG_MCUBOOT_INDICATION_LED

/*
 * The led0 devicetree alias is optional. If present, we'll use it
 * to turn on the LED whenever the button is pressed.
 */
#if DT_NODE_EXISTS(DT_ALIAS(mcuboot_led0))
#define LED0_NODE DT_ALIAS(mcuboot_led0)
#elif DT_NODE_EXISTS(DT_ALIAS(bootloader_led0))
#warning "bootloader-led0 alias is deprecated; use mcuboot-led0 instead"
#define LED0_NODE DT_ALIAS(bootloader_led0)
#endif

#if DT_NODE_HAS_STATUS(LED0_NODE, okay) && DT_NODE_HAS_PROP(LED0_NODE, gpios)
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
#else
/* A build error here means your board isn't set up to drive an LED. */
#error "Unsupported board: led0 devicetree alias is not defined"
#endif

void led_init(void)
{
    if (!device_is_ready(led0.port)) {
        BOOT_LOG_ERR("Didn't find LED device referred by the LED0_NODE\n");
        return;
    }

    gpio_pin_configure_dt(&led0, GPIO_OUTPUT);
    gpio_pin_set_dt(&led0, 0);
}
#endif /* CONFIG_MCUBOOT_INDICATION_LED */

void os_heap_init(void);

#if defined(CONFIG_ARM)

#ifdef CONFIG_SW_VECTOR_RELAY
extern void *_vector_table_pointer;
#endif

struct arm_vector_table {
    uint32_t msp;
    uint32_t reset;
};

static void do_boot(struct boot_rsp *rsp)
{
    struct arm_vector_table *vt;

    /* The beginning of the image is the ARM vector table, containing
     * the initial stack pointer address and the reset vector
     * consecutively. Manually set the stack pointer and jump into the
     * reset vector
     */
#ifdef CONFIG_BOOT_RAM_LOAD
    /* Get ram address for image */
    vt = (struct arm_vector_table *)(rsp->br_hdr->ih_load_addr + rsp->br_hdr->ih_hdr_size);
#else
    uintptr_t flash_base;
    int rc;

    /* Jump to flash image */
    rc = flash_device_base(rsp->br_flash_dev_id, &flash_base);
    assert(rc == 0);

    vt = (struct arm_vector_table *)(flash_base +
                                     rsp->br_image_off +
                                     rsp->br_hdr->ih_hdr_size);
#endif

    if (IS_ENABLED(CONFIG_SYSTEM_TIMER_HAS_DISABLE_SUPPORT)) {
        sys_clock_disable();
    }

#ifdef CONFIG_USB_DEVICE_STACK
    /* Disable the USB to prevent it from firing interrupts */
    usb_disable();
#endif
#if CONFIG_MCUBOOT_CLEANUP_ARM_CORE
    cleanup_arm_nvic(); /* cleanup NVIC registers */

#ifdef CONFIG_CPU_CORTEX_M_HAS_CACHE
    /* Disable instruction cache and data cache before chain-load the application */
    SCB_DisableDCache();
    SCB_DisableICache();
#endif

#if CONFIG_CPU_HAS_ARM_MPU || CONFIG_CPU_HAS_NXP_MPU
    z_arm_clear_arm_mpu_config();
#endif

#if defined(CONFIG_BUILTIN_STACK_GUARD) && \
    defined(CONFIG_CPU_CORTEX_M_HAS_SPLIM)
    /* Reset limit registers to avoid inflicting stack overflow on image
     * being booted.
     */
    __set_PSPLIM(0);
    __set_MSPLIM(0);
#endif

#else
    irq_lock();
#endif /* CONFIG_MCUBOOT_CLEANUP_ARM_CORE */

#ifdef CONFIG_BOOT_INTR_VEC_RELOC
#if defined(CONFIG_SW_VECTOR_RELAY)
    _vector_table_pointer = vt;
#ifdef CONFIG_CPU_CORTEX_M_HAS_VTOR
    SCB->VTOR = (uint32_t)__vector_relay_table;
#endif
#elif defined(CONFIG_CPU_CORTEX_M_HAS_VTOR)
    SCB->VTOR = (uint32_t)vt;
#endif /* CONFIG_SW_VECTOR_RELAY */
#else /* CONFIG_BOOT_INTR_VEC_RELOC */
#if defined(CONFIG_CPU_CORTEX_M_HAS_VTOR) && defined(CONFIG_SW_VECTOR_RELAY)
    _vector_table_pointer = _vector_start;
    SCB->VTOR = (uint32_t)__vector_relay_table;
#endif
#endif /* CONFIG_BOOT_INTR_VEC_RELOC */

    __set_MSP(vt->msp);
#if CONFIG_MCUBOOT_CLEANUP_ARM_CORE
    __set_CONTROL(0x00); /* application will configures core on its own */
    __ISB();
#endif
    ((void (*)(void))vt->reset)();
}

#elif defined(CONFIG_XTENSA)
#define SRAM_BASE_ADDRESS	0xBE030000

static void copy_img_to_SRAM(int slot, unsigned int hdr_offset)
{
    const struct flash_area *fap;
    int area_id;
    int rc;
    unsigned char *dst = (unsigned char *)(SRAM_BASE_ADDRESS + hdr_offset);

    BOOT_LOG_INF("Copying image to SRAM");

    area_id = flash_area_id_from_image_slot(slot);
    rc = flash_area_open(area_id, &fap);
    if (rc != 0) {
        BOOT_LOG_ERR("flash_area_open failed with %d\n", rc);
        goto done;
    }

    rc = flash_area_read(fap, hdr_offset, dst, fap->fa_size - hdr_offset);
    if (rc != 0) {
        BOOT_LOG_ERR("flash_area_read failed with %d\n", rc);
        goto done;
    }

done:
    flash_area_close(fap);
}

/* Entry point (.ResetVector) is at the very beginning of the image.
 * Simply copy the image to a suitable location and jump there.
 */
static void do_boot(struct boot_rsp *rsp)
{
    void *start;

    BOOT_LOG_INF("br_image_off = 0x%x\n", rsp->br_image_off);
    BOOT_LOG_INF("ih_hdr_size = 0x%x\n", rsp->br_hdr->ih_hdr_size);

    /* Copy from the flash to HP SRAM */
    copy_img_to_SRAM(0, rsp->br_hdr->ih_hdr_size);

    /* Jump to entry point */
    start = (void *)(SRAM_BASE_ADDRESS + rsp->br_hdr->ih_hdr_size);
    ((void (*)(void))start)();
}

#else
/* Default: Assume entry point is at the very beginning of the image. Simply
 * lock interrupts and jump there. This is the right thing to do for X86 and
 * possibly other platforms.
 */
#if (CONFIG_SOC_RISCV_TELINK_TL321X || CONFIG_SOC_RISCV_TELINK_B92)
#include <ext_driver/ext_pm.h>
#include <zephyr/device.h>
#include <zephyr/storage/flash_map.h>

#define USER_PARTITION		user_para_partition
#define USER_PARTITION_DEVICE	FIXED_PARTITION_DEVICE(USER_PARTITION)
#define USER_PARTITION_OFFSET	FIXED_PARTITION_OFFSET(USER_PARTITION)
#define USER_PARTITION_SIZE 	FIXED_PARTITION_SIZE(USER_PARTITION) 

#define SLOT0_PARTITION		slot0_partition
#define SLOT0_PARTITION_DEVICE	FIXED_PARTITION_DEVICE(SLOT0_PARTITION)
#define SLOT0_PARTITION_OFFSET	FIXED_PARTITION_OFFSET(SLOT0_PARTITION)

#if CONFIG_SOC_SERIES_RISCV_TELINK_TLX
#define SLOT0_ZB_OFFSET     (0xF6000)
#else
#define SLOT0_ZB_OFFSET     (0x154000)
#endif

#define USER_MATTER_PAIR_VAL    0x55  // jump to matter

#define USER_INIT_VAL           0xff  // init state or others will go into zb 
#define USER_ZB_SW_VAL          0xaa  // jump to matter,use XIP
#define USER_MATTER_BACK_ZB     0xa0  // only commisiion fail will back to zb 

#define ZB_FW_FLAG_OFFSET       0x20 //telink fw valid flag offset .

const struct device * flash_para_dev = USER_PARTITION_DEVICE;
const struct device * flash_slot0_dev = SLOT0_PARTITION_DEVICE;
const uint8_t zb_fw_flag[4]={ 0x4b, 0x4e, 0x4c, 0x54 };
uint8_t zb_slot0_flag[4];

static void restore_all_irq_priorities(void)
{
    #if CONFIG_SOC_SERIES_RISCV_TELINK_TLX 
    #define PLIC_PRIO (0xc4000000)
    #elif CONFIG_SOC_SERIES_RISCV_TELINK_B9X
    #define PLIC_PRIO (0xe4000000)
    #endif  
    #define PLIC_IRQS (CONFIG_NUM_IRQS - CONFIG_2ND_LVL_ISR_TBL_OFFSET)
    volatile uint32_t *prio = (volatile uint32_t *)PLIC_PRIO;
    int i;
    for (i = 1; i < PLIC_IRQS; i++)
    {
        *prio = 1U;
        prio++;
    }
}
#endif

static void do_boot(struct boot_rsp *rsp)
{
    void *start;

#if defined(MCUBOOT_RAM_LOAD)
    start = (void *)(rsp->br_hdr->ih_load_addr + rsp->br_hdr->ih_hdr_size);
#else
    uintptr_t flash_base;
    int rc;

    rc = flash_device_base(rsp->br_flash_dev_id, &flash_base);
    assert(rc == 0);

    start = (void *)(flash_base + rsp->br_image_off +
                     rsp->br_hdr->ih_hdr_size);
#endif

#if (CONFIG_SOC_RISCV_TELINK_TL321X || CONFIG_SOC_RISCV_TELINK_B92)
    /* read the boot flag from the user partition to determine the boot behavior */
    uint8_t boot_flag = 0;
    flash_read(flash_para_dev, USER_PARTITION_OFFSET, &boot_flag, 1);
    printk("boot flag: 0x%x\n", boot_flag);

    /* read the Zigbee firmware flag from the slot1 partition */
    flash_read(flash_slot0_dev, SLOT0_ZB_OFFSET + ZB_FW_FLAG_OFFSET, zb_slot0_flag, sizeof(zb_slot0_flag));
    if (memcmp(zb_slot0_flag, zb_fw_flag, sizeof(zb_fw_flag))){
        /* default to Matter if Zigbee firmware flag not found */
        printk("Zigbee firmware flag not found.\n");
        start = (void *)(flash_base + rsp->br_image_off +
                        rsp->br_hdr->ih_hdr_size);
    } else {
        if ( boot_flag == USER_MATTER_PAIR_VAL ) {
            /* switch to Matter only if commissioned (paired) */
            start = (void *)(flash_base + rsp->br_image_off +
                        rsp->br_hdr->ih_hdr_size);
        } else {
            /* Otherwise, switch to Zigbee */
            restore_all_irq_priorities();
            irq_lock();
            reg_irq_src0 = 0;
            reg_irq_src1 = 0;
            core_interrupt_disable();
            start = (void *)(flash_base + SLOT0_ZB_OFFSET);
        }
    }

    /* print the start address for debugging */
    printk("start address: 0x%x\n", (uint32_t)start);
#else
    /* Lock interrupts and dive into the entry point */
    irq_lock();
#endif
    ((void (*)(void))start)();
}
#endif

#if defined(CONFIG_LOG) && !defined(ZEPHYR_LOG_MODE_IMMEDIATE) && \
    !defined(CONFIG_LOG_PROCESS_THREAD) && !defined(ZEPHYR_LOG_MODE_MINIMAL)
/* The log internal thread for log processing can't transfer log well as has too
 * low priority.
 * Dedicated thread for log processing below uses highest application
 * priority. This allows to transmit all logs without adding k_sleep/k_yield
 * anywhere else int the code.
 */

/* most simple log processing theread */
void boot_log_thread_func(void *dummy1, void *dummy2, void *dummy3)
{
    (void)dummy1;
    (void)dummy2;
    (void)dummy3;

    log_init();

    while (1) {
#if defined(CONFIG_LOG1) || defined(CONFIG_LOG2)
        /* support Zephyr legacy logging implementation before commit c5f2cde */
        if (log_process(false) == false) {
#else
        if (log_process() == false) {
#endif
            if (boot_log_stop) {
                break;
            }
            k_sleep(BOOT_LOG_PROCESSING_INTERVAL);
        }
    }

    k_sem_give(&boot_log_sem);
}

void zephyr_boot_log_start(void)
{
    /* start logging thread */
    k_thread_create(&boot_log_thread, boot_log_stack,
                    K_THREAD_STACK_SIZEOF(boot_log_stack),
                    boot_log_thread_func, NULL, NULL, NULL,
                    K_HIGHEST_APPLICATION_THREAD_PRIO, 0,
                    BOOT_LOG_PROCESSING_INTERVAL);

    k_thread_name_set(&boot_log_thread, "logging");
}

void zephyr_boot_log_stop(void)
{
    boot_log_stop = true;

    /* wait until log procesing thread expired
     * This can be reworked using a thread_join() API once a such will be
     * available in zephyr.
     * see https://github.com/zephyrproject-rtos/zephyr/issues/21500
     */
    (void)k_sem_take(&boot_log_sem, K_FOREVER);
}
#endif /* defined(CONFIG_LOG) && !defined(ZEPHYR_LOG_MODE_IMMEDIATE) && \
        * !defined(CONFIG_LOG_PROCESS_THREAD) && !defined(ZEPHYR_LOG_MODE_MINIMAL)
        */

#if defined(CONFIG_BOOT_SERIAL_ENTRANCE_GPIO) || defined(CONFIG_BOOT_USB_DFU_GPIO)

#ifdef CONFIG_MCUBOOT_SERIAL
#define BUTTON_0_DETECT_DELAY CONFIG_BOOT_SERIAL_DETECT_DELAY
#else
#define BUTTON_0_DETECT_DELAY CONFIG_BOOT_USB_DFU_DETECT_DELAY
#endif

#define BUTTON_0_NODE DT_ALIAS(mcuboot_button0)

#if DT_NODE_EXISTS(BUTTON_0_NODE) && DT_NODE_HAS_PROP(BUTTON_0_NODE, gpios)
static const struct gpio_dt_spec button0 = GPIO_DT_SPEC_GET(BUTTON_0_NODE, gpios);
#else
#error "Serial recovery/USB DFU button must be declared in device tree as 'mcuboot_button0'"
#endif

static bool detect_pin(void)
{
    int rc;
    int pin_active;

    if (!device_is_ready(button0.port)) {
        __ASSERT(false, "GPIO device is not ready.\n");
        return false;
    }

    rc = gpio_pin_configure_dt(&button0, GPIO_INPUT);
    __ASSERT(rc == 0, "Failed to initialize boot detect pin.\n");

    rc = gpio_pin_get_dt(&button0);
    pin_active = rc;

    __ASSERT(rc >= 0, "Failed to read boot detect pin.\n");

    if (pin_active) {
        if (BUTTON_0_DETECT_DELAY > 0) {
#ifdef CONFIG_MULTITHREADING
            k_sleep(K_MSEC(50));
#else
            k_busy_wait(50000);
#endif

            /* Get the uptime for debounce purposes. */
            int64_t timestamp = k_uptime_get();

            for(;;) {
                rc = gpio_pin_get_dt(&button0);
                pin_active = rc;
                __ASSERT(rc >= 0, "Failed to read boot detect pin.\n");

                /* Get delta from when this started */
                uint32_t delta = k_uptime_get() -  timestamp;

                /* If not pressed OR if pressed > debounce period, stop. */
                if (delta >= BUTTON_0_DETECT_DELAY || !pin_active) {
                    break;
                }

                /* Delay 1 ms */
#ifdef CONFIG_MULTITHREADING
                k_sleep(K_MSEC(1));
#else
                k_busy_wait(1000);
#endif
            }
        }
    }

    return (bool)pin_active;
}
#endif

#ifdef CONFIG_MCUBOOT_SERIAL
static void boot_serial_enter()
{
    int rc;

#ifdef CONFIG_MCUBOOT_INDICATION_LED
    gpio_pin_set_dt(&led0, 1);
#endif

    mcuboot_status_change(MCUBOOT_STATUS_SERIAL_DFU_ENTERED);

    BOOT_LOG_INF("Enter the serial recovery mode");
    rc = boot_console_init();
    __ASSERT(rc == 0, "Error initializing boot console.\n");
    boot_serial_start(&boot_funcs);
    __ASSERT(0, "Bootloader serial process was terminated unexpectedly.\n");
}
#endif

#include <zephyr/drivers/uart.h>
#include <zephyr/sys/crc.h>

void print_buffer(uint8_t *buffer, size_t size)
{
	for (size_t i = 0; i < size; i++) {
		printk("%c", buffer[i]);
	}
}

#define HEADER_MAGIC1  0xAA
#define HEADER_MAGIC2  0x12
#define TRAILER_MAGIC  0x55

/* Vendor-specific code executed during MCUBoot startup */
void telink_mcu_boot_startup(void)
{
	/* BOOT_LOG_INF("Telink MCUBoot on early boot"); */
#if (CONFIG_SOC_RISCV_TELINK_B92 || CONFIG_SOC_RISCV_TELINK_TL321X)
#if CONFIG_SOC_RISCV_TELINK_B92
	#include <zephyr/dt-bindings/pinctrl/b92-pinctrl.h>
#elif CONFIG_SOC_RISCV_TELINK_TL321X
	#include <zephyr/dt-bindings/pinctrl/tl321x-pinctrl.h>
#endif

	bool show_chip_id = false;

	/* Check if the console UART RX line is held low (shorted to ground). */
	const struct device *const uart_con = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

	if (device_is_ready(uart_con)) {

		/* Use the uart corresponding to your zephyr_console configuration */
		#define UART_RX_PINMUX \
			DT_PROP(DT_PINCTRL_BY_IDX(DT_NODELABEL(uart0), 0, 1), pinmux)

		#if CONFIG_SOC_RISCV_TELINK_B92
			gpio_pin_e uart_rx = B9x_PINMUX_GET_PIN(UART_RX_PINMUX);
		#elif CONFIG_SOC_RISCV_TELINK_TL321X
			gpio_pin_e uart_rx = TLX_PINMUX_GET_PIN(UART_RX_PINMUX);
		#endif

		/* Disable The UART RX PIN */
		gpio_set_low_level(uart_rx);
		gpio_output_dis(uart_rx);
		gpio_function_dis(uart_rx);

		/* Enable The UART RX PIN GPIO Function */
		gpio_function_en(uart_rx);
		gpio_input_en(uart_rx);
		gpio_set_up_down_res(uart_rx, GPIO_PIN_PULLUP_10K);

		/* Check if Console UART RX PIN is at low level - shorted to ground */
		if (gpio_get_level(uart_rx) == 0) {
			show_chip_id = true;
		}
	} else {
		BOOT_LOG_ERR("UART console device not ready");
	}

	if (show_chip_id) {
		uint8_t chip_id[21] = {0};
		#if CONFIG_SOC_RISCV_TELINK_B92
			extern unsigned char efuse_get_chip_id(unsigned char *chip_id_buff);
			if (efuse_get_chip_id(chip_id + 2))
		#elif CONFIG_SOC_RISCV_TELINK_TL321X
			extern drv_api_status_e efuse_get_chip_id(unsigned char *chip_id_buff);
			if (efuse_get_chip_id(chip_id + 2) == DRV_API_SUCCESS)
		#endif
			{
				uint16_t chip_id_crc = crc16_itu_t(0, chip_id + 2, 16);
				chip_id[0] = HEADER_MAGIC1;
				chip_id[1] = HEADER_MAGIC2;
				chip_id[18] = chip_id_crc & 0x00ff;
				chip_id[19] = chip_id_crc >> 8;
				chip_id[20] = TRAILER_MAGIC;
				print_buffer(chip_id, sizeof(chip_id));
			} else {
				BOOT_LOG_ERR("Failed to read Chip ID");
			}
	}
#endif /*(CONFIG_SOC_RISCV_TELINK_B92 || CONFIG_SOC_RISCV_TELINK_TL321X)*/
}

void main(void)
{
    struct boot_rsp rsp;
    int rc;
    FIH_DECLARE(fih_rc, FIH_FAILURE);

#ifdef CONFIG_BOOT_SERIAL_BOOT_MODE
    int32_t boot_mode;
#endif

#ifdef CONFIG_BOOT_SERIAL_PIN_RESET
    uint32_t reset_cause;
#endif

    MCUBOOT_WATCHDOG_SETUP();
    MCUBOOT_WATCHDOG_FEED();

#if !defined(MCUBOOT_DIRECT_XIP)
    BOOT_LOG_INF("Starting bootloader");
#else
    BOOT_LOG_INF("Starting Direct-XIP bootloader");
#endif

#ifdef CONFIG_MCUBOOT_INDICATION_LED
    /* LED init */
    led_init();
#endif

    os_heap_init();

    ZEPHYR_BOOT_LOG_START();

    telink_mcu_boot_startup();

    (void)rc;

    mcuboot_status_change(MCUBOOT_STATUS_STARTUP);

#ifdef CONFIG_BOOT_SERIAL_ENTRANCE_GPIO
    if (detect_pin() &&
            !boot_skip_serial_recovery()) {
        boot_serial_enter();
    }
#endif

#ifdef CONFIG_BOOT_SERIAL_PIN_RESET
    rc = hwinfo_get_reset_cause(&reset_cause);

    if (rc == 0 && reset_cause == RESET_PIN) {
        (void)hwinfo_clear_reset_cause();
        boot_serial_enter();
    }
#endif

#if defined(CONFIG_BOOT_USB_DFU_GPIO)
    if (detect_pin()) {
#ifdef CONFIG_MCUBOOT_INDICATION_LED
        gpio_pin_set_dt(&led0, 1);
#endif

        mcuboot_status_change(MCUBOOT_STATUS_USB_DFU_ENTERED);

        rc = usb_enable(NULL);
        if (rc) {
            BOOT_LOG_ERR("Cannot enable USB");
        } else {
            BOOT_LOG_INF("Waiting for USB DFU");
            wait_for_usb_dfu(K_FOREVER);
            BOOT_LOG_INF("USB DFU wait time elapsed");
        }
    }
#elif defined(CONFIG_BOOT_USB_DFU_WAIT)
    rc = usb_enable(NULL);
    if (rc) {
        BOOT_LOG_ERR("Cannot enable USB");
    } else {
        BOOT_LOG_INF("Waiting for USB DFU");

        mcuboot_status_change(MCUBOOT_STATUS_USB_DFU_WAITING);

        wait_for_usb_dfu(K_MSEC(CONFIG_BOOT_USB_DFU_WAIT_DELAY_MS));
        BOOT_LOG_INF("USB DFU wait time elapsed");

        mcuboot_status_change(MCUBOOT_STATUS_USB_DFU_TIMED_OUT);
    }
#endif

#ifdef CONFIG_BOOT_SERIAL_WAIT_FOR_DFU
    /* Initialize the boot console, so we can already fill up our buffers while
     * waiting for the boot image check to finish. This image check, can take
     * some time, so it's better to reuse thistime to already receive the
     * initial mcumgr command(s) into our buffers
     */
    rc = boot_console_init();
    int timeout_in_ms = CONFIG_BOOT_SERIAL_WAIT_FOR_DFU_TIMEOUT;
    uint32_t start = k_uptime_get_32();
#endif

    FIH_CALL(boot_go, fih_rc, &rsp);

#ifdef CONFIG_BOOT_SERIAL_BOOT_MODE
    boot_mode = bootmode_check(BOOT_MODE_TYPE_BOOTLOADER);

    if (boot_mode == 1) {
        /* Boot mode to stay in bootloader, clear status and enter serial
         * recovery mode
         */
        bootmode_clear();
        boot_serial_enter();
    }
#endif

#ifdef CONFIG_BOOT_SERIAL_WAIT_FOR_DFU
    timeout_in_ms -= (k_uptime_get_32() - start);
    if( timeout_in_ms <= 0 ) {
        /* at least one check if time was expired */
        timeout_in_ms = 1;
    }
    boot_serial_check_start(&boot_funcs,timeout_in_ms);
#endif

    if (FIH_NOT_EQ(fih_rc, FIH_SUCCESS)) {
        BOOT_LOG_ERR("Unable to find bootable image");

        mcuboot_status_change(MCUBOOT_STATUS_NO_BOOTABLE_IMAGE_FOUND);

#ifdef CONFIG_BOOT_SERIAL_NO_APPLICATION
        /* No bootable image and configuration set to remain in serial
         * recovery mode
         */
        boot_serial_enter();
#endif

        FIH_PANIC;
    }

    BOOT_LOG_INF("Bootloader chainload address offset: 0x%x",
                 rsp.br_image_off);

#if defined(MCUBOOT_DIRECT_XIP)
    BOOT_LOG_INF("Jumping to the image slot");
#else
    BOOT_LOG_INF("Jumping to the first image slot");
#endif

    mcuboot_status_change(MCUBOOT_STATUS_BOOTABLE_IMAGE_FOUND);

    ZEPHYR_BOOT_LOG_STOP();
    do_boot(&rsp);

    mcuboot_status_change(MCUBOOT_STATUS_BOOT_FAILED);

    BOOT_LOG_ERR("Never should get here");
    while (1)
        ;
}
