#include <dfu_trigger.h>

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>

#include <zephyr/storage/flash_map.h>
#include <zephyr/drivers/flash.h>

#include <string.h>

#define DFU_TRIGGER_OFFSET 0x0800

static const struct device *flash_device =
	DEVICE_DT_GET(DT_CHOSEN(zephyr_flash_controller));

bool isDfuTriggered(void)
{
    uint8_t key[] = {'d', 'f', 'u'};
    uint8_t read_key[sizeof(key)];
    int err = 0;

    err = flash_read(flash_device, FIXED_PARTITION_OFFSET(vendor_partition)
			+ DFU_TRIGGER_OFFSET, read_key, sizeof(key));

    if(err != 0)
    {
        return false;
    }
   
    if (memcmp(key, read_key, sizeof(key)) == 0)
    {
        return true;
    }

    return false;
}

void DfuTriggerReset(void)
{
    uint8_t key[] = {0xFF, 0xFF, 0xFF};

    flash_write(flash_device, FIXED_PARTITION_OFFSET(vendor_partition)
		+ DFU_TRIGGER_OFFSET, key, sizeof(key));
}
