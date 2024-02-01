#include "trf79xxa.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define CS_ENABLE       0
#define CS_DISABLE      1
#define CS_GPIO         10

// Only part MCU specific
#define SPI_ENABLE                          gpio_set_level(CS_GPIO, CS_ENABLE)
#define SPI_DISABLE                         gpio_set_level(CS_GPIO, CS_DISABLE)


trf79xxa_error_t trf79xxa_init(trf79xxa_dev_t *dev, trf79xxa_power_t power, trf79xxa_protocols_t protocols, trf79xxa_write_byte_cb write_cb, trf79xxa_read_byte_cb read_cb)
{

    if (!dev || !write_cb || !read_cb)
        return TRF_INVALID_PARAM;

    dev->state = STATE_IDLE;
    dev->protocols = protocols;
    dev->power = power;
    dev->iso_control = 0x02;
    dev->write_cb = write_cb;
    dev->read_cb = read_cb;
    trf79xxa_reset(dev);

    /*trf79xxa_write_register(&trf_handle, REG_REG)
    trf79xxa_write_register(&trf_handle, REG_NFC_TARGET_DETECTION_LEVEL, 0x00, 1);*/
    return TRF_SUCCESS;
}

trf79xxa_error_t trf79xxa_reset(trf79xxa_dev_t *dev)
{
    trf79xxa_send_cmd(dev, CMD_TRF_INIT);
    trf79xxa_send_cmd(dev, CMD_TRF_IDLE);
    trf79xxa_reset_fifo(dev);
    
    trf79xxa_write_register(dev, REG_MODULATOR_SYS_CONTROL, (void*) 0x01, 1);
    return TRF_SUCCESS;
}

trf79xxa_error_t trf79xxa_reset_fifo(trf79xxa_dev_t *dev)
{
    trf79xxa_send_cmd(dev, CMD_TRF_RESET_FIFO);
    vTaskDelay(pdMS_TO_TICKS(2));
    return TRF_SUCCESS;
}

trf79xxa_error_t trf79xxa_set_rf(trf79xxa_dev_t *dev, bool status)
{
    return TRF_SUCCESS;
}

trf79xxa_error_t trf79xxa_read_register(trf79xxa_dev_t *dev, uint8_t reg, uint8_t *value, uint8_t length)
{
    SPI_ENABLE;
    dev->write_cb(reg, dev->user_data);
    for (int i = 0; i < (length); i++) {
        dev->read_cb(value, dev->user_data);
    }
    SPI_DISABLE;
    return TRF_SUCCESS;
}

trf79xxa_error_t trf79xxa_write_register(trf79xxa_dev_t *dev, uint8_t reg, uint8_t *value, uint8_t length)
{
    if (length > 1) {
        //reg = (0x20 | reg);
    } else {
        //reg = (0x80 | reg);
    }
    SPI_ENABLE;
    dev->write_cb(reg, dev->user_data);
    for (int i = 0; i < (length-1); i++) {
        dev->write_cb(value[i], dev->user_data);
    }
    SPI_DISABLE;
    return TRF_SUCCESS;
}

trf79xxa_error_t trf79xxa_send_cmd(trf79xxa_dev_t *dev, uint8_t cmd)
{
    SPI_ENABLE;
    cmd = (0x80 | cmd);			
	cmd = (0x9f & cmd);
    dev->write_cb(cmd, dev->user_data);
    SPI_DISABLE;
    return TRF_SUCCESS;
}
