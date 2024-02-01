#include <stdio.h>
#include <rom/ets_sys.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "string.h"

#include "trf79xxa.h"

#define MISO_GPIO        13
#define MOSI_GPIO        11
#define CLK_GPIO         12
#define CS_GPIO          10
#define IRQ_GPIO         21
#define EN_GPIO          20
#define EN2_GPIO         47

QueueHandle_t interputQueue;
static spi_device_handle_t spi_handle;

static void IRAM_ATTR gpio_interrupt_handler(void *args)
{
    int pinNumber = (int)args;
    xQueueSendFromISR(interputQueue, &pinNumber, NULL);
}

static void write_byte(uint8_t data, void* user_data) 
{
    spi_transaction_t trans = {
        .flags = SPI_TRANS_USE_TXDATA,
        .length = 8,
    };
    trans.tx_data[0] = data;
    ESP_ERROR_CHECK(spi_device_polling_transmit(*(spi_device_handle_t*) user_data, &trans));
}

static void read_byte(uint8_t* data, void* user_data) 
{
    spi_transaction_t trans = {
        .flags = SPI_TRANS_USE_RXDATA,
        .length = 8,
        .rxlength = 8,
        .tx_buffer = NULL,
    };
    ESP_ERROR_CHECK(spi_device_polling_transmit(*(spi_device_handle_t*) user_data, &trans));
    ESP_LOGD("DATA", "Recieved : %x %x %x %x", trans.rx_data[0], trans.rx_data[1], trans.rx_data[2], trans.rx_data[3]);
    *data = trans.rx_data[0];
}

void app_main(void)
{
    gpio_set_level(CS_GPIO, 0);
    gpio_set_level(EN_GPIO, 0);
    gpio_set_level(EN2_GPIO, 0);

    gpio_set_direction(CS_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(EN_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(EN2_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(IRQ_GPIO, GPIO_MODE_INPUT);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(IRQ_GPIO, gpio_interrupt_handler, (void*) IRQ_GPIO);
    gpio_set_intr_type(IRQ_GPIO, GPIO_INTR_POSEDGE);
    interputQueue = xQueueCreate(10, sizeof(int));

    // Start Procedure
    vTaskDelay(pdMS_TO_TICKS(2));
    gpio_set_level(CS_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(3));
    gpio_set_level(EN_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(EN2_GPIO, 1);

    //Initialize the SPI bus
    spi_bus_config_t buscfg = {
        .miso_io_num = MISO_GPIO,
        .mosi_io_num = MOSI_GPIO,
        .sclk_io_num = CLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 16
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 2 * 1000 * 1000,
        .mode = 1,
        .spics_io_num = -1,
        .queue_size = 5,           
    };

    memset(&spi_handle, 0, sizeof(spi_device_handle_t));
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_DISABLED));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi_handle));
    vTaskDelay(pdMS_TO_TICKS(1));

    // trf79 initialization
    trf79xxa_dev_t trf_handle;
    memset(&trf_handle, 0, sizeof(trf79xxa_dev_t));
    trf_handle.user_data = &spi_handle;
    trf79xxa_init(&trf_handle, TRF_5V_FULL_POWER, TRF_PROTOCOL_ALL, write_byte, read_byte);

    uint8_t status = 0;
    trf79xxa_read_register(&trf_handle, REG_RX_RESPONSE_TIME, &status, 1);

    int pinNumber;
    uint8_t irq_status = 0;
    while (true)
    {
        if (xQueueReceive(interputQueue, &pinNumber, 500)) {
            ESP_LOGW("GPIO", "Interrupt on pin %d", pinNumber);
            trf79xxa_read_register(&trf_handle, REG_IRQ_STATUS, &irq_status, 1);
            ESP_LOGI("APP", "IRQ : %d",  irq_status);
        }
    }
}