#include <iostream>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"

#define GPIO_MOSI 23
#define GPIO_MISO 19
#define GPIO_SCLK 18
#define CHIP_SELECT 27

#define BUF_SIZE 1024
#define transferBufSize 130

bool masterSelectedMe = pdFALSE;
int intr_trig = 0;

// xQueueHandle queue;
// static const int queue_len = 5;

static void IRAM_ATTR gpio_isr_handler(void *args)
{
    ++intr_trig;

    if (gpio_get_level((gpio_num_t)CHIP_SELECT) == 0)
        masterSelectedMe = pdTRUE;
    else
        masterSelectedMe = pdFALSE;
}

class SpiSlave
{
private:
    spi_host_device_t m_host;
    const int queue_len = 20;
    xQueueHandle queue;
    WORD_ALIGNED_ATTR char dataBuff[150] = "";

public:
    SpiSlave(spi_host_device_t host, int mosi, int miso, int sclk)
    {
        this->m_host = host;
        esp_err_t ret;
        spi_bus_config_t buscfg = {
            .mosi_io_num = mosi,
            .miso_io_num = miso,
            .sclk_io_num = sclk,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
        };

        spi_slave_interface_config_t slvcfg = {
            .spics_io_num = -1, // -1 no chip select pin bcz i implemented my own slave select
            .flags = 0,
            .queue_size = 3,
            .mode = 0,
            .post_setup_cb = NULL,
            .post_trans_cb = NULL};

        // Enable pull-ups on SPI lines so we don't detect rogue pulses when no master is connected.
        gpio_set_pull_mode((gpio_num_t)mosi, GPIO_PULLUP_ONLY);
        gpio_set_pull_mode((gpio_num_t)sclk, GPIO_PULLUP_ONLY);
        // TODO should u dont need pull up on miso ?

        // Initialize SPI slave interface
        ret = spi_slave_initialize(host, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
        assert(ret == ESP_OK);

        queue = xQueueCreate(queue_len, sizeof(dataBuff));
    }
    void dataToSnd(char *data)
    {
        // std::cout << data << std::endl;
        long err = xQueueSend(queue, data, 1500 / portTICK_PERIOD_MS);
        if (!err)
        {
            printf("[queue] Could not add to queue. \n");
        }
    }

    esp_err_t transmit(spi_host_device_t host, char *recvBuf)
    {
        WORD_ALIGNED_ATTR char sendBuf[150] = "";

        if (xQueueReceive(queue, sendBuf, 5000 / portTICK_PERIOD_MS))
        {
            // std::cout << "xQueueReceive :: " << sendBuf << std::endl;
            /* TODO can spi_transaction_t in private ? */
            spi_slave_transaction_t t;
            memset(&t, 0, sizeof(t));
            t.length = (130 + 130) * 8;
            t.tx_buffer = sendBuf;
            t.rx_buffer = recvBuf;
            // t.user = (void *)0; // Set the slave ID to 0
            // TODO figure out a way to use m_host rather than passing it as args in function.
            return spi_slave_transmit(host, &t, portMAX_DELAY);
            memset(sendBuf, 0, sizeof(sendBuf));
        }
        return ESP_FAIL;
    }
};

SpiSlave spi_slave(HSPI_HOST, GPIO_MOSI, GPIO_MISO, GPIO_SCLK);

void logWithUART(void *args)
{
    int count = 0;
    while (true)
    {
        WORD_ALIGNED_ATTR char uartDataBuff[129] = "";
        sprintf(uartDataBuff, "uartDATA%i", count);
        spi_slave.dataToSnd(uartDataBuff);
        memset(uartDataBuff, 0, sizeof(uartDataBuff));
        ++count;
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
}

extern "C" void app_main()
{
    // int n = 0;
    // queue = xQueueCreate(queue_len, sizeof(uartDataBuff));
    std::cout << "SPI SLAVE" << std::endl;

    gpio_set_direction((gpio_num_t)CHIP_SELECT, GPIO_MODE_INPUT);
    gpio_pullup_en((gpio_num_t)CHIP_SELECT);
    gpio_set_intr_type((gpio_num_t)CHIP_SELECT, GPIO_INTR_ANYEDGE);

    gpio_install_isr_service(0);
    gpio_isr_handler_add((gpio_num_t)CHIP_SELECT, gpio_isr_handler, NULL);

    xTaskCreatePinnedToCore(
        logWithUART,
        "logWithUART",
        2048,
        NULL,
        2,
        NULL,
        APP_CPU_NUM);

    // WORD_ALIGNED_ATTR char sendBuf[129] = "";
    WORD_ALIGNED_ATTR char recvBuf[129] = "";

    for (;;)
    {
        // std::cout << "INSIDE WHILE LOOP" << intr_trig << std::endl;
        if (masterSelectedMe)
        {
            // memset(sendBuf, 0, sizeof(129));
            memset(recvBuf, 0, sizeof(129));
            // sprintf(sendBuf, "This is the receiver %i", n++);
            // // std::cout << "master slected me sending data " << sendBuf << std::endl;

            // if (xQueueReceive(queue, &sendBuf, 5000 / portTICK_PERIOD_MS))
            {
                esp_err_t ret = spi_slave.transmit(HSPI_HOST, recvBuf);
                if (ret == ESP_OK)
                    std::cout << recvBuf << std::endl;
                vTaskDelay(600 / portTICK_PERIOD_MS);
            }
            // }
            // else
            //     vTaskDelay(600 / portTICK_PERIOD_MS);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}