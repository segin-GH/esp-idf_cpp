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
    xQueueHandle sndQueue;
    const int sndQueueLen = 5;
    spi_slave_transaction_t t;
    WORD_ALIGNED_ATTR char dataBuff[130] = "";

public:
    WORD_ALIGNED_ATTR char recvBuf[129] = "";

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

        //  TODO implement proper Size for queue
        sndQueue = xQueueCreate(sndQueueLen, sizeof(130));
        memset(&t, 0, sizeof(t));
    }

    void dataToSend(char data[])
    {
        /* TODO portMAX_DELAY really code smells bad ..... */
        long err = xQueueSend(sndQueue, &data, 1500 / portTICK_PERIOD_MS);
        if (!err)
            std::cout << "UNABLE TO SEND TO QUEUE" << std::endl;
    }

    esp_err_t transmit(spi_host_device_t host)
    {
        /* TODO can spi_transaction_t in private ? */
        // spi_slave_transaction_t t;
        // memset(&t, 0, sizeof(t));

        if (xQueueReceive(sndQueue, &dataBuff, 5000 / portTICK_PERIOD_MS))
        {
            std::cout << "THIS IS AFTER QUEUE IS RECEIVED " << dataBuff << std::endl;
            // TODO fix hardcoded value
            t.length = (129 + 129) * 8;
            t.tx_buffer = dataBuff;
            t.rx_buffer = recvBuf;
            // t.user = (void *)0; // Set the slave ID to 0
            // TODO figure out a way to use m_host rather than passing it as args in function.
            return spi_slave_transmit(host, &t, portMAX_DELAY);
        }
        std::cout << "UNABLE TO SEND DATA" << std::endl;
        return ESP_FAIL;
    }
};

SpiSlave spi_slave(HSPI_HOST, GPIO_MOSI, GPIO_MISO, GPIO_SCLK);

void sendDataThroughSPI(void *args)
{
    for (;;)
    {
        std::cout << "INSIDE SEND LOOP " << intr_trig << std::endl;
        // TODO implement a mutex or task suspend
        if (masterSelectedMe)
        {
            esp_err_t err = spi_slave.transmit(HSPI_HOST);
            if (err == ESP_OK)
                std::cout << spi_slave.recvBuf << std::endl;
        }
        else
            vTaskDelay(100);
    }
}

void logWithUART(void *args)
{
    int n = 0;
    WORD_ALIGNED_ATTR char sendBuf[129] = "";

    for (;;)
    {
        ++n;
        memset(sendBuf, 0, sizeof(sendBuf));
        sprintf(sendBuf, "This is the receiver %i", n);
        spi_slave.dataToSend(sendBuf);
        std::cout << "THIS IS AFTER SND TO QUEUE " << sendBuf << std::endl;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

extern "C" void app_main()
{
    std::cout << "SPI SLAVE" << std::endl;

    gpio_set_direction((gpio_num_t)CHIP_SELECT, GPIO_MODE_INPUT);
    gpio_pullup_en((gpio_num_t)CHIP_SELECT);
    gpio_set_intr_type((gpio_num_t)CHIP_SELECT, GPIO_INTR_ANYEDGE);

    gpio_install_isr_service(0);
    gpio_isr_handler_add((gpio_num_t)CHIP_SELECT, gpio_isr_handler, NULL);

    xTaskCreatePinnedToCore(
        sendDataThroughSPI,
        "sendDataThroughSPI",
        2048,
        NULL,
        2,
        NULL,
        APP_CPU_NUM);

    xTaskCreatePinnedToCore(
        logWithUART,
        "logWithUART",
        2048,
        NULL,
        2,
        NULL,
        APP_CPU_NUM);

    // WORD_ALIGNED_ATTR char recvBuf[129] = "";

    // for (;;)
    // {
    //     std::cout << "INSIDE WHILE LOOP" << intr_trig << std::endl;
    //     if (masterSelectedMe)
    //     {
    //         ++n;
    //         memset(recvBuf, 0, sizeof(recvBuf));

    //         esp_err_t ret = spi_slave.transmit(HSPI_HOST, sendBuf, transferBufSize, recvBuf, transferBufSize);
    //         if (ret == ESP_OK)
    //             std::cout << recvBuf << std::endl;
    //         vTaskDelay(600 / portTICK_PERIOD_MS);
    //     }
    //     else
    //         vTaskDelay(600 / portTICK_PERIOD_MS);
    // }
}