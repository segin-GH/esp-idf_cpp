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

// bool masterSelectedMe = pdFALSE;
// int intr_trig = 0;

// static void IRAM_ATTR gpio_isr_handler(void *args)
// {
//     ++intr_trig;

//     if (gpio_get_level((gpio_num_t)CHIP_SELECT) == 0)
//         masterSelectedMe = pdTRUE;
//     else
//         masterSelectedMe = pdFALSE;
// }

class SpiSlave
{
private:
    spi_host_device_t m_host;
    xQueueHandle spiSndQueue;
    const int spiSndQueueLen = 5;
    WORD_ALIGNED_ATTR char dataBuff[150] = "";
    static bool masterSelectedMe = pdFALSE;
    int intr_trig = 0;


    static void IRAM_ATTR gpio_isr_handler(void *arg)
    {
        ++intr_trig;

        if (gpio_get_level((gpio_num_t)CHIP_SELECT) == 0)
            masterSelectedMe = pdTRUE;
        else
            masterSelectedMe = pdFALSE;
    }

public:
    WORD_ALIGNED_ATTR char recvBuff[150] = "";

public:
    SpiSlave(spi_host_device_t host, int mosi, int miso, int sclk, int cs)
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

        spiSndQueue = xQueueCreate(spiSndQueueLen, sizeof(dataBuff));

        gpio_set_direction((gpio_num_t)cs, GPIO_MODE_INPUT);
        gpio_pullup_en((gpio_num_t)cs);
        gpio_set_intr_type((gpio_num_t)cs, GPIO_INTR_ANYEDGE);

        gpio_install_isr_service(0);
        gpio_isr_handler_add((gpio_num_t)cs, gpio_isr_handler, NULL);
    }

    void dataToSend(char txData[])
    {
        long err = xQueueSend(spiSndQueue, &txData, portMAX_DELAY);
        if (!err)
            std::cout << "Could Not Send queue FULL" << std::endl;
        else
            memset(txData, 0, sizeof(130));

        // TODO  this memset needed ?
    }

    esp_err_t transmit(spi_host_device_t host)
    {
        /* TODO can spi_transaction_t in private ? */
        if (masterSelectedMe)
        {
            spi_slave_transaction_t t;
            memset(&t, 0, sizeof(t));
            if (xQueueReceive(spiSndQueue, &t.tx_buffer, portMAX_DELAY))
            {
                t.length = (130 + 130) * 8;
                // t.tx_buffer = txBuff;
                t.rx_buffer = recvBuff;
                // t.user = (void *)0; // Set the slave ID to 0
                // TODO figure out a way to use m_host rather than passing it as args in function.
                return spi_slave_transmit(host, &t, portMAX_DELAY);
            }
        }
        return ESP_FAIL;
    }
};

SpiSlave spi_slave(HSPI_HOST, GPIO_MOSI, GPIO_MISO, GPIO_SCLK, CHIP_SELECT);

void logWithUART(void *args)
{
    WORD_ALIGNED_ATTR char sendBuf[129] = "";

    int count = 0;
    while (true)
    {
        sprintf(sendBuf, "uartDATA %i", count);
        spi_slave.dataToSend(sendBuf);
        memset(sendBuf, 0, sizeof(sendBuf));
        ++count;
        vTaskDelay(600 / portTICK_PERIOD_MS);
    }
}
void sendDataThroughSPI(void *args)
{
    spi_slave.transmit(HSPI_HOST);
}

extern "C" void app_main()
{
    std::cout << "SPI SLAVE" << std::endl;

    xTaskCreatePinnedToCore(
        logWithUART,
        "logWithUART",
        2048,
        NULL,
        2,
        NULL,
        APP_CPU_NUM);

    xTaskCreatePinnedToCore(
        sendDataThroughSPI,
        "sendDataThroughSPI",
        2048,
        NULL,
        2,
        NULL,
        APP_CPU_NUM);

    // for (;;)
    // {
    //     // std::cout << "INSIDE WHILE LOOP" << intr_trig << std::endl;

    //     ++n;
    //     memset(recvBuf, 0, sizeof(recvBuf));
    //     sprintf(sendBuf, "This is the receiver %i", n);

    //     spi_slave.dataToSend(sendBuf);
    //     vTaskDelay(600 / portTICK_PERIOD_MS);

    //     std::cout << spi_slave.recvBuff << std::endl;

    //     // if (masterSelectedMe)
    //     // {
    //     //     ++n;
    //     //     memset(sendBuf, 0, sizeof(sendBuf));
    //     //     memset(recvBuf, 0, sizeof(recvBuf));
    //     //     sprintf(sendBuf, "This is the receiver %i", n);

    //     //     esp_err_t ret = spi_slave.transmit(HSPI_HOST, sendBuf, transferBufSize, recvBuf, transferBufSize);
    //     //     if (ret == ESP_OK)
    //     //         std::cout << recvBuf << std::endl;
    //     //     vTaskDelay(600 / portTICK_PERIOD_MS);
    //     // }
    //     // else
    //     //     vTaskDelay(600 / portTICK_PERIOD_MS);
    // }
}
