// #include <stdio.h>
#include <iostream>
#include <cstring>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "esp_system.h"
#include "esp_event.h"
#include "driver/spi_master.h"
#include "esp_log.h"

#include "driver/gpio.h"
#include "esp_intr_alloc.h"

#define GPIO_MOSI 23
#define GPIO_MISO 19
#define GPIO_SCLK 18
#define GPIO_CS 15

#define transferBufSize 130

class SpiMaster
{
private:
    spi_device_handle_t m_spi;
    const int sndQueueLen = 1;
    xQueueHandle sndQueue;
    WORD_ALIGNED_ATTR char dataBuff[150] = "";

public:
    SpiMaster(spi_host_device_t host, int mosi, int miso, int sclk, int cs)
    {
        spi_bus_config_t buscfg = {
            .mosi_io_num = mosi,
            .miso_io_num = miso,
            .sclk_io_num = sclk,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
        };

        spi_device_interface_config_t devcfg = {
            .command_bits = 0,
            .address_bits = 0,
            .dummy_bits = 0,
            .mode = 0,
            .duty_cycle_pos = 128, // 50% duty cycle
            .cs_ena_posttrans = 3, // Keep the CS low 3 cycles after transaction, to stop slave from missing the last bit when CS has less propagation delay than CLK
            .clock_speed_hz = 5000000,
            .spics_io_num = -1, // -1 bcz no slave select pin;
            .queue_size = 6,
        };

        esp_err_t ret;
        ret = spi_bus_initialize(host, &buscfg, SPI_DMA_CH_AUTO);
        assert(ret == ESP_OK);
        ret = spi_bus_add_device(host, &devcfg, &m_spi);
        assert(ret == ESP_OK);

        /* SLAVE SELECT PIN CONFIG  */
        gpio_pad_select_gpio((gpio_num_t)cs);
        gpio_set_direction((gpio_num_t)cs, GPIO_MODE_OUTPUT);

        sndQueue = xQueueCreate(sndQueueLen, sizeof(dataBuff));
    }

    void selectSlave(int SlaveNum, int level)
    {
        gpio_set_level((gpio_num_t)SlaveNum, level);
    }

    void sendToSlave(char *data)
    {

        long err = xQueueSend(sndQueue, data, 1500 / portTICK_PERIOD_MS);
        if (!err)
        {
            printf("[queue] Could not add to queue\n.");
        }
    }

    esp_err_t transfer(char *rxBuff)
    {
        WORD_ALIGNED_ATTR char sendBuf[150] = "";
        /* TODO spi transcation should be private? */
        if (xQueueReceive(sndQueue, sendBuf, portMAX_DELAY))
        {
            spi_transaction_t t;
            memset(&t, 0, sizeof(t));
            /* TODO code smells bad can we hardcode the value of the length ? */
            t.length = (130 + 130) * 8;
            t.tx_buffer = sendBuf;
            t.rx_buffer = rxBuff;
            // t.user = (void *)0; // Set the slave ID to 0
            return spi_device_transmit(m_spi, &t);
        }
        return ESP_FAIL;
    }
};

extern "C" void app_main(void)
{
    std::cout << "SPI MASTER" << std::endl;
    SpiMaster spi_master(HSPI_HOST, GPIO_MOSI, GPIO_MISO, GPIO_SCLK, GPIO_CS);

    char sendBuf[130] = "";
    char recvBuf[130] = "";
    memset(sendBuf, 0, sizeof(sendBuf));
    memset(recvBuf, 0, sizeof(recvBuf));

    spi_master.selectSlave(15, 0);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    for (int i = 0; i < 100; i++)
    {
        memset(sendBuf, 0, sizeof(sendBuf));
        memset(recvBuf, 0, sizeof(recvBuf));

        sprintf(sendBuf, "%i I am Master obey Slaves", i);
        // if (res >= sizeof(sendBuf))
        // printf("Data truncated\n");

        spi_master.sendToSlave(sendBuf);

        esp_err_t ret = spi_master.transfer(recvBuf);
        if (ret == ESP_OK)
            std::cout << recvBuf << std::endl;
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
    spi_master.selectSlave(15, 1);

    for (;;)
        vTaskDelete(NULL);
}
