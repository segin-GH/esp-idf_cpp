#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/spi_slave.h"

#define GPIO_MOSI 23
#define SPI_SLAVE_MISO 19
#define SPI_SLAVE_SCLK 18
#define SPI_SLAVE_CS 27

#define BUF_SIZE 1024

void app_main(void)
{
    esp_err_t ret;
    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    // Configuration for the SPI slave interface
    spi_slave_interface_config_t slvcfg = {
        .mode = 0,
        .spics_io_num = -1, // -1 no chip select pin bcz i implemented my own slave select
        .queue_size = 3,
        .flags = 0,
        .post_setup_cb = NULL,
        .post_trans_cb = NULL};
    // Enable pull-ups on SPI lines so we don't detect rogue pulses when no master is connected.
    gpio_set_pull_mode(GPIO_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_SCLK, GPIO_PULLUP_ONLY);
    // gpio_set_pull_mode(GPIO_CS, GPIO_PULLUP_ONLY);

    // Initialize SPI slave interface
    ret = spi_slave_initialize(RCV_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
    assert(ret == ESP_OK);

    char buffer[BUF_SIZE];

    spi_slave_transaction_t t = {
        .length = BUF_SIZE * 8, // transaction length is in bits
        .rx_buffer = buffer,
        .tx_buffer = buffer,
        .user = NULL};
    while (1)
    {
        ret = spi_slave_transmit(HSPI_HOST, &t, portMAX_DELAY);
        assert(ret == ESP_OK);

        printf("Received %d bytes: ", BUF_SIZE);
        for (int i = 0; i < BUF_SIZE; i++)
        {
            printf("%02X ", buffer[i]);
        }
        printf("\n");
    }
}
