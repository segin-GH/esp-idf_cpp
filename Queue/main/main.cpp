#include <iostream>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

xQueueHandle queue;
int queueLen = 10;

extern "C" void app_main(void)
{
    for (;;)
    {
        queue = xQueueCreate(queueLen, sizeof(std::string));
        std::string str = "data";
        xQueueSend(queue, &str, portMAX_DELAY);

        std::string recvStr;
        xQueueReceive(queue, &recvStr, portMAX_DELAY);

        std::cout << recvStr << std::endl;
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}