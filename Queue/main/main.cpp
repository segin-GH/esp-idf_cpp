#include <iostream>
#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

xQueueHandle queue;
int queueLen = 10;

extern "C" void app_main(void)
{
    queue = xQueueCreate(queueLen, (sizeof(char) * 1000 ));
    
    for (;;)
    {
        char str[1000] ="abcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyz";
        xQueueSend(queue, str, portMAX_DELAY);

        char recvBuff[1000] = "";
        memset(recvBuff, 0 ,sizeof (1000));
        xQueueReceive(queue, &recvBuff, portMAX_DELAY);

        std::cout << recvBuff << std::endl;
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}