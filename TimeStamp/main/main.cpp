#include <iostream>
#include <ctime>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sys/time.h>


extern "C" {
    void app_main(void);
}

struct timeval tv;

//TODO timestamp 

void app_main(void)
{
    tv.tv_sec = 1666945715;
    tv.tv_usec = 0;

    int err = settimeofday(&tv, NULL);
    if (err != 0)
        esp_restart();
    while(true)
    {
        gettimeofday(&tv, NULL);
        std::cout << time(NULL) << std::endl;\
        vTaskDelay(2000/portTICK_PERIOD_MS);
    }
}

