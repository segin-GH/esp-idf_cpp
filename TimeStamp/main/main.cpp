#include <iostream>
#include <string>
#include <ctime>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sys/time.h>

extern "C" {
    void app_main(void);
}

struct timeval tv;

std::string getTimeStamp(struct timeval &_tv)
{
    gettimeofday(&tv, NULL);
    std::string data = std::to_string(time(NULL)); 
    return data;
}

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
        std::string data = getTimeStamp(tv);
	int64_t time_us = (int64_t)tv.tv_sec * 1000000L + (int64_t)tv.tv_usec;
        std::string uartData = "luffy";
        data = data + ":" + uartData;
        std::cout << data << std::endl;
        vTaskDelay(2000/portTICK_PERIOD_MS);
    }

}
