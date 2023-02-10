#include <esp_log.h>
#include <string>
#include <sys/time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

class Time
{
private:
    struct timeval tv;

public:
    std::string m_usec;
    std::string m_sec;

public:
    void setTime();
    std::string getTime();
};

void Time::setTime()
{
    tv.tv_sec = strtol(m_sec.c_str(), NULL, 10);
    if(m_usec.length() != 0)
        tv.tv_usec = strtol(m_usec.c_str(), NULL ,10);
    int err = settimeofday(&tv, NULL);
    if (err != 0)
    {
        ESP_LOGE("TIME CONFIG", "Unable to set time");
    }
}

std::string Time::getTime()
{
    gettimeofday(&tv, NULL);
    int64_t time_us = (int64_t)tv.tv_sec * 1000000L + (int64_t)tv.tv_usec;
    char buf[50];
    sprintf(buf, "ES1:%lld:", (unsigned long long)time_us);
    return std::string(buf);
}


extern "C" void app_main(void)
{
    Time time;
    time.m_sec = "1676022260";
    time.m_usec = "0";
    time.setTime();
    
    for(;;)
    {
        // Set the time to 1000000000 microseconds (1 second) from the epoch
        std::string str = time.getTime();
        ESP_LOGI("Time", "%s", str.c_str());
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}
