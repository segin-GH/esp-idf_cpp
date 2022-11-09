#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include <esp_spiffs.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define TAG "SPIFFS" 

void app_main(void)
{
    esp_vfs_spiffs_conf_t config = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true,
    };
    esp_vfs_spiffs_register(&config);   


// opening and reading dir name & size;

    DIR *dir = opendir("/spiffs");
    struct dirent *entry;
    while((entry = readdir(dir)) != NULL)
    {
        char fullpath[300];
        sprintf(fullpath,"/spiffs/%s", entry->d_name);
        // printf("%s\n",fullpath);
        struct stat entrystat;
        if(stat(fullpath, &entrystat) == -1)
        {
            ESP_LOGE("[FILE]","error getting stats for %s", fullpath);
        }
        else
        {
            ESP_LOGI("[FILE]","%s : %ld",fullpath, entrystat.st_size);
        }

    } 

// oepn and reading from a file;

/*  
    FILE *file = fopen("/spiffs/sub/data.txt", "r");
    if(file == NULL)
    {
        ESP_LOGE(TAG,"could not open file");
    }
    else
    {
        char line[255];
        while(fgets(line, sizeof(line), file) != NULL)
        {
            vTaskDelay(10/portTICK_PERIOD_MS);
            printf(line);
        }
        fclose(file);
    }

*/
    esp_vfs_spiffs_unregister(NULL);


}
