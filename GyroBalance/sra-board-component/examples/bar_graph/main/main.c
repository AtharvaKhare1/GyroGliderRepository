#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include "stdbool.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sra_board.h"
#include "esp_spiffs.h"
#include <errno.h>
#include "esp_log.h"

static const char *TAG = "TASK4";

void file_read_task(void*arg)
{
    motor_handle_t motor_a_0, motor_a_1;
    ESP_ERROR_CHECK(enable_motor_driver(&motor_a_0, MOTOR_A_0));
    ESP_ERROR_CHECK(enable_motor_driver(&motor_a_1, MOTOR_A_1));

    ESP_ERROR_CHECK(enable_bar_graph());

    esp_vfs_spiffs_conf_t config = {
        .base_path = "/storage",
        .partition_label = "storage",
        .max_files = 5,
        .format_if_mount_failed = true
    };

    esp_err_t result = esp_vfs_spiffs_register(&config);

    if(result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(result));
        return;
    }

    size_t total = 0, used = 0;
    result = esp_spiffs_info(config.partition_label, &total, &used);
    if(result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get partition info (%s)", esp_err_to_name(result));
    }
    else
    {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }

    FILE *ptr = fopen("/storage/task1.txt", "r");

    if (ptr == NULL) 
    {
        ESP_LOGE(TAG, "Error opening input file.");
        return;
    }

    uint8_t var1 = 0x00;
    uint8_t var2 = 0x00;
    uint8_t sum = 0x00;
    char buffer[1024];

    while (!feof(ptr))
    {
        if (fgets(buffer, sizeof(buffer), ptr) == NULL)
        {
            ESP_LOGE(TAG, "Error reading from file or end of file reached.");
            break; // breaking of loop at the end of the file
        }

        char data[3][20]; // array of 3 strings, each with max length 20

        sscanf(buffer, "%19s %19s %19s", data[0], data[1], data[2]);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        if (strcmp(data[0], "END") == 0) // end case
        {   
            ESP_LOGE(TAG, "PWMA: 00, PWMB: 00, END Case: STOPPING");
            set_motor_speed(motor_a_0, MOTOR_FORWARD, 0);
            set_motor_speed(motor_a_1, MOTOR_FORWARD, 0);

            var1 = 0xFF;
            ESP_ERROR_CHECK(set_bar_graph(var1));// all LEDs on

            break; // break the loop
        }
        else if (strcmp(data[1], "ONLYLEFT") == 0) // left only case
        {
            ESP_LOGE(TAG, "PWMA: 00, PWMB: 45, ONLYLEFT Case: Turning LEFT");

            var1 = 0x01;

            ESP_ERROR_CHECK(set_bar_graph(0x00));
            vTaskDelay(500 / portTICK_PERIOD_MS);

            for (int i = 0; i < 4; i++)
            {
                ESP_ERROR_CHECK(set_bar_graph(var1));
                vTaskDelay(1000 / portTICK_PERIOD_MS); // delay of 0.15 s
                var1 = var1 << 1;
            }

            ESP_ERROR_CHECK(set_bar_graph(0x00)); // turning off all LEDs after blinking
            
            set_motor_speed(motor_a_0, MOTOR_FORWARD, 0);
            set_motor_speed(motor_a_1, MOTOR_FORWARD, 45);
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
        else if (strcmp(data[1], "ONLYRIGHT") == 0) // right only case
        {
            ESP_LOGE(TAG, "PWMA: 45, PWMB: 00, ONLYRIGHT Case: Turning RIGHT");

            var1 = 0x08;

            ESP_ERROR_CHECK(set_bar_graph(0x00));
            vTaskDelay(500 / portTICK_PERIOD_MS);

            for (int i = 0; i < 4; i++)
            {
                ESP_ERROR_CHECK(set_bar_graph(var1));
                vTaskDelay(1000 / portTICK_PERIOD_MS); // delay of 0.15 s
                var1 = var1 >> 1;
            }

            ESP_ERROR_CHECK(set_bar_graph(0x00)); // turning off all LEDs after blinking

            set_motor_speed(motor_a_0, MOTOR_FORWARD, 45);
            set_motor_speed(motor_a_1, MOTOR_FORWARD, 0);
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
        else if (strcmp(data[1], "STRAIGHTANDLEFT") == 0) // straight and left case
        {
            ESP_LOGE(TAG, "PWMA: 00, PWMB: 45, STRAIGHTANDLEFT Case: Turning LEFT");

            var1 = 0x08;
            var2 = 0x01;
            sum = var2 + var1;

            ESP_ERROR_CHECK(set_bar_graph(0x00));
            vTaskDelay(500 / portTICK_PERIOD_MS);

            for (int i = 0; i <= 2; i++) // LED blink from ends to center
            {
                if (var1 == 0x00) 
                {
                    var1 = 0x08;
                    var2 = 0x01;
                    sum = var2 + var1; // for using var1 and var2 together as an 8-bit number
                }
                ESP_ERROR_CHECK(set_bar_graph(sum)); // Setting LEDs on

                var1 = var1 >> 1;
                var2 = var2 << 1;
                sum = var2 + var1; // for using var1 and var2 together as an 8-bit number

                vTaskDelay(1000 / portTICK_PERIOD_MS); // delay of 0.15 s
            }

            ESP_ERROR_CHECK(set_bar_graph(0x00)); // turning off all LEDs after blinking

            set_motor_speed(motor_a_0, MOTOR_FORWARD, 0);
            set_motor_speed(motor_a_1, MOTOR_FORWARD, 45);
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
        else if (strcmp(data[1], "STRAIGHTANDRIGHT") == 0) // straight and right case
        {
            ESP_LOGE(TAG, "PWMA: 80, PWMB: 80, STRAIGHTANDRIGHT Case: Going STRAIGHT");

            var1 = 0x08;
            var2 = 0x01;
            sum = var2 + var1;

            ESP_ERROR_CHECK(set_bar_graph(0x00));
            vTaskDelay(500 / portTICK_PERIOD_MS);

            for (int i = 0; i <= 2; i++) // LED blink from ends to center
            {
                if (var1 == 0x00)
                {
                    var1 = 0x08;
                    var2 = 0x01;
                    sum = var2 + var1; // for using var1 and var2 together as an 8-bit number
                }
                ESP_ERROR_CHECK(set_bar_graph(sum)); // Setting LEDs on

                var1 = var1 >> 1;
                var2 = var2 << 1;
                sum = var2 + var1; // for using var1 and var2 together as an 8-bit number

                vTaskDelay(1000 / portTICK_PERIOD_MS); // delay of 0.15 s
            }

            ESP_ERROR_CHECK(set_bar_graph(0x00)); // turning off all LEDs after blinking
            
            set_motor_speed(motor_a_0, MOTOR_FORWARD, 80);
            set_motor_speed(motor_a_1, MOTOR_FORWARD, 80);
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
        else if (strcmp(data[1], "PLUS") == 0) // plus junction
        {
            ESP_LOGE(TAG, "PWMA: 00, PWMB: 45, PLUS Case: Turning LEFT");

            var1 = 0xFF;
            var2 = 0x00;

            ESP_ERROR_CHECK(set_bar_graph(0x00));
            vTaskDelay(500 / portTICK_PERIOD_MS);

            for (int i = 0; i < 2; i++) // LED simultaneous blink
            {
                ESP_ERROR_CHECK(set_bar_graph(var1));
                vTaskDelay(1000 / portTICK_PERIOD_MS); // 0.15 s delay
                ESP_ERROR_CHECK(set_bar_graph(var2));
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }

            ESP_ERROR_CHECK(set_bar_graph(0x00));

            set_motor_speed(motor_a_0, MOTOR_FORWARD, 0);
            set_motor_speed(motor_a_1, MOTOR_FORWARD, 45);

            vTaskDelay(500 / portTICK_PERIOD_MS);            
        }
        else // straight only case
        {
            set_motor_speed(motor_a_0, MOTOR_FORWARD, 80);
            set_motor_speed(motor_a_1, MOTOR_FORWARD, 80);
        }
    }

    fclose(ptr); // Close the input file

    esp_vfs_spiffs_unregister(config.partition_label); // Unmount SPIFFS
    ESP_LOGI(TAG, "SPIFFS unmounted");
}

void app_main()
{
    xTaskCreate(&file_read_task, "file_read_task", 4096, NULL, 1, NULL);   
}

