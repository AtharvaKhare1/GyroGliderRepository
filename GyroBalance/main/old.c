/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "dshot_esc_encoder.h"
#include "sra_board.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "sdkconfig.h"
// #include "tuning_websocket_server.h"



#if CONFIG_IDF_TARGET_ESP32H2
#define DSHOT_ESC_RESOLUTION_HZ 32000000 // 32MHz resolution, DSHot protocol needs a relative high resolution
#else
#define DSHOT_ESC_RESOLUTION_HZ 40000000 // 40MHz resolution, DSHot protocol needs a relative high resolution
#endif
#define DSHOT_ESC_GPIO_NUM_1      15
#define DSHOT_ESC_GPIO_NUM_2      2
#define MIN_THROTTLE     100               // always above 50
#define MAX_THROTTLE     640
#define P_term 0.0
#define I_term 0.0
#define D_term 0.0

static const char *TAG = "example";
uint16_t disp;
uint16_t thro;
uint16_t test;
uint64_t start_time;
uint64_t current_time;
uint16_t delay;
int count = 0;

// plot_graph_data_t pg_data;
// QueueHandle_t plot_graph_queue = NULL;

void throttle_task(void *arg)
{
    ESP_LOGI(TAG, "Create RMT TX channel");
    rmt_channel_handle_t esc_chan_1 = NULL;
    rmt_channel_handle_t esc_chan_2 = NULL;

    rmt_tx_channel_config_t tx_chan_config_1 = { // update new pins
        .clk_src = RMT_CLK_SRC_DEFAULT, // select a clock that can provide needed resolution
        .gpio_num = DSHOT_ESC_GPIO_NUM_1,// enter pin number
        .mem_block_symbols = 64,
        .resolution_hz = DSHOT_ESC_RESOLUTION_HZ,
        .trans_queue_depth = 10, // set the number of transactions that can be pending in the background
    };
    rmt_tx_channel_config_t tx_chan_config_2 = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select a clock that can provide needed resolution
        .gpio_num = DSHOT_ESC_GPIO_NUM_2,// enter pin number
        .mem_block_symbols = 64,
        .resolution_hz = DSHOT_ESC_RESOLUTION_HZ,
        .trans_queue_depth = 10, // set the number of transactions that can be pending in the background

    };

    ESP_LOGI(TAG, "Enable RMT TX channel");

    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config_1, &esc_chan_1));
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config_2, &esc_chan_2));
    ESP_ERROR_CHECK(rmt_enable(esc_chan_1));
    ESP_ERROR_CHECK(rmt_enable(esc_chan_2));

    

    ESP_LOGI(TAG, "Install Dshot ESC encoder");
    rmt_encoder_handle_t dshot_encoder_1 = NULL;
    rmt_encoder_handle_t dshot_encoder_2 = NULL;

    dshot_esc_encoder_config_t encoder_config_1 = { // update new pins
        .resolution = DSHOT_ESC_RESOLUTION_HZ,
        .baud_rate = 300000, // DSHOT300 protocol
        .post_delay_us = 50, // extra delay between each frame   
    };
    dshot_esc_encoder_config_t encoder_config_2 = {
        .resolution = DSHOT_ESC_RESOLUTION_HZ,
        .baud_rate = 300000, // DSHOT300 protocol
        .post_delay_us = 50, // extra delay between each frame
    };

    ESP_ERROR_CHECK(rmt_new_dshot_esc_encoder(&encoder_config_1 , &dshot_encoder_1));
    ESP_ERROR_CHECK(rmt_new_dshot_esc_encoder(&encoder_config_2 , &dshot_encoder_2));

    rmt_transmit_config_t tx_config_1 = { // update new pins
        .loop_count = -1, // infinite loop
    };
    rmt_transmit_config_t tx_config_2 = {
        .loop_count = -1, // infinite loop
    };

    dshot_esc_throttle_t throttle = {
        .throttle = 0,
        .telemetry_req = false, // telemetry is not supported in this example
    };

    ESP_LOGI(TAG, "Start ESC by sending zero throttle for a while...");

    ESP_ERROR_CHECK(rmt_transmit(esc_chan_1, dshot_encoder_1, &throttle, sizeof(throttle), &tx_config_1));// applic
    ESP_ERROR_CHECK(rmt_transmit(esc_chan_2, dshot_encoder_2, &throttle, sizeof(throttle), &tx_config_2));

    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "Increase throttle, no telemetry");

    // while(1)
    // {
    //     throttle.throttle = 200;
    //     ESP_ERROR_CHECK(rmt_transmit(esc_chan_1, dshot_encoder_1, &throttle, sizeof(throttle), &tx_config_1)); //applic
    //     ESP_ERROR_CHECK(rmt_transmit(esc_chan_2, dshot_encoder_2, &throttle, sizeof(throttle), &tx_config_2));

    //     ESP_ERROR_CHECK(rmt_disable(esc_chan_1));
    //     ESP_ERROR_CHECK(rmt_disable(esc_chan_2));
    //     ESP_ERROR_CHECK(rmt_enable(esc_chan_1));
    //     ESP_ERROR_CHECK(rmt_enable(esc_chan_2));

    //     vTaskDelay(pdMS_TO_TICKS(125));
    // }   
    
    test = 50;
    while (test != MIN_THROTTLE) 
    {
        delay = 0;
        disp = test;
        // ESP_LOGI(TAG, "Throttle: %d", disp);

        while(delay < 20) // delay = total delay / 5
        {
            throttle.throttle = test;
            disp = test;

            ESP_ERROR_CHECK(rmt_transmit(esc_chan_1, dshot_encoder_1, &throttle, sizeof(throttle), &tx_config_1)); //applic
            ESP_ERROR_CHECK(rmt_transmit(esc_chan_2, dshot_encoder_2, &throttle, sizeof(throttle), &tx_config_2));

            // the previous loop transfer is till undergoing, we need to stop it and restart,
            // so that the new throttle can be updated on the output

            ESP_ERROR_CHECK(rmt_disable(esc_chan_1));
            ESP_ERROR_CHECK(rmt_disable(esc_chan_2));
            ESP_ERROR_CHECK(rmt_enable(esc_chan_1));
            ESP_ERROR_CHECK(rmt_enable(esc_chan_2));
            
            vTaskDelay(pdMS_TO_TICKS(10));
            delay += 1;
        }
        test += 2; // step
    }

    thro = MIN_THROTTLE;
    while (thro <= MAX_THROTTLE)
    {
        delay = 0;
        disp = thro;
        // ESP_LOGI(TAG, "Throttle: %d", disp);

        while (delay < 40) // delay = total delay / 10
        {
            throttle.throttle = thro;
            disp = thro;

            ESP_ERROR_CHECK(rmt_transmit(esc_chan_1, dshot_encoder_1, &throttle, sizeof(throttle), &tx_config_1)); //applic
            ESP_ERROR_CHECK(rmt_transmit(esc_chan_2, dshot_encoder_2, &throttle, sizeof(throttle), &tx_config_2));

            // the previous loop transfer is till undergoing, we need to stop it and restart,
            // so that the new throttle can be updated on the output

            ESP_ERROR_CHECK(rmt_disable(esc_chan_1));
            ESP_ERROR_CHECK(rmt_disable(esc_chan_2));
            ESP_ERROR_CHECK(rmt_enable(esc_chan_1));
            ESP_ERROR_CHECK(rmt_enable(esc_chan_2));

            vTaskDelay(pdMS_TO_TICKS(10));
            delay += 1;
        }
        thro += 2; // step   
    }

    while(1)
    {
        throttle.throttle = MAX_THROTTLE;
        disp = MAX_THROTTLE;

        ESP_ERROR_CHECK(rmt_transmit(esc_chan_1, dshot_encoder_1, &throttle, sizeof(throttle), &tx_config_1)); //applic
        ESP_ERROR_CHECK(rmt_transmit(esc_chan_2, dshot_encoder_2, &throttle, sizeof(throttle), &tx_config_2));

        // the previous loop transfer is till undergoing, we need to stop it and restart,
        // so that the new throttle can be updated on the output

        ESP_ERROR_CHECK(rmt_disable(esc_chan_1));
        ESP_ERROR_CHECK(rmt_disable(esc_chan_2));
        ESP_ERROR_CHECK(rmt_enable(esc_chan_1));
        ESP_ERROR_CHECK(rmt_enable(esc_chan_2));

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void mpu_task(void *arg)
{
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create(); // Create I2C command link
    // Other MPU task code...
    // Euler angle buffer includes two floats - Roll angle and Pitch angle
    //float euler_angle[2], mpu_offset[2] = {0.0f, 0.0f};
     while (1)
    {
        float euler_angle[2], mpu_offset[2] = {0.0f, 0.0f};

            /*The MPU6050 comes up in sleep mode upon power-up
             * enable_mpu6050 disables the SLEEP_MODE of the MPU by accessing the POWER_MANAGEMENT_1 register
             * and checks if the MPU is initialised correctly
             */

            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // THIS SECTION IS TO GET READINGS FROM POTENTIOMETER AND APPLY CORRESPONDING THROTTLE
        if (enable_mpu6050() == ESP_OK)
        {
                /* Euler angles are obtained by reading values from the accelerometer and gyroscope
                 * Calculating change in roll, pitch from raw accelerometer and gyroscope readings seperately
                 * And then performing sensor fusion via complementary filter to obtain a stable reading
                 * The changes in angles from both measurements are weighted and added to the complementary angle
                 */
                                
            // printf("Throttle: %d | Roll: %0.2f | Pitch: %0.2f\n ", thro, euler_angle[0], euler_angle[1]);
            read_mpu6050(euler_angle, mpu_offset);
            // // Logging information of roll and pitch angles
            ESP_LOGI(TAG, "Throttle: %d | Pitch: %0.2f", disp, euler_angle[1]);

            if(count == 100)
            {
                i2c_cmd_link_delete(cmd_handle);  // Correctly calling the function  
                count = 0;
            }
            count ++ ;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void)
{
    // plot_graph_queue = xQueueCreate(10, sizeof(&pg_data));

    xTaskCreatePinnedToCore(&mpu_task, "mpu_task", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(&throttle_task, "throttle_task", 4096, NULL, 2, NULL, 1);
}

