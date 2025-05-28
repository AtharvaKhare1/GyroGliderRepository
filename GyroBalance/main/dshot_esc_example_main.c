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
#include "tuning_websocket_server.h"



#if CONFIG_IDF_TARGET_ESP32H2
#define DSHOT_ESC_RESOLUTION_HZ 32000000 // 32MHz resolution, DSHot protocol needs a relative high resolution
#else
#define DSHOT_ESC_RESOLUTION_HZ 40000000 // 40MHz resolution, DSHot protocol needs a relative high resolution
#endif
#define DSHOT_ESC_GPIO_NUM_1      15
#define DSHOT_ESC_GPIO_NUM_2      2
#define MIN_THROTTLE 560
#define MAX_THROTTLE 1027

#define MAX_PITCH_CORRECTION (90.0f)
#define MAX_PITCH_AREA (850.0f)
#define MAX_PITCH_RATE (850.0f)

static const char *TAG = "example";
uint16_t disp;
uint16_t thro;
uint16_t test = MIN_THROTTLE;
uint64_t start_time;
uint64_t current_time;
uint16_t delay;

float P_term = 0.0;
float I_term = 0.0;
float D_term = 0.0;
float setpt = 30;
int baseValue = 100;
float integralError = 0.0;
float prevError = 0.0;
float currentAngle = 0.0;
float euler_angle[2], mpu_offset[2] = {0.0f, 0.0f};
float throtang = 0.0;

plot_graph_data_t pg_data;
QueueHandle_t plot_graph_queue = NULL;

int pid_calc(void *arg)
{
    setpt = read_pid_const().setpoint;
    float error = setpt - currentAngle;
    integralError += error;
    P_term = (read_pid_const().kp * error);
    D_term = (read_pid_const().kd * (error - prevError));
    I_term = (read_pid_const().ki * (integralError));
    throtang = P_term + D_term + I_term;
    throtang = throtang * (M_PI / 180); // to radians
    throtang = cos(throtang);
    throtang = throtang * 2.504; // edit the constant term here
	
    int throttle = MIN_THROTTLE + (int)throtang;

    prevError = error;
    throttle = (throttle < MIN_THROTTLE) ? MIN_THROTTLE : (throttle > MAX_THROTTLE ? MAX_THROTTLE : throttle);

    return throttle;

    pg_data.p_term = P_term;
	pg_data.d_term = D_term;
	pg_data.i_term = I_term;
	pg_data.pitch_corr = integralError;
	pg_data.pitch_err = error;

    plot_graph_data_t *pg_data_handle = &pg_data;
    xQueueSend(plot_graph_queue, (void *)&pg_data_handle, (TickType_t)0);
}

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

    vTaskDelay(pdMS_TO_TICKS(1000));

    // ESP_LOGI(TAG, "Increase throttle, no telemetry");

    while (test != thro)
    {
        
        disp = test;
        delay = 0;
        while (delay < 1) // delay < (total delay / 5)
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

            vTaskDelay(pdMS_TO_TICKS(5));
            delay += 1;
        }
        test += (thro > test) ? 1 : -1;

    }

    while(test == thro)
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

            vTaskDelay(pdMS_TO_TICKS(5));
    }

}

void mpu_task(void *arg)
{
    // Euler angle buffer includes two floats - Roll angle and Pitch angle
    //float euler_angle[2], mpu_offset[2] = {0.0f, 0.0f};
     while (1)
    {
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
            read_mpu6050(euler_angle, mpu_offset);
            currentAngle = euler_angle[1];
            thro = pid_calc(NULL);

            // printf("Throttle: %d | Roll: %0.2f | Pitch: %0.2f\n ", thro, euler_angle[0], euler_angle[1]);

            ESP_LOGI(TAG, "Throttle: %d | Pitch: %0.2f", disp, euler_angle[1]);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

void app_main(void)
{
    plot_graph_queue = xQueueCreate(10, sizeof(&pg_data));

    xTaskCreatePinnedToCore(&mpu_task, "mpu_task", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(&throttle_task, "throttle_task", 4096, NULL, 1, NULL, 1);
}

