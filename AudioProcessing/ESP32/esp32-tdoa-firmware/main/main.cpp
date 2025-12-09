/* =================================================================================================
 * ESP32-S3 - DUAL CHANNEL I2S BIT SCOPE
 * =================================================================================================
 * PURPOSE: Monitors two I2S data lines simultaneously to detect specific failure modes.
 * WIRING:
 * - BCLK: GPIO 4
 * - WS:   GPIO 5
 * - DIN1: GPIO 6 (UMA-8 J3.1)
 * - DIN2: GPIO 7 (UMA-8 J3.3)
 * =================================================================================================
 */
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"

static const char* TAG = "DUAL_SCOPE";

// --- PINS ---
#define I2S_WS_PIN        GPIO_NUM_5
#define I2S_SCK_PIN       GPIO_NUM_4
#define I2S_DIN_1_PIN     GPIO_NUM_6 
#define I2S_DIN_2_PIN     GPIO_NUM_7

const int SAMPLE_RATE = 48000;

// Handles for two separate I2S peripherals
i2s_chan_handle_t rx_handle_0 = NULL;
i2s_chan_handle_t rx_handle_1 = NULL;

int32_t buf_1[64];
int32_t buf_2[64];

void setup_dual_i2s() {
    ESP_LOGI(TAG, "Initializing Dual I2S Scope...");

    // --- CHANNEL 0 (Line 1) ---
    i2s_chan_config_t cfg0 = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_SLAVE);
    ESP_ERROR_CHECK(i2s_new_channel(&cfg0, NULL, &rx_handle_0));
    
    i2s_std_config_t std0 = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = (gpio_num_t)I2S_SCK_PIN,
            .ws = (gpio_num_t)I2S_WS_PIN,
            .dout = I2S_GPIO_UNUSED,
            .din = (gpio_num_t)I2S_DIN_1_PIN,
        },
    };
    std0.slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT;
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle_0, &std0));

    // --- CHANNEL 1 (Line 2) ---
    i2s_chan_config_t cfg1 = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_SLAVE);
    ESP_ERROR_CHECK(i2s_new_channel(&cfg1, NULL, &rx_handle_1));
    
    i2s_std_config_t std1 = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = (gpio_num_t)I2S_SCK_PIN,
            .ws = (gpio_num_t)I2S_WS_PIN,
            .dout = I2S_GPIO_UNUSED,
            .din = (gpio_num_t)I2S_DIN_2_PIN,
        },
    };
    std1.slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT;
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle_1, &std1));

    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle_0));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle_1));
}

void scope_task(void* pv) {
    size_t r1 = 0, r2 = 0;
    
    while(1) {
        // Read both lines
        i2s_channel_read(rx_handle_0, buf_1, sizeof(buf_1), &r1, pdMS_TO_TICKS(100));
        i2s_channel_read(rx_handle_1, buf_2, sizeof(buf_2), &r2, pdMS_TO_TICKS(100));

        if (r1 > 0 && r2 > 0) {
            int32_t val1 = buf_1[0];
            int32_t val2 = buf_2[0];

            // Helper lambda to get status string
            auto get_status = [](int32_t val) -> const char* {
                if (val == 0) return "MUTE (00)";
                if (val == -1) return "FLOAT (FF)";
                return "DATA (OK)";
            };

            const char* s1 = get_status(val1);
            const char* s2 = get_status(val2);

            // Print side-by-side
            // If EITHER has data, we print bold log. If both silent, we just print dots.
            if (val1 != 0 && val1 != -1) {
                ESP_LOGW(TAG, "L1: %s [%08lx] | L2: %s [%08lx]", s1, (long)val1, s2, (long)val2);
            } 
            else if (val2 != 0 && val2 != -1) {
                ESP_LOGW(TAG, "L1: %s [%08lx] | L2: %s [%08lx]", s1, (long)val1, s2, (long)val2);
            }
            else {
                // If inputs are different (e.g. one MUTE one FLOAT), print update
                if (val1 != val2) {
                     ESP_LOGI(TAG, "L1: %s | L2: %s", s1, s2);
                } else {
                     // Both same (likely MUTE/MUTE)
                     printf("."); fflush(stdout);
                }
            }

        } else {
            ESP_LOGE(TAG, "TIMEOUT - No Clocks");
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

extern "C" void app_main() {
    setup_dual_i2s();
    xTaskCreatePinnedToCore(scope_task, "Scope", 4096, NULL, 5, NULL, 1);
}