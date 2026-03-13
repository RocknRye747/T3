#include "esp_log.h"

static const char *TAG = "reflex";

extern void reflex_kernel_start(void);

void app_main(void)
{
    ESP_LOGI(TAG, "Starting reflex kernel from firmware/reflex/reflex.S");
    reflex_kernel_start();
}
