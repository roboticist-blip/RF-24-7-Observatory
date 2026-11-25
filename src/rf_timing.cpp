#include "rf_timing.h"
#include <Arduino.h>
#include <string.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "config.h"

static esp_timer_handle_t s_timer = nullptr;
static SemaphoreHandle_t s_sem = nullptr;

static void wakeCb(void*) { xSemaphoreGive(s_sem); }

void rfTimingInit() {
  s_sem = xSemaphoreCreateBinary();
  if (!s_sem) return;
  esp_timer_create_args_t a;
  memset(&a, 0, sizeof(a));
  a.callback = wakeCb;
  a.arg = nullptr;
  a.dispatch_method = ESP_TIMER_TASK;
  a.name = "rfwake";
  if (esp_timer_create(&a, &s_timer) != ESP_OK) s_timer = nullptr;
}

int64_t rfNowUs() { return esp_timer_get_time(); }

void rfSleepUntilUs(int64_t dl) {
  int64_t rem = dl - esp_timer_get_time();
  if (rem > 20000) {                                   // long wait: let the scheduler sleep us (1 ms ticks assumed)
    vTaskDelay(pdMS_TO_TICKS((uint32_t)((rem - 10000) / 1000)));
    rem = dl - esp_timer_get_time();
  }
  if (rem > (int64_t)CFG_RF_WAKE_LATENCY_US + 20 && s_timer) {
    esp_timer_stop(s_timer);                           // harmless if not running
    xSemaphoreTake(s_sem, 0);                          // clear a stale wake-up
    if (esp_timer_start_once(s_timer, (uint64_t)(rem - CFG_RF_WAKE_LATENCY_US)) == ESP_OK) {
      if (xSemaphoreTake(s_sem, pdMS_TO_TICKS((uint32_t)(rem / 1000) + 50)) != pdTRUE) esp_timer_stop(s_timer);
    }
  }
  if (!s_timer) {
    rem = dl - esp_timer_get_time();
    if (rem > 2000) vTaskDelay(pdMS_TO_TICKS((uint32_t)(rem / 1000) - 1));
  }
  while (esp_timer_get_time() < dl) { /* bounded spin: < wake latency */ }
}

void rfSleepUs(uint32_t us) { rfSleepUntilUs(esp_timer_get_time() + us); }
