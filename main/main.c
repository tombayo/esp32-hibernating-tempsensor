/**
 * ESP32 Hibernating Temperature Sensor
 * https://github.com/tombayo/esp32-hibernating-tempsensor
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_sleep.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp32/ulp.h"
#include "driver/adc.h"
#include "driver/rtc_io.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc.h"
#include "nvs_flash.h"
#include "tcpip_adapter.h"
#include "esp_http_client.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#define ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY
/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;
/* The event group allows multiple bits for each event, but we only care about one event 
 * - are we connected to the AP with an IP? */
const int WIFI_CONNECTED_BIT = BIT0;
static const char *TAG = "wifi station";
static int s_retry_num = 0;


#define MAX_HTTP_RECV_BUFFER 512
static const char *HTTPTAG = "HTTP_CLIENT";

#define WAKEUP_TIMEOUT  CONFIG_WAKEUP_TIMEOUT
static RTC_DATA_ATTR struct timeval sleep_enter_time;

/**
 * Sends the ESP32 off to sleep.
 * 
 */
void hibernate() {
  const int wakeup_time_sec = WAKEUP_TIMEOUT;
  printf("Enabling timer wakeup, %ds\n", wakeup_time_sec);
  esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000);

  printf("Entering deep sleep\n");
  gettimeofday(&sleep_enter_time, NULL);

  esp_deep_sleep_start();
}

esp_err_t _http_event_handler(esp_http_client_event_t *evt) {
  switch(evt->event_id) {
    case HTTP_EVENT_ERROR:
      ESP_LOGD(HTTPTAG, "HTTP_EVENT_ERROR");
      break;
    case HTTP_EVENT_ON_CONNECTED:
      ESP_LOGD(HTTPTAG, "HTTP_EVENT_ON_CONNECTED");
      break;
    case HTTP_EVENT_HEADER_SENT:
      ESP_LOGD(HTTPTAG, "HTTP_EVENT_HEADER_SENT");
      break;
    case HTTP_EVENT_ON_HEADER:
      ESP_LOGD(HTTPTAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
      break;
    case HTTP_EVENT_ON_DATA:
      ESP_LOGD(HTTPTAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
      if (!esp_http_client_is_chunked_response(evt->client)) {
        // Write out data
        // printf("%.*s", evt->data_len, (char*)evt->data);
      }

      break;
    case HTTP_EVENT_ON_FINISH:
      ESP_LOGD(HTTPTAG, "HTTP_EVENT_ON_FINISH");
      break;
    case HTTP_EVENT_DISCONNECTED:
      ESP_LOGD(HTTPTAG, "HTTP_EVENT_DISCONNECTED");
      break;
  }
  return ESP_OK;
}

static void http_post_data() {
  esp_http_client_config_t config = {
    .url = "http://httpbin.org/get",
    .event_handler = _http_event_handler,
  };
  esp_http_client_handle_t client = esp_http_client_init(&config);

  // GET
  esp_err_t err = esp_http_client_perform(client);
  if (err == ESP_OK) {
    ESP_LOGI(HTTPTAG, "HTTP GET Status = %d, content_length = %d",
              esp_http_client_get_status_code(client),
              esp_http_client_get_content_length(client));
  } else {
    ESP_LOGE(HTTPTAG, "HTTP GET request failed: %s", esp_err_to_name(err));
  }

  // POST
  const char *post_data = "field1=value1&field2=value2";
  esp_http_client_set_url(client, "http://httpbin.org/post");
  esp_http_client_set_method(client, HTTP_METHOD_POST);
  esp_http_client_set_post_field(client, post_data, strlen(post_data));
  err = esp_http_client_perform(client);
  if (err == ESP_OK) {
    ESP_LOGI(HTTPTAG, "HTTP POST Status = %d, content_length = %d",
              esp_http_client_get_status_code(client),
              esp_http_client_get_content_length(client));
  } else {
    ESP_LOGE(HTTPTAG, "HTTP POST request failed: %s", esp_err_to_name(err));
  }

  esp_http_client_cleanup(client);
}

static void http_task(void *pvParameters) {
    http_post_data();

    ESP_LOGI(HTTPTAG, "Ran HTTP request");
    hibernate();
    vTaskDelete(NULL);
}

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
    if (s_retry_num < ESP_MAXIMUM_RETRY) {
      esp_wifi_connect();
      xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
      s_retry_num++;
      ESP_LOGI(TAG, "retry to connect to the AP");
    }
    ESP_LOGI(TAG,"connect to the AP fail");
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
    ESP_LOGI(TAG, "got ip:%s", ip4addr_ntoa(&event->ip_info.ip));
    s_retry_num = 0;
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);

    xTaskCreate(&http_task, "http_task", 8192, NULL, 5, NULL); // Run HTTP stuff
  }
}

void wifi_init_sta() {
    s_wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    wifi_config_t wifi_config = {
      .sta = {
        .ssid = ESP_WIFI_SSID,
        .password = ESP_WIFI_PASS
      },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");
    ESP_LOGI(TAG, "connect to ap SSID:%s password:%s", ESP_WIFI_SSID, ESP_WIFI_PASS);
}

/**
 * Reports the reason for waking up aswell as how long sleep lasted
 * 
 */
void report_wakeup_status() {
  struct timeval now;
  gettimeofday(&now, NULL);
  int sleep_time_ms = (now.tv_sec - sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - sleep_enter_time.tv_usec) / 1000;

  switch (esp_sleep_get_wakeup_cause()) {
    case ESP_SLEEP_WAKEUP_TIMER: {
      printf("Wake up from timer. Time spent in deep sleep: %dms\n", sleep_time_ms);
      break;
    }
    case ESP_SLEEP_WAKEUP_UNDEFINED:
    default:
      printf("Not a deep sleep reset\n");
  }
}

void app_main() {
  report_wakeup_status();

  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
  wifi_init_sta();
}