/**
 * ESP32 Hibernating Temperature Sensor
 * https://github.com/tombayo/esp32-hibernating-tempsensor
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <inttypes.h>

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

#include "owb.h"
#include "owb_rmt.h"
#include "ds18b20.h"

static const char *WIFITAG = "WIFI_STATION";
#define ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY
static EventGroupHandle_t s_wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;
static int s_retry_num = 0;

static const char *HTTPTAG = "HTTP_CLIENT";
#define MAX_HTTP_RECV_BUFFER 512
#define HTTP_POST_URL CONFIG_HTTP_URL

#define HOSTNAME CONFIG_HOSTNAME
#define SERVERPW CONFIG_SERVERPW

static const char *SLEEPTAG = "SLEEP_WAKEUP";
#define WAKEUP_TIMEOUT  CONFIG_WAKEUP_TIMEOUT
static RTC_DATA_ATTR struct timeval sleep_enter_time;

static const char *TEMPTAG = "1W_TEMPSENSOR";
             char TEMPDATA[] = "";
#define GPIO_DS18B20_0      CONFIG_ONE_WIRE_GPIO
#define MAX_DEVICES         8
#define DS18B20_RESOLUTION  DS18B20_RESOLUTION_12_BIT
#define SAMPLE_PERIOD       1000 // ms

char* LastcharDel(char* name) {
  int i = 0;
  while(name[i] != '\0') {
    i++;
  }
  name[i-1] = '\0';
  return name;
}

static void read_temperature_sensor() {

  // Create a 1-Wire bus, using the RMT timeslot driver
  OneWireBus *owb;
  owb_rmt_driver_info rmt_driver_info;
  owb = owb_rmt_initialize(&rmt_driver_info, GPIO_DS18B20_0, RMT_CHANNEL_1, RMT_CHANNEL_0);
  owb_use_crc(owb, true);  // enable CRC check for ROM code

  // Find all connected devices
  ESP_LOGI(TEMPTAG,"Find devices:\n");
  OneWireBus_ROMCode device_rom_codes[MAX_DEVICES] = {0};
  int num_devices = 0;
  OneWireBus_SearchState search_state = {0};
  bool found = false;
  owb_search_first(owb, &search_state, &found);
  while (found) {
    char rom_code_s[17];
    owb_string_from_rom_code(search_state.rom_code, rom_code_s, sizeof(rom_code_s));
    printf("  %d : %s\n", num_devices, rom_code_s);
    device_rom_codes[num_devices] = search_state.rom_code;
    ++num_devices;
    owb_search_next(owb, &search_state, &found);
  }
  ESP_LOGI(TEMPTAG, "Found %d device%s\n", num_devices, num_devices == 1 ? "" : "s");

  // Create DS18B20 devices on the 1-Wire bus
  DS18B20_Info * devices[MAX_DEVICES] = {0};
  for (int i = 0; i < num_devices; ++i) {
    DS18B20_Info * ds18b20_info = ds18b20_malloc();  // heap allocation
    devices[i] = ds18b20_info;

    if (num_devices == 1) {
      ESP_LOGI(TEMPTAG, "Single device optimisations enabled\n");
      ds18b20_init_solo(ds18b20_info, owb);          // only one device on bus
    }
    else {
      ds18b20_init(ds18b20_info, owb, device_rom_codes[i]); // associate with bus and device
    }
    ds18b20_use_crc(ds18b20_info, true);           // enable CRC check for temperature readings
    ds18b20_set_resolution(ds18b20_info, DS18B20_RESOLUTION);
  }

  // Read temperatures more efficiently by starting conversions on all devices at the same time
  int errors_count[MAX_DEVICES] = {0};
  int sample_count = 0;
  if (num_devices > 0) {
    ds18b20_convert_all(owb);

    // In this application all devices use the same resolution,
    // so use the first device to determine the delay
    ds18b20_wait_for_conversion(devices[0]);

    // Read the results immediately after conversion otherwise it may fail
    // (using printf before reading may take too long)
    float readings[MAX_DEVICES] = { 0 };
    DS18B20_ERROR errors[MAX_DEVICES] = { 0 };

    for (int i = 0; i < num_devices; ++i) {
      errors[i] = ds18b20_read_temp(devices[i], &readings[i]);
    }

    // Print results in a separate loop, after all have been read
    ESP_LOGI(TEMPTAG, "\nTemperature readings (degrees C): sample %d\n", ++sample_count);
    for (int i = 0; i < num_devices; ++i) {
      if (errors[i] != DS18B20_OK) {
        ++errors_count[i];
      } else {
        char buffer[32];
        snprintf(buffer, sizeof(buffer), "\"%.1f\",", readings[i]);
        strcat(TEMPDATA, buffer);
      }

      ESP_LOGI(TEMPTAG, "  %d: %.1f    %d errors\n", i, readings[i], errors_count[i]);
    }

  }

  // clean up dynamically allocated data
  for (int i = 0; i < num_devices; ++i) {
    ds18b20_free(&devices[i]);
  }

  owb_uninitialize(owb);
}

/**
 * Sends the ESP32 off to sleep.
 * */
static void hibernate() {
  const int wakeup_time_sec = WAKEUP_TIMEOUT;
  ESP_LOGI(SLEEPTAG, "Enabling timer wakeup, %ds.", wakeup_time_sec);
  esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000);

  ESP_LOGI(SLEEPTAG, "Entering deep sleep.");
  gettimeofday(&sleep_enter_time, NULL);

  esp_deep_sleep_start();
}


/**
 * The HTTP event-handler
 * */
static esp_err_t _http_event_handler(esp_http_client_event_t *evt) {
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

/**
 * Performs the HTTP-request
 * */
static void http_post_data(char *post_data) {
  esp_http_client_config_t config = {
    .url            = HTTP_POST_URL,
    .method         = HTTP_METHOD_POST,
    .event_handler  = _http_event_handler,
  };
  esp_http_client_handle_t client = esp_http_client_init(&config);

  esp_http_client_set_post_field(client, post_data, strlen(post_data));
  esp_http_client_set_header(client, "Content-Type", "application/json");

  esp_err_t err = esp_http_client_perform(client);
  if (err == ESP_OK) {
    ESP_LOGI(HTTPTAG, "HTTP POST Status = %d, content_length = %d",
              esp_http_client_get_status_code(client),
              esp_http_client_get_content_length(client));
  } else {
    ESP_LOGE(HTTPTAG, "HTTP POST request failed: %s", esp_err_to_name(err));
  }

  esp_http_client_cleanup(client);
}

/**
 * Called when Wifi and IP-address is OK.
 * 
 * This is the function to put online tasks into.
 * */
static void connected_task(void *pvParameters) {

  read_temperature_sensor(); // Read temperature data

  ESP_LOGI("DEBUG", "Current value from tempsensor: %s", TEMPDATA);

  // Build our data-string:
  char buffer[1024];
  snprintf(buffer, sizeof(buffer),
    "{\"hostname\":\"%s\",\"password\":\"%s\",\"temperature\":[%s]}",
    HOSTNAME,
    SERVERPW,
    LastcharDel(TEMPDATA)
  );

  ESP_LOGI(HTTPTAG, "Posting data: %s", buffer);
  http_post_data(buffer);

  hibernate();

  vTaskDelete(NULL);
}

/**
 * Handles the WiFi and IP events
 * */
static void _wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
    if (s_retry_num < ESP_MAXIMUM_RETRY) {
      esp_wifi_connect();
      xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
      s_retry_num++;
      ESP_LOGI(WIFITAG, "retry to connect to the AP");
    }
    ESP_LOGI(WIFITAG,"connect to the AP fail");
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
    ESP_LOGI(WIFITAG, "got ip:%s", ip4addr_ntoa(&event->ip_info.ip));
    s_retry_num = 0;
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);

    xTaskCreate(&connected_task, "connected_task", 8192, NULL, 5, NULL); // Runs once wifi is up, and ip is OK.
  }
}

/**
 * Inits a Wifi-station and connects to the network defined in menuconfig
 * */
static void wifi_init_sta() {
  ESP_LOGI(WIFITAG, "ESP_WIFI_MODE_STA");
  s_wifi_event_group = xEventGroupCreate();

  tcpip_adapter_init();

  ESP_ERROR_CHECK(esp_event_loop_create_default());

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &_wifi_event_handler, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &_wifi_event_handler, NULL));

  wifi_config_t wifi_config = {
    .sta = {
      .ssid = ESP_WIFI_SSID,
      .password = ESP_WIFI_PASS
    },
  };
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
  ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
  ESP_ERROR_CHECK(esp_wifi_start() );

  ESP_LOGI(WIFITAG, "wifi_init_sta finished.");
  ESP_LOGI(WIFITAG, "connect to ap SSID:%s password:%s", ESP_WIFI_SSID, ESP_WIFI_PASS);
}

/**
 * Reports the reason for waking up aswell as how long sleep lasted
 * */
static void report_wakeup_status() {
  struct timeval now;
  gettimeofday(&now, NULL);
  int sleep_time_ms = (now.tv_sec - sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - sleep_enter_time.tv_usec) / 1000;

  switch (esp_sleep_get_wakeup_cause()) {
    case ESP_SLEEP_WAKEUP_TIMER: {
      ESP_LOGI(SLEEPTAG,"Wake up from timer. Time spent in deep sleep: %dms\n", sleep_time_ms);
      break;
    }
    case ESP_SLEEP_WAKEUP_UNDEFINED:
    default:
      ESP_LOGI(SLEEPTAG,"Not a deep sleep reset\n");
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

  wifi_init_sta(); // Runs connected_task() once everything is up an running

  ESP_LOGI("DEBUG", "End of Script?");
}