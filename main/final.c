#include "driver/adc.h"
#include "driver/gpio.h"
#include "esp_adc_cal.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_nimble_hci.h"
#include "esp_rom_gpio.h"
#include "esp_wifi.h"
#include "esp_wifi_types_generic.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "mpu6050.h"
#include "mqtt_client.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "os/os_mbuf.h"
#include "sdkconfig.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define TMP36_ADC_CHANNEL ADC1_CHANNEL_6
#define DEFAULT_VREF 3300
#define ADC_RESOLUTION 4095
#define TMP36_VOFFSET 500
#define TMP36_SCALING_FACTOR 10
#define ADC_ATTENUATION ADC_ATTEN_DB_12

char *TAG = "BlackBox";

#define NVS_NAMESPACE "storage"

#define NVS_KEY_SSID "SSID"
#define NVS_KEY_PASS "PASSWD"
#define NVS_KEY_MQTT "MQTTIP"
#define NVS_KEY_MQTT_PASSWD "MQTTPASSWD"

#define NVS_D_VALUE_SSID "BlackBox"
#define NVS_D_VALUE_PASSWD "1234567890"
#define NVS_D_VALUE_MQTT_IP "mqtt://192.168.145.179"
#define NVS_D_VALUE_MQTT_PASSWD "0000"

#define TOPIC_SIZE 55
char acc_topic[TOPIC_SIZE];
char gyro_topic[TOPIC_SIZE];
char temp_topic[TOPIC_SIZE];
char conf_topic[TOPIC_SIZE];

#define NVS_SIZE 25
char ssid[NVS_SIZE], password[NVS_SIZE], mqtt_ip[NVS_SIZE],
    mqtt_passwd[NVS_SIZE];

uint8_t mac[6];
char mac_str[18];

#define BLINK_GPIO GPIO_NUM_2
#define BUTTON_GPIO GPIO_NUM_0

volatile bool isConnectedWiFi = false;
volatile bool isConnectedMQTT = false;

float accel_x = 0, accel_y = 0, accel_z = 0;
float gyro_x = 0, gyro_y = 0, gyro_z = 0;
float temp = 0;

volatile int TIME_TO_PUBLISH = 1;

esp_mqtt_client_handle_t client = NULL;

esp_mqtt_client_config_t mqtt_cfg = {
    .credentials.username = "user",
    .credentials.authentication.password = "password",
    .session.keepalive = 120,
    .network.reconnect_timeout_ms = 10000,
    .network.timeout_ms = 10000,
    .task.priority = 5,
    .task.stack_size = 4096,
    .buffer.size = 1024,
};

MPU6050_State mpu_state = {.address =
                               MPU6050_ADDR, // Using default address (0x68)
                           .accel_scale = MPU6050_ACCEL_SCALE_2G,
                           .gyro_scale = MPU6050_GYRO_SCALE_250DPS};

void mqtt_publish_data(esp_mqtt_client_handle_t client) {
  if (isConnectedWiFi && isConnectedMQTT) {
    double data = 1.0;
    char payload[40];
    snprintf(payload, sizeof(payload), "a-data: %.2f %.2f %.2f", accel_x,
             accel_y, accel_z);
    int msg_id = esp_mqtt_client_publish(client, acc_topic, payload, 0, 1, 0);
    ESP_LOGI("MQTT", "Message sent, ID: %d", msg_id);

    data = 2.0;
    snprintf(payload, sizeof(payload), "g-data: %.2f %.2f %.2f", gyro_x, gyro_y,
             gyro_z);
    msg_id = esp_mqtt_client_publish(client, gyro_topic, payload, 0, 1, 0);
    ESP_LOGI("MQTT", "Message sent, ID: %d", msg_id);

    data = 3.0;
    snprintf(payload, sizeof(payload), "t-data: %.2f", temp);
    msg_id = esp_mqtt_client_publish(client, temp_topic, payload, 0, 1, 0);
    ESP_LOGI("MQTT", "Message sent, ID: %d", msg_id);

    // ESP_LOGI(TAG, "High water mark: %d bytes",
    // uxTaskGetStackHighWaterMark(NULL));
  }
}

static void tmp36gt92_task(void *param) {
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(TMP36_ADC_CHANNEL, ADC_ATTENUATION);
  int raw_adc_value;
  float voltage, temp_celsius;

  while (1) {
    raw_adc_value = adc1_get_raw(TMP36_ADC_CHANNEL);
    voltage = raw_adc_value * DEFAULT_VREF / ADC_RESOLUTION;
    temp_celsius = (voltage - TMP36_VOFFSET) / TMP36_SCALING_FACTOR;

    temp = temp_celsius;

    printf("ADC Value: %d, Voltage: %.2f mV, Temperature: %.2f °C\n",
           raw_adc_value, voltage, temp_celsius);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void monitor_task(void *pvParameters) {
  char buffer[512]; // Buffer to store task stats
  while (1) {
    // Get task information
    vTaskList(buffer);
    ESP_LOGI("Task Monitor", "\nTask Name\tState\tPrio\tStack\tTask Num\n%s",
             buffer);

    // Delay to prevent flooding the log
    vTaskDelay(pdMS_TO_TICKS(5000)); // 5-second delay
  }
}

static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event) {
  if (event == NULL) {
    ESP_LOGE("MQTT", "Received null event.");
    return ESP_FAIL;
  }
  switch (event->event_id) {
  case MQTT_EVENT_CONNECTED:
    ESP_LOGI("MQTT", "Connected to broker");
    esp_mqtt_client_subscribe(client, conf_topic, 1);
    break;
  case MQTT_EVENT_DISCONNECTED:
    ESP_LOGI("MQTT", "Disconnected from broker");
    isConnectedMQTT = false;
    while (1) {
      ESP_LOGI("MQTT", "Reconnecting to broker...");
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      int temp = esp_mqtt_client_reconnect(client);
      if (temp == ESP_OK) {
        isConnectedMQTT = true;
        break;
      }
    }
    break;
  case MQTT_EVENT_PUBLISHED:
    ESP_LOGI("MQTT", "Message Published. ID: %d", event->msg_id);
    break;
  case MQTT_EVENT_SUBSCRIBED:
    ESP_LOGI("MQTT", "Subscribed to topic, ID: %d", event->msg_id);
    break;
  case MQTT_EVENT_UNSUBSCRIBED:
    ESP_LOGI("MQTT", "Unsubscribed from topic, ID: %d", event->msg_id);
    break;
  case MQTT_EVENT_DATA:
    ESP_LOGI("MQTT", "Message received:");
    ESP_LOGI("MQTT", "Topic: %.*s", event->topic_len, event->topic);
    ESP_LOGI("MQTT", "Data: %.*s", event->data_len, event->data);

    char data[8];
    strncpy(data, event->data, event->data_len);
    data[event->data_len] = '\0';

    if (strcmp(data, "sleep") == 0) {
      ESP_LOGI("MQTT", "Sleeping...");
      mpu6050_set_sleep_mode(&mpu_state);
      ESP_LOGI(TAG, "MPU6050 in sleep mode");

    } else if (strcmp(data, "cycle") == 0) {
      ESP_LOGI("MQTT", "Cycling...");
      mpu6050_set_cycle_mode(&mpu_state);
      ESP_LOGI(TAG, "MPU6050 in cycle mode");
    } else if (strcmp(data, "calib") == 0) {
      ESP_LOGI("MQTT", "Calibrating...");
      // TODO Calibrate
      mpu6050_calibrate_bias(&mpu_state);
      ESP_LOGI(TAG, "MPU6050 calibrated");
    } else if (strcmp(data, "reboot") == 0) {
      esp_restart();
    } else if (strcmp(data, "t1") == 0) {
      ESP_LOGI("MQTT", "setting time in seconds to publish data");
      TIME_TO_PUBLISH = 1;
      ESP_LOGI("MQTT", "Time: %d", 1);
    } else if (strcmp(data, "t5") == 0) {
      ESP_LOGI("MQTT", "setting time in seconds to publish data");
      TIME_TO_PUBLISH = 5;
      ESP_LOGI("MQTT", "Time: %d", 5);
    } else if (strcmp(data, "a0") == 0) {

      // acc config 0
      // get current mpu6050 state
      MPU6050_StateEnum state = mpu_state.state;
      // go to normal
      mpu6050_go_to_normal_mode(&mpu_state);
      // set acc scale
      mpu6050_set_accel_scale(&mpu_state, MPU6050_ACCEL_SCALE_2G);
      // back to previous state
      if (state == MPU6050_STATE_SLEEP) {
        mpu6050_set_sleep_mode(&mpu_state);
      } else {
        mpu6050_set_cycle_mode(&mpu_state);
      }
    } else if (strcmp(data, "a1") == 0) {
      // acc config 0
      // get current mpu6050 state
      MPU6050_StateEnum state = mpu_state.state;
      // go to normal
      mpu6050_go_to_normal_mode(&mpu_state);
      // set acc scale
      mpu6050_set_accel_scale(&mpu_state, MPU6050_ACCEL_SCALE_4G);
      // back to previous state
      if (state == MPU6050_STATE_SLEEP) {
        mpu6050_set_sleep_mode(&mpu_state);
      } else {
        mpu6050_set_cycle_mode(&mpu_state);
      }
    } else {
      ESP_LOGI("MQTT", "Unknown command");
    }

    break;
  default:
    ESP_LOGI("MQTT", "Unhandled event: %d", event->event_id);
    break;
  }
  return ESP_OK;
}

static void mqtt_event_handler_adapter(void *handler_args,
                                       esp_event_base_t base, int32_t id,
                                       void *event_data) {
  esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
  mqtt_event_handler(event);
}

void mqtt_init() {

  // esp_read_mac(mac, ESP_MAC_WIFI_STA);
  esp_wifi_get_mac(ESP_IF_WIFI_STA, mac);
  snprintf(mac_str, sizeof(mac_str), "%02x%02x%02x%02x%02x%02x", mac[0], mac[1],
           mac[2], mac[3], mac[4], mac[5]);
  ESP_LOGI("MQTT", "MAC address: %s", mac_str);

  // snprintf(acc_topic, sizeof(acc_topic), "blackbox/%s/sensor1/acc", mac_str);
  snprintf(acc_topic, sizeof(acc_topic), "bb/%s/%s/acc", mac_str, mqtt_passwd);
  ESP_LOGI("MQTT", "Acceleration topic: %s", acc_topic);
  // snprintf(gyro_topic, sizeof(gyro_topic), "blackbox/%s/sensor1/gyro",
  // mac_str);
  snprintf(gyro_topic, sizeof(gyro_topic), "bb/%s/%s/gyro", mac_str,
           mqtt_passwd);
  ESP_LOGI("MQTT", "Gyro topic: %s", gyro_topic);
  // snprintf(temp_topic, sizeof(temp_topic), "blackbox/%s/sensor2/temp",
  // mac_str);
  snprintf(temp_topic, sizeof(temp_topic), "bb/%s/%s/temp", mac_str,
           mqtt_passwd);
  ESP_LOGI("MQTT", "Temperature topic: %s", temp_topic);
  // snprintf(conf_topic, sizeof(conf_topic), "blackbox/%s/config", mac_str);
  snprintf(conf_topic, sizeof(conf_topic), "bb/%s/%s/config", mac_str,
           mqtt_passwd);
  ESP_LOGI("MQTT", "Configuration topic: %s", conf_topic);

  ESP_LOGI("MQTT", "Initiating MQTT...");

  mqtt_cfg.broker.address.uri = mqtt_ip;
  client = esp_mqtt_client_init(&mqtt_cfg);

  esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID,
                                 mqtt_event_handler_adapter, NULL);

  ESP_LOGI("MQTT", "MQTT client initialized");
  isConnectedMQTT = true;

  ESP_ERROR_CHECK(esp_mqtt_client_start(client));
}

void publish_mqtt_data(void *param) {
  while (1) {
    if (isConnectedMQTT && isConnectedWiFi) {
      mqtt_publish_data(client);
    }
    mpu6050_debug_registers(&mpu_state);
    vTaskDelay(TIME_TO_PUBLISH * 1000 / portTICK_PERIOD_MS);
  }
}

volatile int blink = 0;
esp_err_t write_to_nvs(const char *key, const char *value);
esp_err_t read_from_nvs(const char *key, char *value, size_t max_len);
void init_nvs(void);
void debug_nvs(void *param);

uint8_t ble_addr_type;
void ble_app_advertise(void);

static int write_passwd(uint16_t conn_handle, uint16_t attr_handle,
                        struct ble_gatt_access_ctxt *ctxt, void *arg);

static int descriptor_passwd(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg);

static int write_ssid(uint16_t conn_handle, uint16_t attr_handle,
                      struct ble_gatt_access_ctxt *ctxt, void *arg);

static int descriptor_ssid(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg);

static int write_mqtt_ip(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg);

static int descriptor_mqtt_ip(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg);

static int write_mqtt_passwd(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg);

static int descriptor_mqtt_passwd(uint16_t conn_handle, uint16_t attr_handle,
                                  struct ble_gatt_access_ctxt *ctxt, void *arg);

static int read_mac(uint16_t conn_handle, uint16_t attr_handle,
                    struct ble_gatt_access_ctxt *ctxt, void *arg);

static int descriptor_mac(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg);

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = BLE_UUID16_DECLARE(0x0001),
     .characteristics =
         (struct ble_gatt_chr_def[]){
             {.uuid = BLE_UUID16_DECLARE(0x1001),
              .flags = BLE_GATT_CHR_F_WRITE,
              .access_cb = write_passwd,
              .descriptors =
                  (struct ble_gatt_dsc_def[]){
                      {.uuid = BLE_UUID16_DECLARE(0x2901),
                       .att_flags = BLE_ATT_F_READ,
                       .access_cb = descriptor_passwd},
                      {0}}},
             {.uuid = BLE_UUID16_DECLARE(0x1002),
              .flags = BLE_GATT_CHR_F_WRITE,
              .access_cb = write_ssid,
              .descriptors =
                  (struct ble_gatt_dsc_def[]){
                      {.uuid = BLE_UUID16_DECLARE(0x2901),
                       .att_flags = BLE_ATT_F_READ,
                       .access_cb = descriptor_ssid},
                      {0}}},
             {.uuid = BLE_UUID16_DECLARE(0x1003),
              .flags = BLE_GATT_CHR_F_WRITE,
              .access_cb = write_mqtt_ip,
              .descriptors =
                  (struct ble_gatt_dsc_def[]){
                      {
                          .uuid = BLE_UUID16_DECLARE(0x2901),
                          .att_flags = BLE_ATT_F_READ,
                          .access_cb = descriptor_mqtt_ip,
                      },
                      {0}}},
             {
                 .uuid = BLE_UUID16_DECLARE(0x1004),
                 .flags = BLE_GATT_CHR_F_WRITE,
                 .access_cb = write_mqtt_passwd,
                 .descriptors =
                     (struct ble_gatt_dsc_def[]){
                         {
                             .uuid = BLE_UUID16_DECLARE(0x2901),
                             .att_flags = BLE_ATT_F_READ,
                             .access_cb = descriptor_mqtt_passwd,
                         },
                         {0},
                     },
             },
             {
                 .uuid = BLE_UUID16_DECLARE(0x1005),
                 .flags = BLE_GATT_CHR_F_READ,
                 .access_cb = read_mac,
                 .descriptors =
                     (struct ble_gatt_dsc_def[]){
                         {
                             .uuid = BLE_UUID16_DECLARE(0x2901),
                             .att_flags = BLE_ATT_F_READ,
                             .access_cb = descriptor_mac,
                         },
                         {0},
                     },
             },
             {0}}},
    {0}};

#define MAX_CONNECTIONS 2
static uint16_t conn_handles[MAX_CONNECTIONS] = {0};

static int ble_gap_event(struct ble_gap_event *event, void *arg);
void ble_app_advertise(void);
void ble_app_on_sync(void);
void host_task(void *param);

static int retry_count = 0; // Counter to limit number of reconnection attempts
#define WIFI_MAX_RETRIES 5

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {
    isConnectedWiFi = false;
    ESP_LOGI(TAG, "Disconnected from Wi-Fi, reconnecting...");
    esp_wifi_connect(); // Try to reconnect immediately
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    isConnectedWiFi = true;
    ESP_LOGI(TAG, "Connected to Wi-Fi");
  }
}

wifi_config_t wifi_config = {
    .sta =
        {
            .ssid = "",
            .password = "",
        },
};

void init_wifi(void) {
  esp_netif_init();
  esp_event_loop_create_default();
  esp_netif_create_default_wifi_sta();
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);

  esp_event_handler_instance_t instance_any_id;
  esp_event_handler_instance_t instance_got_ip;
  esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                      &wifi_event_handler, NULL,
                                      &instance_any_id);
  esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                      &wifi_event_handler, NULL,
                                      &instance_got_ip);

  ESP_LOGI(TAG, "SSID: >%s<", ssid);
  ESP_LOGI(TAG, "Password: >%s<", password);

  strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
  strncpy((char *)wifi_config.sta.password, password,
          sizeof(wifi_config.sta.password));

  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
  esp_wifi_start();
}

void blink_task(void *pvParameter);
void button_task(void *pvParameter);

void ble_init(void);

void init_mpu(void) {
  mpu_state.accel_bias_x = 0;
  mpu_state.accel_bias_y = 0;
  mpu_state.accel_bias_z = 0;

  mpu_state.gyro_bias_x = 0;
  mpu_state.gyro_bias_y = 0;
  mpu_state.gyro_bias_z = 0;

  // Initialize the MPU6050
  ESP_LOGI(TAG, "Initializing MPU6050");
  esp_err_t ret = mpu6050_init(&mpu_state);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize MPU6050");
    return;
  }

  mpu6050_go_to_normal_mode(&mpu_state);

  bool gyro = false;
  bool accel = false;

  mpu6050_set_standby(&mpu_state, 0, gyro);  // Gyroscope active Z
  mpu6050_set_standby(&mpu_state, 1, gyro);  // Gyroscope active Y
  mpu6050_set_standby(&mpu_state, 2, gyro);  // Gyroscope active X
  mpu6050_set_standby(&mpu_state, 3, accel); // Accelerometer active Z
  mpu6050_set_standby(&mpu_state, 4, accel); // Accelerometer active Y
  mpu6050_set_standby(&mpu_state, 5, accel); // Accelerometer active X

  ESP_LOGI(TAG, "Setting accelerometer scale");
  mpu6050_set_accel_scale(&mpu_state, MPU6050_ACCEL_SCALE_2G);
  mpu6050_set_gyro_scale(&mpu_state, MPU6050_GYRO_SCALE_250DPS);
  mpu6050_set_low_power_wake(&mpu_state, MPU6050_WAKE_FREQ_1_25HZ);

  ESP_LOGI(TAG, "Calibrating sensor biases");
  mpu6050_calibrate_bias(&mpu_state);

  mpu6050_set_sleep_mode(&mpu_state);

  mpu6050_debug_registers(&mpu_state);

  ESP_LOGI(TAG, "MPU6050 initialized successfully");
  ESP_LOGI(TAG, "Chose mode");

  // chose mode
  mpu6050_set_cycle_mode(&mpu_state);
  // mpu6050_set_sleep_mode(&mpu_state);
  //mpu6050_go_to_normal_mode(&mpu_state);

  mpu6050_debug_registers(&mpu_state);
}

void mpu_read(void *param) {
  while (1) {
    // Read accelerometer data
    esp_err_t ret =
        mpu6050_read_accel(&mpu_state, &accel_x, &accel_y, &accel_z);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to read accelerometer data");
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      continue;
    }

    // Read gyroscope data
    ret = mpu6050_read_gyro(&mpu_state, &gyro_x, &gyro_y, &gyro_z);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to read gyroscope data");
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      continue;
    }

    // ESP_LOGI(
    //     TAG,
    //     "Accelerometer: X=%.2f g, Y=%.2f g, Z=%.2f g, sqrt(x^2+y^2+z^2) =
    //     %.2f", accel_x, accel_y, accel_z, sqrt(accel_x * accel_x + accel_y *
    //     accel_y + accel_z * accel_z));
    // ESP_LOGI(TAG, "Gyroscope: X=%.2f dps, Y=%.2f dps, Z=%.2f dps", gyro_x,
    //          gyro_y, gyro_z);

    // mpu6050_debug_registers(&mpu_state);
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

void app_main(void) {
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
      err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);

  init_nvs();
  // xTaskCreate(&debug_nvs, "debug_nvs", 2048, NULL, 5, NULL);
  init_wifi();

  ble_init();

  mqtt_init();

  init_mpu();

  xTaskCreate(&tmp36gt92_task, "temp_task", 2048, NULL, 5, NULL);

  xTaskCreate(&publish_mqtt_data, "publish_mqtt_data", 4096, NULL, 5, NULL);
  xTaskCreate(&mpu_read, "mpu_read", 2048, NULL, 5, NULL);

  xTaskCreate(&blink_task, "blink_task", 2048, NULL, 5, NULL);
  xTaskCreate(&button_task, "button_task", 2048, NULL, 5, NULL);

  // xTaskCreate(&monitor_task, "monitor_task", 2048, NULL, 1, NULL);  // Create
  // monitor task
}

esp_err_t write_to_nvs(const char *key, const char *value) {
  esp_err_t err;
  nvs_handle my_handle;

  err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle);
  if (err != ESP_OK) {
    ESP_LOGE("NVS", "Error (%s) opening NVS handle", esp_err_to_name(err));
    return err;
  }

  err = nvs_set_str(my_handle, key, value);
  if (err != ESP_OK) {
    ESP_LOGE("NVS", "Error (%s) writing to NVS", esp_err_to_name(err));
    nvs_close(my_handle);
    return err;
  }

  err = nvs_commit(my_handle);
  if (err != ESP_OK) {
    ESP_LOGE("NVS", "Error (%s) committing NVS changes", esp_err_to_name(err));
  }

  nvs_close(my_handle);
  return err;
}

esp_err_t read_from_nvs(const char *key, char *value, size_t max_len) {
  esp_err_t err;
  nvs_handle my_handle;

  err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &my_handle);
  if (err != ESP_OK) {
    ESP_LOGE("NVS", "Error (%s) opening NVS handle", esp_err_to_name(err));
    return err;
  }

  err = nvs_get_str(my_handle, key, value, &max_len);
  if (err != ESP_OK) {
    ESP_LOGE("NVS", "Error (%s) reading from NVS", esp_err_to_name(err));
  }

  nvs_close(my_handle);
  return err;
}

void init_nvs(void) {
  size_t len = sizeof(ssid);
  if (read_from_nvs(NVS_KEY_SSID, ssid, len) != ESP_OK || strlen(ssid) == 0) {
    ESP_LOGI("NVS", "SSID not found or empty, setting default value");
    write_to_nvs(NVS_KEY_SSID, NVS_D_VALUE_SSID);
    strncpy(ssid, NVS_D_VALUE_SSID, sizeof(ssid));
  }

  len = sizeof(password);
  if (read_from_nvs(NVS_KEY_PASS, password, len) != ESP_OK ||
      strlen(password) == 0) {
    ESP_LOGI("NVS", "Password not found or empty, setting default value");
    write_to_nvs(NVS_KEY_PASS, NVS_D_VALUE_PASSWD);
    strncpy(password, NVS_D_VALUE_PASSWD, sizeof(password));
  }

  len = sizeof(mqtt_ip);
  if (read_from_nvs(NVS_KEY_MQTT, mqtt_ip, len) != ESP_OK ||
      strlen(mqtt_ip) == 0) {
    ESP_LOGI("NVS", "MQTT IP not found or empty, setting default value");
    write_to_nvs(NVS_KEY_MQTT, NVS_D_VALUE_MQTT_IP);
    strncpy(mqtt_ip, NVS_D_VALUE_MQTT_IP, sizeof(mqtt_ip));
  }

  len = sizeof(mqtt_passwd);
  if (read_from_nvs(NVS_KEY_MQTT_PASSWD, mqtt_passwd, len) != ESP_OK ||
      strlen(mqtt_passwd) == 0) {
    ESP_LOGI("NVS", "MQTT Password not found or empty, setting default value");
    write_to_nvs(NVS_KEY_MQTT_PASSWD, NVS_D_VALUE_MQTT_PASSWD);
    strncpy(mqtt_passwd, NVS_D_VALUE_MQTT_PASSWD, sizeof(mqtt_passwd));
  }

  len = sizeof(ssid);
  if (read_from_nvs(NVS_KEY_SSID, ssid, len) != ESP_OK || strlen(ssid) == 0) {
    ESP_LOGI("NVS", "SSID not found or empty, setting default value");
    strncpy(ssid, NVS_D_VALUE_SSID, sizeof(ssid));
  }

  len = sizeof(password);
  if (read_from_nvs(NVS_KEY_PASS, password, len) != ESP_OK ||
      strlen(password) == 0) {
    ESP_LOGI("NVS", "Password not found or empty, setting default value");
    strncpy(password, NVS_D_VALUE_PASSWD, sizeof(password));
  }

  len = sizeof(mqtt_ip);
  if (read_from_nvs(NVS_KEY_MQTT, mqtt_ip, len) != ESP_OK ||
      strlen(mqtt_ip) == 0) {
    ESP_LOGI("NVS", "MQTT IP not found or empty, setting default value");
    strncpy(mqtt_ip, NVS_D_VALUE_MQTT_IP, sizeof(mqtt_ip));
  }

  len = sizeof(mqtt_passwd);
  if (read_from_nvs(NVS_KEY_MQTT_PASSWD, mqtt_passwd, len) != ESP_OK ||
      strlen(mqtt_passwd) == 0) {
    ESP_LOGI("NVS", "MQTT Password not found or empty, setting default value");
    strncpy(mqtt_passwd, NVS_D_VALUE_MQTT_PASSWD, sizeof(mqtt_passwd));
  }
}

void blink_task(void *pvParameter) {
  esp_rom_gpio_pad_select_gpio(BLINK_GPIO);
  gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

  while (1) {
    if (blink) {
      gpio_set_level(BLINK_GPIO, 1); // Turn LED ON
      vTaskDelay(500 / portTICK_PERIOD_MS);
      gpio_set_level(BLINK_GPIO, 0); // Turn LED OFF
      vTaskDelay(500 / portTICK_PERIOD_MS);
    } else {
      gpio_set_level(BLINK_GPIO, 0); // Ensure LED remains OFF
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
  }
}

void button_task(void *pvParameter) {
  // Configure Button pin
  esp_rom_gpio_pad_select_gpio(BUTTON_GPIO);
  gpio_set_direction(BUTTON_GPIO, GPIO_MODE_INPUT);
  gpio_set_pull_mode(BUTTON_GPIO, GPIO_PULLUP_ONLY); // Internal pull-up

  int last_button_state = 1; // Previous button state (initially not pressed)

  while (1) {
    int button_state = gpio_get_level(BUTTON_GPIO);

    if (button_state == 0 && last_button_state == 1) { // Button press detected
      blink = !blink;                                  // Toggle blinking state
      printf("Button pressed! Blinking: %s\n", blink ? "ON" : "OFF");
      vTaskDelay(200 / portTICK_PERIOD_MS); // Debounce delay
    }
    last_button_state = button_state;

    // UBaseType_t high_water_mark = uxTaskGetStackHighWaterMark(NULL);
    // printf("Remaining stack: %u bytes\n", high_water_mark *
    // sizeof(StackType_t));

    vTaskDelay(50 / portTICK_PERIOD_MS); // Polling delay
  }
}

void ble_init(void) {
  esp_nimble_hci_init();
  nimble_port_init();
  ble_svc_gap_device_name_set("BlackBox");
  ble_svc_gap_init();
  ble_svc_gatt_init();
  ble_gatts_count_cfg(gatt_svcs);
  ble_gatts_add_svcs(gatt_svcs);
  ble_hs_cfg.sync_cb = ble_app_on_sync;
  nimble_port_freertos_init(host_task);
}

void debug_nvs(void *param) {
  while (1) {
    ESP_LOGI("NVS", "SSID: %s, Password: %s, MQTT IP: %s", ssid, password,
             mqtt_ip);
    vTaskDelay(pdMS_TO_TICKS(5000)); // 5000 ms = 5 seconds
  }
}

static int write_passwd(uint16_t conn_handle, uint16_t attr_handle,
                        struct ble_gatt_access_ctxt *ctxt, void *arg) {
  char *data = (char *)ctxt->om->om_data;

  data[ctxt->om->om_len] = '\0';
  ESP_LOGI("BLE", "Data received: %s", data);
  ESP_LOGI("BLE", "Data length: %d", ctxt->om->om_len);

  if (blink == 0) {
    ESP_LOGI("BLE", "PASSWORD REJECTED");
    return 0;
  }

  if (data[0] == 0) {
    return 0;
  }

  strncpy(password, data, ctxt->om->om_len);
  password[ctxt->om->om_len] = '\0';

  // Write the new value to NVS
  write_to_nvs(NVS_KEY_PASS, password);

  if (strcmp((char *)wifi_config.sta.password, password) != 0) {
    // Update password in configuration
    strncpy((char *)wifi_config.sta.password, password,
            sizeof(wifi_config.sta.password));

    // Disconnect and reconnect with the new password
    esp_wifi_disconnect();
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    esp_wifi_connect();
    ESP_LOGI(TAG, "Wi-Fi password changed and reconnected");
  }

  return 0;
}

static int descriptor_passwd(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg) {
  char buff[64] = "";
  snprintf(buff, sizeof(buff), "Password: %s", password);
  os_mbuf_append(ctxt->om, buff, strlen(buff));
  return 0;
}

static int write_ssid(uint16_t conn_handle, uint16_t attr_handle,
                      struct ble_gatt_access_ctxt *ctxt, void *arg) {
  char *data = (char *)ctxt->om->om_data;

  data[ctxt->om->om_len] = '\0';
  ESP_LOGI("BLE", "Data received: %s", data);
  ESP_LOGI("BLE", "Data length: %d", ctxt->om->om_len);

  if (data[0] == 0) {
    return 0;
  }

  if (blink == 0) {
    ESP_LOGI("BLE", "SSID REJECTED");
    return 0;
  }

  strncpy(ssid, data, ctxt->om->om_len);
  ssid[ctxt->om->om_len] = '\0';

  // Write the new value to NVS
  write_to_nvs(NVS_KEY_SSID, ssid);

  if (strcmp((char *)wifi_config.sta.ssid, ssid) != 0) {
    // Update SSID in configuration
    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));

    // Disconnect and reconnect with the new SSID
    esp_wifi_disconnect();
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    esp_wifi_connect();
    ESP_LOGI(TAG, "Wi-Fi SSID changed and reconnected");
  }

  return 0;
}

static int descriptor_ssid(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg) {
  char buff[64] = "";
  snprintf(buff, sizeof(buff), "SSID: %s", ssid);
  os_mbuf_append(ctxt->om, buff, strlen(buff));
  return 0;
}

static int write_mqtt_ip(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg) {

  char *data = (char *)ctxt->om->om_data;

  data[ctxt->om->om_len] = '\0';
  ESP_LOGI("BLE", "Data received: %s", data);
  ESP_LOGI("BLE", "Data length: %d", ctxt->om->om_len);

  if (data[0] == 0) {
    return 0;
  }

  strncpy(mqtt_ip, data, ctxt->om->om_len);
  mqtt_ip[ctxt->om->om_len] = '\0';

  write_to_nvs(NVS_KEY_MQTT, mqtt_ip);

  //   if (client != NULL) {
  //     esp_mqtt_client_stop(client);
  //     esp_mqtt_client_destroy(client);
  //   }

  //     mqtt_init();

  return 0;
}

static int descriptor_mqtt_ip(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg) {
  char buff[64] = "";
  snprintf(buff, sizeof(buff), "MQTT IP: %s", mqtt_ip);
  os_mbuf_append(ctxt->om, buff, strlen(buff));
  return 0;
}

static int ble_gap_event(struct ble_gap_event *event, void *arg) {
  switch (event->type) {
  case BLE_GAP_EVENT_CONNECT:
    if (event->connect.status == 0) {
      ESP_LOGI("GAP", "Device connected, handle: %d",
               event->connect.conn_handle);

      for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (conn_handles[i] == 0) {
          conn_handles[i] = event->connect.conn_handle;
          break;
        }
      }

      int connected_count = 0;
      for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (conn_handles[i] != 0) {
          connected_count++;
        }
      }
      if (connected_count < MAX_CONNECTIONS) {
        ble_app_advertise();
      }
    } else {
      ESP_LOGI("GAP", "Connection failed, restarting advertisement");
      ble_app_advertise();
    }
    break;

  case BLE_GAP_EVENT_DISCONNECT:
    ESP_LOGI("GAP", "Device disconnected, handle: %d",
             event->disconnect.conn.conn_handle);

    for (int i = 0; i < MAX_CONNECTIONS; i++) {
      if (conn_handles[i] == event->disconnect.conn.conn_handle) {
        conn_handles[i] = 0;
        break;
      }
    }

    ble_app_advertise();
    break;

  case BLE_GAP_EVENT_ADV_COMPLETE:
    ESP_LOGI("GAP", "Advertisement complete, restarting advertisement");
    ble_app_advertise();
    break;

  default:
    break;
  }
  return 0;
}

void ble_app_advertise(void) {
  struct ble_hs_adv_fields fields;
  const char *device_name;
  memset(&fields, 0, sizeof(fields));
  device_name = ble_svc_gap_device_name(); // Read the BLE device name
  fields.name = (uint8_t *)device_name;
  fields.name_len = strlen(device_name);
  fields.name_is_complete = 1;
  ble_gap_adv_set_fields(&fields);

  struct ble_gap_adv_params adv_params;
  memset(&adv_params, 0, sizeof(adv_params));
  adv_params.conn_mode =
      BLE_GAP_CONN_MODE_UND; // connectable or non-connectable
  adv_params.disc_mode =
      BLE_GAP_DISC_MODE_GEN; // discoverable or non-discoverable
  ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params,
                    ble_gap_event, NULL);
}

void ble_app_on_sync(void) {
  ble_hs_id_infer_auto(0, &ble_addr_type);
  ble_app_advertise();
}

void host_task(void *param) { nimble_port_run(); }

static int write_mqtt_passwd(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg) {
  // size has to be 4
  char *data = (char *)ctxt->om->om_data;

  data[ctxt->om->om_len] = '\0';
  ESP_LOGI("BLE", "Data received: %s", data);
  ESP_LOGI("BLE", "Data length: %d", ctxt->om->om_len);

  if (blink == 0) {
    ESP_LOGI("BLE", "MQTT PASSWORD REJECTED");
    return 0;
  }

  if (ctxt->om->om_len != 4) {
    ESP_LOGI("BLE", "MQTT PASSWORD REJECTED");
    return 0;
  }

  strncpy(mqtt_passwd, data, ctxt->om->om_len);

  mqtt_passwd[ctxt->om->om_len] = '\0';

  // Write the new value to NVS
  write_to_nvs(NVS_KEY_MQTT_PASSWD, mqtt_passwd);

  return 0;
}

static int descriptor_mqtt_passwd(uint16_t conn_handle, uint16_t attr_handle,
                                  struct ble_gatt_access_ctxt *ctxt,
                                  void *arg) {
  char buff[64] = "";
  snprintf(buff, sizeof(buff), "MQTT Password: %s", mqtt_passwd);
  os_mbuf_append(ctxt->om, buff, strlen(buff));
  return 0;
}

static int read_mac(uint16_t conn_handle, uint16_t attr_handle,
                    struct ble_gatt_access_ctxt *ctxt, void *arg) {
  char buff[64] = "";
  snprintf(buff, sizeof(buff), "MAC: %s", mac_str);
  os_mbuf_append(ctxt->om, buff, strlen(buff));
  return 0;
}

static int descriptor_mac(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg) {
  char buff[64] = "";
  snprintf(buff, sizeof(buff), "MAC: %s", mac_str);
  os_mbuf_append(ctxt->om, buff, strlen(buff));
  return 0;
}