/*
 * Об'єднаний код: Bluetooth HID Mouse (MPU6050 + EMG Click)
 * Автор злиття: Gemini AI
 * На основі коду: Yaremko Vitaliy
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_mac.h"
#if CONFIG_BT_NIMBLE_ENABLED
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#else
#include "esp_bt_defs.h"
#if CONFIG_BT_BLE_ENABLED
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#endif
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#if CONFIG_BT_SDP_COMMON_ENABLED
#include "esp_sdp_api.h"
#endif /* CONFIG_BT_SDP_COMMON_ENABLED */
#endif

#include "esp_hidd.h"
#include "esp_hid_gap.h"

// ADC та GPIO
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/adc.h"
#include "sdkconfig.h"
// Бібліотека HID Device (стандартна в ESP-IDF examples, або esp_hidd)
// ПРИМІТКА: Для простоти ми використовуємо емуляцію логіки відправки.
// В реальному проекті потрібно підключити компонент "esp_hid" або скопіювати файли профілю.
// Нижче наведено код, який передбачає наявність API esp_hidd_dev (стандартний приклад).

static const char *TAG = "BLE_MOUSE";

// --- КОНФІГУРАЦІЯ MPU6050 ---
#define I2C_MASTER_SCL_IO 5 /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO 4 /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM 0    /*!< I2C master i2c port number */
#define I2C_MASTER_FREQ_HZ 100000
#define MPU6050_ADDR 0x68
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_WHO_AM_I 0x75

// --- КОНФІГУРАЦІЯ EMG ---
#define EMG_GPIO_PIN GPIO_NUM_1
#define EMG_ADC_CHANNEL ADC_CHANNEL_0
#define EMG_ADC_UNIT ADC_UNIT_1
#define EMG_ADC_ATTEN ADC_ATTEN_DB_12
#define EMG_ADC_BITWIDTH ADC_BITWIDTH_12

// Поріг спрацювання EMG (у мілівольтах). Потрібно підлаштувати під твій м'яз!
#define EMG_CLICK_THRESHOLD_MV 1800
// Гістерезис, щоб кнопка не "дренчала"
#define EMG_RELEASE_THRESHOLD_MV 1500

// --- Змінні MPU ---
float pitch = 0.0, roll = 0.0;
float gyro_offset_x = 0.0, gyro_offset_y = 0.0, gyro_offset_z = 0.0;
float dt = 0.01; // 10ms loop

// --- Змінні ADC ---
static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t adc1_cali_chan0_handle = NULL;
static bool do_calibration1_chan0 = false;

// --- Змінні Bluetooth HID ---
static uint16_t hid_conn_id = 0;
static bool sec_conn = false;

// ==========================================
// ЧАСТИНА 1: ADC / EMG (з твого файлу)
// ==========================================
typedef struct
{
    TaskHandle_t task_hdl;
    esp_hidd_dev_t *hid_dev;
    uint8_t protocol_mode;
    uint8_t *buffer;
} local_param_t;

// static local_param_t s_bt_hid_param = {0};
static local_param_t s_ble_hid_param = {0};
TaskHandle_t s_ble_hid_param_task_hdl = NULL;
const unsigned char mouseReportMap[] = {
    0x05, 0x01, // USAGE_PAGE (Generic Desktop)
    0x09, 0x02, // USAGE (Mouse)
    0xa1, 0x01, // COLLECTION (Application)

    0x09, 0x01, //   USAGE (Pointer)
    0xa1, 0x00, //   COLLECTION (Physical)

    0x05, 0x09, //     USAGE_PAGE (Button)
    0x19, 0x01, //     USAGE_MINIMUM (Button 1)
    0x29, 0x03, //     USAGE_MAXIMUM (Button 3)
    0x15, 0x00, //     LOGICAL_MINIMUM (0)
    0x25, 0x01, //     LOGICAL_MAXIMUM (1)
    0x95, 0x03, //     REPORT_COUNT (3)
    0x75, 0x01, //     REPORT_SIZE (1)
    0x81, 0x02, //     INPUT (Data,Var,Abs)
    0x95, 0x01, //     REPORT_COUNT (1)
    0x75, 0x05, //     REPORT_SIZE (5)
    0x81, 0x03, //     INPUT (Cnst,Var,Abs)

    0x05, 0x01, //     USAGE_PAGE (Generic Desktop)
    0x09, 0x30, //     USAGE (X)
    0x09, 0x31, //     USAGE (Y)
    0x09, 0x38, //     USAGE (Wheel)
    0x15, 0x81, //     LOGICAL_MINIMUM (-127)
    0x25, 0x7f, //     LOGICAL_MAXIMUM (127)
    0x75, 0x08, //     REPORT_SIZE (8)
    0x95, 0x03, //     REPORT_COUNT (3)
    0x81, 0x06, //     INPUT (Data,Var,Rel)

    0xc0, //   END_COLLECTION
    0xc0  // END_COLLECTION
};

static esp_hid_raw_report_map_t bt_report_maps[] = {
    {.data = mouseReportMap,
     .len = sizeof(mouseReportMap)},
};

static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated)
    {
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = EMG_ADC_BITWIDTH,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK)
            calibrated = true;
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated)
    {
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = EMG_ADC_BITWIDTH,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK)
            calibrated = true;
    }
#endif

    *out_handle = handle;
    return calibrated;
}

static void configure_adc(void)
{
    adc_oneshot_unit_init_cfg_t init_config1 = {.unit_id = EMG_ADC_UNIT};
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {.bitwidth = EMG_ADC_BITWIDTH, .atten = EMG_ADC_ATTEN};
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EMG_ADC_CHANNEL, &config));

    do_calibration1_chan0 = adc_calibration_init(EMG_ADC_UNIT, EMG_ADC_CHANNEL, EMG_ADC_ATTEN, &adc1_cali_chan0_handle);
}

static int read_emg_voltage(void)
{
    int adc_raw = 0;
    int voltage = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EMG_ADC_CHANNEL, &adc_raw));

    if (do_calibration1_chan0)
    {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw, &voltage));
    }
    else
    {
        voltage = adc_raw; // Fallback if no calibration
    }
    return voltage;
}

// ==========================================
// ЧАСТИНА 2: I2C / MPU6050 (з твого файлу)
// ==========================================

void i2c_master_init()
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
}

esp_err_t mpu6050_write_byte(uint8_t reg_addr, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t mpu6050_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    if (len == 0)
        return ESP_ERR_INVALID_ARG;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    if (len > 1)
    {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t mpu6050_init_sensor()
{
    esp_err_t ret = mpu6050_write_byte(MPU6050_PWR_MGMT_1, 0x00); // Wake up
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MPU6050: %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    // Test communication by reading WHO_AM_I register
    uint8_t who_am_i;
    ret = mpu6050_read(MPU6050_WHO_AM_I, &who_am_i, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I register: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "MPU6050 WHO_AM_I: 0x%02X (expected: 0x68)", who_am_i);
    if (who_am_i != 0x68) {
        ESP_LOGW(TAG, "WHO_AM_I mismatch, but continuing...");
    }
    
    return ESP_OK;
}

esp_err_t mpu6050_calibrate(int samples)
{
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;
    uint8_t data[6];
    ESP_LOGI(TAG, "Calibrating Gyro... Don't move!");

    for (int i = 0; i < samples; i++)
    {
        esp_err_t ret = mpu6050_read(MPU6050_GYRO_XOUT_H, data, 6);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read gyroscope data during calibration: %s", esp_err_to_name(ret));
            return ret;
        }
        
        int16_t gx = (data[0] << 8) | data[1];
        int16_t gy = (data[2] << 8) | data[3];
        int16_t gz = (data[4] << 8) | data[5];

        sum_gx += gx;
        sum_gy += gy;
        sum_gz += gz;
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
    gyro_offset_x = (sum_gx / samples) / 131.0;
    gyro_offset_y = (sum_gy / samples) / 131.0;
    gyro_offset_z = (sum_gz / samples) / 131.0;
    ESP_LOGI(TAG, "Calibration Done. Offsets: %.2f, %.2f, %.2f", gyro_offset_x, gyro_offset_y, gyro_offset_z);
    return ESP_OK;
}

// ==========================================
// ЧАСТИНА 3: Логіка Миші
// ==========================================

// Функція відправки звіту миші (X, Y, Button)
// void send_mouse_report(int8_t x, int8_t y, uint8_t buttons) {
//     if (sec_conn) { // Тільки якщо Bluetooth підключено
//         // Ця функція є частиною esp_hidd API
//         esp_hidd_send_mouse_value(hid_conn_id, buttons, x, y);
//     }
// }

// send the buttons, change in x, and change in y
void send_mouse(uint8_t buttons, char dx, char dy, char wheel)
{
    static uint8_t buffer[4] = {0};
    buffer[0] = buttons;
    buffer[1] = dx;
    buffer[2] = dy;
    buffer[3] = wheel;
    esp_hidd_dev_input_set(s_ble_hid_param.hid_dev, 0, 0, buffer, 4);
}

// Головне завдання миші
void mouse_logic_task(void *pvParameters)
{
    uint8_t data[14];
    int16_t ax, ay, az, gx, gy, gz;
    float acc_sensitivity = 16384.0;
    float gyro_sensitivity = 131.0;
    float alpha = 0.98; // Комплементарний фільтр

    bool left_click_pressed = false;

    while (1)
    {
        // 1. Читаємо MPU
        esp_err_t mpu_ret = mpu6050_read(MPU6050_ACCEL_XOUT_H, data, 14);
        if (mpu_ret != ESP_OK) {
            ESP_LOGW(TAG, "MPU6050 read failed: %s", esp_err_to_name(mpu_ret));
            // Use previous values or defaults, don't crash
            ax = ay = az = gx = gy = gz = 0;
        } else {
            ax = (data[0] << 8) | data[1];
            ay = (data[2] << 8) | data[3];
            az = (data[4] << 8) | data[5];
            gx = (data[8] << 8) | data[9];
            gy = (data[10] << 8) | data[11];
            gz = (data[12] << 8) | data[13];
        }

        // Конвертація
        float accX = (float)ax / acc_sensitivity;
        float accY = (float)ay / acc_sensitivity;
        float accZ = (float)az / acc_sensitivity;
        float gyroX = ((float)gx / gyro_sensitivity) - gyro_offset_x;
        float gyroY = ((float)gy / gyro_sensitivity) - gyro_offset_y;

        // Розрахунок кутів
        float acc_roll = atan2(accY, accZ) * 180.0 / M_PI;
        float acc_pitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180.0 / M_PI;

        roll = alpha * (roll + gyroX * dt) + (1.0 - alpha) * acc_roll;
        pitch = alpha * (pitch + gyroY * dt) + (1.0 - alpha) * acc_pitch;

        // 2. Маппінг руху миші (Tilt to Move)
        // Якщо нахил Roll > 10 градусів -> рух X
        // Якщо нахил Pitch > 10 градусів -> рух Y

        int8_t mouse_x = 0;
        int8_t mouse_y = 0;

        int deadzone = 10;   // Мертва зона в градусах
        int sensitivity = 2; // Множник швидкості

        // Логіка X (Roll)
        if (roll > deadzone)
            mouse_x = (int8_t)((roll - deadzone) * sensitivity);
        else if (roll < -deadzone)
            mouse_x = (int8_t)((roll + deadzone) * sensitivity);

        // Логіка Y (Pitch) - інвертуємо за потребою
        if (pitch > deadzone)
            mouse_y = (int8_t)((pitch - deadzone) * sensitivity);
        else if (pitch < -deadzone)
            mouse_y = (int8_t)((pitch + deadzone) * sensitivity);

        // Обмеження значень для HID звіту (-127 до 127)
        if (mouse_x > 127)
            mouse_x = 127;

        if (mouse_x < -127)
            mouse_x = -127;

        if (mouse_y > 127)
            mouse_y = 127;

        if (mouse_y < -127)
            mouse_y = -127;

        // 3. Читаємо EMG
        int emg_voltage = read_emg_voltage();
        uint8_t buttons = 0;

        // Логіка кліка (з гістерезисом)
        if (!left_click_pressed && emg_voltage > EMG_CLICK_THRESHOLD_MV)
        {
            left_click_pressed = true;
            ESP_LOGI(TAG, "CLICK DOWN! (Voltage: %d mV)", emg_voltage);
        }
        else if (left_click_pressed && emg_voltage < EMG_RELEASE_THRESHOLD_MV)
        {
            left_click_pressed = false;
            ESP_LOGI(TAG, "CLICK UP! (Voltage: %d mV)", emg_voltage);
        }

        if (left_click_pressed)
        {
            buttons |= 0x01; // Біт 0 = Left Click
        }

        // 4. Відправка даних, якщо є зміни або натиснута кнопка
        if (mouse_x != 0 || mouse_y != 0 || buttons != 0 || left_click_pressed)
        {
            // Навіть якщо x=0, y=0, але кнопка затиснута, треба слати звіт
            send_mouse(buttons, mouse_x, mouse_y, 0);
        }

        // Затримка циклу (важлива для dt та частоти опитування)
        vTaskDelay(pdMS_TO_TICKS(10)); // 10ms = 100Hz
    }
}

// ==========================================
// ЧАСТИНА 4: Ініціалізація системи
// ==========================================
void ble_hid_task_start_up(void)
{
    if (s_ble_hid_param.task_hdl)
    {
        // Task already exists
        return;
    }

    xTaskCreate(&mouse_logic_task, "ble_hid_demo_task_mouse", 3 * 1024, NULL, configMAX_PRIORITIES - 3,
                &s_ble_hid_param.task_hdl);
}
void ble_hid_task_shut_down(void)
{
    if (s_ble_hid_param.task_hdl)
    {
        vTaskDelete(s_ble_hid_param.task_hdl);
        s_ble_hid_param.task_hdl = NULL;
    }
}
// void bt_hid_task_start_up(void)
// {
//     //xTaskCreate(bt_hid_demo_task, "bt_hid_demo_task", 2 * 1024, NULL, configMAX_PRIORITIES - 3, &s_bt_hid_param.task_hdl);
//     xTaskCreate(&mouse_logic_task, "mouse_task", 4096, NULL, 5, NULL);
//     return;
// }

// void bt_hid_task_shut_down(void)
// {
//     if (s_bt_hid_param.task_hdl) {
//         vTaskDelete(s_bt_hid_param.task_hdl);
//         s_bt_hid_param.task_hdl = NULL;
//     }
// }
// Callback подій HID (підключення/відключення)
static void ble_hidd_event_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidd_event_t event = (esp_hidd_event_t)id;
    esp_hidd_event_data_t *param = (esp_hidd_event_data_t *)event_data;
    static const char *TAG = "HID_DEV_BLE";

    switch (event)
    {
    case ESP_HIDD_START_EVENT:
    {
        ESP_LOGI(TAG, "START");
        esp_hid_ble_gap_adv_start();
        break;
    }
    case ESP_HIDD_CONNECT_EVENT:
    {
        ESP_LOGI(TAG, "CONNECT");
        break;
    }
    case ESP_HIDD_PROTOCOL_MODE_EVENT:
    {
        ESP_LOGI(TAG, "PROTOCOL MODE[%u]: %s", param->protocol_mode.map_index, param->protocol_mode.protocol_mode ? "REPORT" : "BOOT");
        break;
    }
    case ESP_HIDD_CONTROL_EVENT:
    {
        ESP_LOGI(TAG, "CONTROL[%u]: %sSUSPEND", param->control.map_index, param->control.control ? "EXIT_" : "");
        if (param->control.control)
        {
            // exit suspend
            ble_hid_task_start_up();
        }
        else
        {
            // suspend
            ble_hid_task_shut_down();
        }
        break;
    }
    case ESP_HIDD_OUTPUT_EVENT:
    {
        ESP_LOGI(TAG, "OUTPUT[%u]: %8s ID: %2u, Len: %d, Data:", param->output.map_index, esp_hid_usage_str(param->output.usage), param->output.report_id, param->output.length);
        ESP_LOG_BUFFER_HEX(TAG, param->output.data, param->output.length);
        break;
    }
    case ESP_HIDD_FEATURE_EVENT:
    {
        ESP_LOGI(TAG, "FEATURE[%u]: %8s ID: %2u, Len: %d, Data:", param->feature.map_index, esp_hid_usage_str(param->feature.usage), param->feature.report_id, param->feature.length);
        ESP_LOG_BUFFER_HEX(TAG, param->feature.data, param->feature.length);
        break;
    }
    case ESP_HIDD_DISCONNECT_EVENT:
    {
        ESP_LOGI(TAG, "DISCONNECT: %s", esp_hid_disconnect_reason_str(esp_hidd_dev_transport_get(param->disconnect.dev), param->disconnect.reason));
        ble_hid_task_shut_down();
        esp_hid_ble_gap_adv_start();
        break;
    }
    case ESP_HIDD_STOP_EVENT:
    {
        ESP_LOGI(TAG, "STOP");
        break;
    }
    default:
        break;
    }
    return;
}

static esp_hid_raw_report_map_t ble_report_maps[] = {
    {.data = mouseReportMap,
     .len = sizeof(mouseReportMap)},
};

static esp_hid_device_config_t ble_hid_config = {
    .vendor_id = 0x16C0,
    .product_id = 0x05DF,
    .version = 0x0100,
#if CONFIG_EXAMPLE_HID_DEVICE_ROLE == 2
    .device_name = "ESP Keyboard",
#elif CONFIG_EXAMPLE_HID_DEVICE_ROLE == 3
    .device_name = "ESP Mouse",
#else
    .device_name = "ESP BLE HID2",
#endif
    .manufacturer_name = "Espressif",
    .serial_number = "1234567890",
    .report_maps = ble_report_maps,
    .report_maps_len = 1};


void app_main(void)
{
    esp_err_t ret;
#if HID_DEV_MODE == HIDD_IDLE_MODE
    ESP_LOGE(TAG, "Please turn on BT HID device or BLE!");
    return;
#endif
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize peripherals first
    ESP_LOGI(TAG, "Initializing I2C and ADC...");
    i2c_master_init();
    configure_adc();

    ESP_LOGI(TAG, "Initializing MPU6050...");
    ret = mpu6050_init_sensor();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "MPU6050 initialization failed, continuing without IMU");
    } else {
        ret = mpu6050_calibrate(100); // Calibrate gyroscope (don't move the board!)
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "MPU6050 calibration failed, using default offsets");
        }
    }

    ESP_LOGI(TAG, "setting hid gap, mode:%d", HID_DEV_MODE);
    ret = esp_hid_gap_init(HID_DEV_MODE);
    ESP_ERROR_CHECK(ret);

#if CONFIG_BT_BLE_ENABLED || CONFIG_BT_NIMBLE_ENABLED
    #if CONFIG_EXAMPLE_HID_DEVICE_ROLE == 2
        ret = esp_hid_ble_gap_adv_init(ESP_HID_APPEARANCE_KEYBOARD, ble_hid_config.device_name);
    #elif CONFIG_EXAMPLE_HID_DEVICE_ROLE == 3
        ret = esp_hid_ble_gap_adv_init(ESP_HID_APPEARANCE_MOUSE, ble_hid_config.device_name);
    #else
        ret = esp_hid_ble_gap_adv_init(ESP_HID_APPEARANCE_GENERIC, ble_hid_config.device_name);
    #endif
        ESP_ERROR_CHECK(ret);
    #if CONFIG_BT_BLE_ENABLED
        if ((ret = esp_ble_gatts_register_callback(esp_hidd_gatts_event_handler)) != ESP_OK)
        {
            ESP_LOGE(TAG, "GATTS register callback failed: %d", ret);
            return;
        }
    #endif
        ESP_LOGI(TAG, "setting ble device");
        ESP_ERROR_CHECK(
            esp_hidd_dev_init(&ble_hid_config, ESP_HID_TRANSPORT_BLE, ble_hidd_event_callback, &s_ble_hid_param.hid_dev));
#endif

#if CONFIG_BT_HID_DEVICE_ENABLED
    ESP_LOGI(TAG, "setting device name");
    esp_bt_gap_set_device_name(bt_hid_config.device_name);
    ESP_LOGI(TAG, "setting cod major, peripheral");
    esp_bt_cod_t cod = {0};
    cod.major = ESP_BT_COD_MAJOR_DEV_PERIPHERAL;
    cod.minor = ESP_BT_COD_MINOR_PERIPHERAL_POINTING;
    esp_bt_gap_set_cod(cod, ESP_BT_SET_COD_MAJOR_MINOR);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "setting bt device");
    ESP_ERROR_CHECK(
        esp_hidd_dev_init(&bt_hid_config, ESP_HID_TRANSPORT_BT, bt_hidd_event_callback, &s_bt_hid_param.hid_dev));
#if CONFIG_BT_SDP_COMMON_ENABLED
    ESP_ERROR_CHECK(esp_sdp_register_callback(esp_sdp_cb));
    ESP_ERROR_CHECK(esp_sdp_init());
#endif /* CONFIG_BT_SDP_COMMON_ENABLED */
#endif /* CONFIG_BT_HID_DEVICE_ENABLED */
#if CONFIG_BT_NIMBLE_ENABLED
    /* XXX Need to have template for store */
    ble_store_config_init();

    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    /* Starting nimble task after gatts is initialized*/
    ret = esp_nimble_enable(ble_hid_device_host_task);
    if (ret)
    {
        ESP_LOGE(TAG, "esp_nimble_enable failed: %d", ret);
    }
#endif
}

// void app_main(void)
// {
//     esp_err_t ret;

//     // 1. Init NVS (потрібно для BT)
//     ret = nvs_flash_init();
//     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
//         ESP_ERROR_CHECK(nvs_flash_erase());
//         ret = nvs_flash_init();
//     }
//     ESP_ERROR_CHECK(ret);

//     // 2. Init Peripherals
//     ESP_LOGI(TAG, "Initializing I2C and ADC...");
//     i2c_master_init();
//     configure_adc();

//     ESP_LOGI(TAG, "Initializing MPU6050...");
//     mpu6050_init_sensor();
//     mpu6050_calibrate(100); // Калібрування гіроскопа (не рухай плату!)

//     // 3. Init Bluetooth HID Device
//     ESP_LOGI(TAG, "Initializing Bluetooth HID...");
//     // В реальному прикладі тут ініціалізується стек ESP-HID
//     // Цей рядок вимагає налаштованого компоненту esp_hid_device
//     esp_hid_gap_init(ESP_BT_MODE_BLE);
//     esp_hid_ble_gap_adv_init(ESP_HID_APPEARANCE_MOUSE, "ESP32 EMG Mouse");

//     ESP_ERROR_CHECK(
//         esp_hidd_dev_init(&ble_hid_config, ESP_HID_TRANSPORT_BLE, ble_hidd_event_callback, &s_ble_hid_param.hid_dev));
//     // 4. Start Logic Task
//     //xTaskCreate(&mouse_logic_task, "mouse_task", 4096, NULL, 5, NULL);
// }