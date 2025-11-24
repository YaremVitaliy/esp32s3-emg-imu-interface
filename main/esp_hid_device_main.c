/*
 * Об'єднаний код: Bluetooth HID Air Mouse (MPU6050 + EMG Click)
 * Автор злиття: Gemini AI
 * На основі коду: Yaremko Vitaliy
 * 
 * AIR MOUSE MODE:
 * - Використовує гіроскоп для детекції переміщення в просторі
 * - Рух X контролюється поворотом навколо Y-осі (ліво-право)
 * - Рух Y контролюється поворотом навколо X-осі (вгору-вниз)
 * - EMG сигнал використовується для лівої кнопки миші
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

#define MOUSE_SENSITIVITY 10
#define MOTION_GAIN 2
#define YAW_DRIFT_DEADZONE 0.6f
#define YAW_BIAS_SLOW_ALPHA 0.002f
#define YAW_BIAS_FAST_ALPHA 0.02f
#define YAW_STILL_GYRO_THRESH 1.5f
#define YAW_STILL_HOLD_MS 150
#define YAW_INPUT_MIN   -350
#define YAW_INPUT_MAX    900
#define YAW_OUTPUT_MIN -1400
#define YAW_OUTPUT_MAX  3200
#define ROLL_INPUT_MIN  -600
#define ROLL_INPUT_MAX   600
#define ROLL_OUTPUT_MIN 1600
#define ROLL_OUTPUT_MAX -2000

// --- КОНФІГУРАЦІЯ EMG ---
#define EMG_GPIO_PIN GPIO_NUM_1
#define EMG_ADC_CHANNEL ADC_CHANNEL_0
#define EMG_ADC_UNIT ADC_UNIT_1
#define EMG_ADC_ATTEN ADC_ATTEN_DB_12
#define EMG_ADC_BITWIDTH ADC_BITWIDTH_12

// Покращені параметри EMG детекції
#define EMG_BASELINE_SAMPLES 120           // Кількість зразків для обчислення базової лінії
#define EMG_FILTER_SAMPLES 8               // Кількість зразків для ковзного середнього
#define EMG_NOISE_THRESHOLD_MV 60          // Мінімальна різниця від базової лінії для адаптації
#define EMG_ACTIVATION_THRESHOLD_MV 50     // Мінімальний поріг активації відносно базової лінії 70
#define EMG_RELEASE_THRESHOLD_MV 35        // Мінімальний поріг відпускання відносно базової лінії
#define EMG_MIN_ACTIVATION_TIME_MS 50      // Мінімальний час утримання для валідного кліка
#define EMG_DEBOUNCE_TIME_MS 150           // Час антидеребезгу між кліками
#define EMG_BASELINE_UPDATE_RATE 0.0003    // Швидкість адаптації базової лінії
#define EMG_STABILITY_CHECK_SAMPLES 3      // Кількість стабільних зразків для підтвердження активації
#define EMG_REFRACTORY_TIME_MS 250         // Час "мертвого" періоду після відпускання
#define EMG_DYNAMIC_MARGIN_MV 15           // Додаткова надбавка до динамічного порога
#define EMG_NOISE_UPDATE_ALPHA 0.008f      // Швидкість оновлення оцінки шуму
#define EMG_RELEASE_SCALE 0.2f             // Відносний рівень відпускання відносно порога активації
#define EMG_LIVE_LOG_ENABLED 0             // 1 = виводити EMG у реальному часі
#define EMG_LIVE_LOG_INTERVAL_MS 50        // Період логування в мілісекундах

// --- Змінні MPU ---
float pitch = 0.0, roll = 0.0;
float gyro_offset_x = 0.0, gyro_offset_y = 0.0, gyro_offset_z = 0.0;
float dt = 0.01; // 10ms loop
static float yaw_angle = 0.0f;
static float yaw_bias = 0.0f;

typedef struct {
    int32_t prev_abs_x;
    int32_t prev_abs_y;
    int32_t residual_dx;
    int32_t residual_dy;
} mouse_mapping_state_t;

static mouse_mapping_state_t mouse_map_state = {0};

static inline int32_t map_value(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
    if (in_max == in_min) {
        return out_min;
    }
    return (int32_t)((int64_t)(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

static inline int32_t clamp_i32(int32_t value, int32_t min_value, int32_t max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

// --- Змінні ADC ---
static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t adc1_cali_chan0_handle = NULL;
static bool do_calibration1_chan0 = false;

// --- Структура для EMG фільтрації ---
typedef struct {
    int voltage_buffer[EMG_FILTER_SAMPLES];
    int buffer_index;
    float baseline_voltage;
    bool baseline_initialized;
    uint32_t last_activation_time;
    uint32_t last_deactivation_time;
    bool is_active;
    bool is_confirmed_active;  // Підтверджена активація після мінімального часу
    int activation_counter;
    int stable_activation_count;  // Лічильник стабільних високих показань
    int stable_release_count;     // Лічильник стабільних низьких показань
    float noise_floor_mv;         // Оцінка рівня шуму (амплітуда)
    float dynamic_activation_threshold_mv;
    float dynamic_release_threshold_mv;
    uint32_t refractory_until_ms;
} emg_filter_t;

static emg_filter_t emg_filter = {0};
static int emg_diag_last_diff = 0;
static uint32_t emg_diag_last_log_ms = 0;

// --- Змінні Bluetooth HID ---
static uint16_t hid_conn_id = 0;
static bool sec_conn = false;
static bool ble_connected = false;

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

// Ініціалізація EMG фільтра
static void emg_filter_init(void)
{
    memset(&emg_filter, 0, sizeof(emg_filter_t));
    emg_filter.baseline_voltage = 1500.0; // Початкове значення базової лінії
    emg_filter.is_active = false;
    emg_filter.is_confirmed_active = false;
    emg_filter.stable_activation_count = 0;
    emg_filter.stable_release_count = 0;
    emg_filter.noise_floor_mv = 0.0f;
    emg_filter.dynamic_activation_threshold_mv = EMG_ACTIVATION_THRESHOLD_MV;
    emg_filter.dynamic_release_threshold_mv = EMG_RELEASE_THRESHOLD_MV;
    emg_filter.refractory_until_ms = 0;
}

// Оновлення ковзного середнього для EMG
static int emg_filter_update(int new_voltage)
{
    // Додаємо новий зразок до буфера
    emg_filter.voltage_buffer[emg_filter.buffer_index] = new_voltage;
    emg_filter.buffer_index = (emg_filter.buffer_index + 1) % EMG_FILTER_SAMPLES;
    
    // Обчислюємо ковзне середнє
    int sum = 0;
    for (int i = 0; i < EMG_FILTER_SAMPLES; i++) {
        sum += emg_filter.voltage_buffer[i];
    }
    int filtered_voltage = sum / EMG_FILTER_SAMPLES;
    
    // Ініціалізуємо або повільно адаптуємо базову лінію
    if (!emg_filter.baseline_initialized) {
        // Перші кілька зразків - швидка ініціалізація
        static int init_samples = 0;
        if (init_samples < EMG_BASELINE_SAMPLES) {
            emg_filter.baseline_voltage = (emg_filter.baseline_voltage * init_samples + filtered_voltage) / (init_samples + 1);
            init_samples++;
        } else {
            emg_filter.baseline_initialized = true;
            ESP_LOGI(TAG, "EMG baseline initialized: %.1f mV", emg_filter.baseline_voltage);
        }
    } else {
        // Повільна адаптація базової лінії тільки коли немає активності
        int diff = abs(filtered_voltage - (int)emg_filter.baseline_voltage);
        if (diff < EMG_NOISE_THRESHOLD_MV && !emg_filter.is_active) {
            emg_filter.baseline_voltage = emg_filter.baseline_voltage * (1.0 - EMG_BASELINE_UPDATE_RATE) + 
                                         filtered_voltage * EMG_BASELINE_UPDATE_RATE;
        }
    }

    // Оцінюємо рівень шуму та динамічні пороги, коли базова лінія вже визначена
    if (emg_filter.baseline_initialized) {
        int baseline_mv = (int)emg_filter.baseline_voltage;
        int voltage_diff = filtered_voltage - baseline_mv;
        int voltage_diff_abs = abs(voltage_diff);

        if (!emg_filter.is_active) {
            float alpha = EMG_NOISE_UPDATE_ALPHA;
            float capped_diff = (float)voltage_diff_abs;
            if (emg_filter.dynamic_activation_threshold_mv > 0 &&
                capped_diff > emg_filter.dynamic_activation_threshold_mv * 0.8f) {
                // Ігноруємо великі сплески при оновленні шуму
                capped_diff = emg_filter.dynamic_activation_threshold_mv * 0.8f;
            }
            if (emg_filter.noise_floor_mv == 0.0f) {
                emg_filter.noise_floor_mv = capped_diff;
            } else {
                emg_filter.noise_floor_mv = (1.0f - alpha) * emg_filter.noise_floor_mv + alpha * capped_diff;
            }
        }

        float dynamic_activation = fmaxf((float)EMG_ACTIVATION_THRESHOLD_MV,
                                         emg_filter.noise_floor_mv + (float)EMG_DYNAMIC_MARGIN_MV);
        float dynamic_release = fmaxf((float)EMG_RELEASE_THRESHOLD_MV,
                                      dynamic_activation * EMG_RELEASE_SCALE);

        emg_filter.dynamic_activation_threshold_mv = dynamic_activation;
        emg_filter.dynamic_release_threshold_mv = dynamic_release;
    }
    
    return filtered_voltage;
}

// Покращена детекція EMG активації з перевіркою стабільності
static bool emg_detect_activation(int filtered_voltage)
{
    if (!emg_filter.baseline_initialized) {
        return false; // Не детектуємо поки базова лінія не ініціалізована
    }
    
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    int voltage_diff = filtered_voltage - (int)emg_filter.baseline_voltage;
    float activation_threshold = emg_filter.dynamic_activation_threshold_mv;
    float release_threshold = emg_filter.dynamic_release_threshold_mv;
    if (activation_threshold <= 0.0f) {
        activation_threshold = EMG_ACTIVATION_THRESHOLD_MV;
    }
    if (release_threshold <= 0.0f) {
        release_threshold = EMG_RELEASE_THRESHOLD_MV;
    }
    bool above_activation = (float)voltage_diff > activation_threshold;
    bool below_release = (float)voltage_diff < release_threshold;
    
    // Періодичне діагностичне логування при відсутності активації
    if (!emg_filter.is_active && emg_filter.baseline_initialized && activation_threshold > 0.0f) {
        if (voltage_diff > 0 && (float)voltage_diff > activation_threshold * 0.45f) {
            if (current_time - emg_diag_last_log_ms >= 500) {
                emg_diag_last_log_ms = current_time;
                emg_diag_last_diff = voltage_diff;
                ESP_LOGI(TAG, "EMG monitor: Diff=%d mV, Th=%.1f/%.1f, Noise=%.1f", 
                         voltage_diff, activation_threshold, release_threshold, emg_filter.noise_floor_mv);
            }
        }
    }
    
    // Примусова пауза після відпускання
    if (current_time < emg_filter.refractory_until_ms) {
        return false;
    }
    
    // Перевіряємо антидеребезг
    if (current_time - emg_filter.last_deactivation_time < EMG_DEBOUNCE_TIME_MS) {
        return emg_filter.is_confirmed_active; // Зберігаємо попередній ПІДТВЕРДЖЕНИЙ стан під час дебаунсу
    }
    
    if (!emg_filter.is_active) {
        // Перевіряємо чи сигнал стабільно високий
        if (above_activation) {
            emg_filter.stable_activation_count++;
            emg_filter.stable_release_count = 0; // Скидаємо лічильник відпускання
            
            // Активуємо тільки після кількох стабільних високих зразків
            if (emg_filter.stable_activation_count >= EMG_STABILITY_CHECK_SAMPLES) {
                emg_filter.last_activation_time = current_time;
                emg_filter.activation_counter++;
                emg_filter.is_active = true;
                emg_filter.is_confirmed_active = false; // Ще не підтверджено
                ESP_LOGI(TAG, "EMG ACTIVATION START! Diff: %d mV, Th: %.1f/%.1f, Baseline: %.1f mV", 
                         voltage_diff, activation_threshold, release_threshold, emg_filter.baseline_voltage);
            }
        } else {
            emg_filter.stable_activation_count = 0; // Скидаємо лічильник активації
        }
    } else {
        // Перевіряємо мінімальний час активації для підтвердження
        if (!emg_filter.is_confirmed_active && 
            current_time - emg_filter.last_activation_time >= EMG_MIN_ACTIVATION_TIME_MS) {
            emg_filter.is_confirmed_active = true;
            ESP_LOGI(TAG, "EMG ACTIVATION CONFIRMED! Duration: %lu ms", 
                     current_time - emg_filter.last_activation_time);
        }
        
        // Перевіряємо деактивацію - також потрібна стабільність
        if (below_release) {
            emg_filter.stable_release_count++;
            emg_filter.stable_activation_count = 0; // Скидаємо лічильник активації
            
            // Деактивуємо тільки після кількох стабільних низьких зразків
            if (emg_filter.stable_release_count >= EMG_STABILITY_CHECK_SAMPLES) {
                emg_filter.last_deactivation_time = current_time;
                emg_filter.is_active = false;
                emg_filter.is_confirmed_active = false;
                emg_filter.stable_activation_count = 0;
                emg_filter.stable_release_count = 0;
                emg_filter.refractory_until_ms = current_time + EMG_REFRACTORY_TIME_MS;
                ESP_LOGI(TAG, "EMG DEACTIVATION! Diff: %d mV, Total Duration: %lu ms (Refractory %u ms)", 
                         voltage_diff, current_time - emg_filter.last_activation_time, EMG_REFRACTORY_TIME_MS);
            }
        } else {
            emg_filter.stable_release_count = 0; // Скидаємо лічильник відпускання
        }
    }
    
    // Повертаємо тільки підтверджену активацію
    return emg_filter.is_confirmed_active;
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

// send the buttons, change in x, and change in y
void send_mouse(uint8_t buttons, char dx, char dy, char wheel)
{
    if (!ble_connected || !s_ble_hid_param.hid_dev) {
        // Не відправляємо дані, якщо не підключені
        ESP_LOGW(TAG, "Attempted to send mouse report while not connected");
        return;
    }
    
    static uint8_t buffer[4] = {0};
    buffer[0] = buttons;
    buffer[1] = dx;
    buffer[2] = dy;
    buffer[3] = wheel;
    
    esp_err_t ret = esp_hidd_dev_input_set(s_ble_hid_param.hid_dev, 0, 0, buffer, 4);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to send mouse report: %s", esp_err_to_name(ret));
    }
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
    
    // Ініціалізуємо EMG фільтр
    emg_filter_init();
    ESP_LOGI(TAG, "EMG filter initialized. Collecting baseline samples...");
    static bool absolute_position_initialized = false;
    static bool yaw_stationary = false;
    static uint32_t yaw_stationary_start_ms = 0;
    mouse_map_state.prev_abs_x = 0;
    mouse_map_state.prev_abs_y = 0;
    mouse_map_state.residual_dx = 0;
    mouse_map_state.residual_dy = 0;
    absolute_position_initialized = false;
    yaw_stationary = false;
    yaw_stationary_start_ms = 0;
    yaw_bias = 0.0f;
    
    // Low-pass filter for angle smoothing (still used for diagnostics)
    float filtered_roll = 0.0f;
    float filtered_yaw = 0.0f;
    float filter_alpha = 0.7f;  // Filter coefficient (0 = no filtering, 1 = max filtering)
    
    // Movement detection
    uint32_t last_movement_time = 0;
    const uint32_t movement_timeout_ms = 50;  // Мінімальний час між рухами

    while (1)
    {
        // 1. Читаємо MPU
        esp_err_t mpu_ret = mpu6050_read(MPU6050_ACCEL_XOUT_H, data, 14);
        if (mpu_ret != ESP_OK) {
            //ESP_LOGW(TAG, "MPU6050 read failed: %s", esp_err_to_name(mpu_ret));
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
        
        // Застосовуємо фільтр низьких частот для згладжування
        filtered_roll = filter_alpha * filtered_roll + (1.0f - filter_alpha) * roll;

        // Інтегруємо Z-вісь гіроскопа для розрахунку кута yaw
        float gyroZ = ((float)gz / gyro_sensitivity) - gyro_offset_z;
        yaw_angle += gyroZ * dt;
        if (yaw_angle > 180.0f) {
            yaw_angle -= 360.0f;
        } else if (yaw_angle < -180.0f) {
            yaw_angle += 360.0f;
        }
        filtered_yaw = filter_alpha * filtered_yaw + (1.0f - filter_alpha) * yaw_angle;

        uint32_t loop_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        bool gyro_still = fabsf(gyroX) < YAW_STILL_GYRO_THRESH &&
                          fabsf(gyroY) < YAW_STILL_GYRO_THRESH &&
                          fabsf(gyroZ) < YAW_STILL_GYRO_THRESH;

        if (gyro_still) {
            if (!yaw_stationary) {
                yaw_stationary = true;
                yaw_stationary_start_ms = loop_time_ms;
            }
        } else {
            yaw_stationary = false;
        }

        float bias_alpha = 0.0f;
        if (yaw_stationary && (loop_time_ms - yaw_stationary_start_ms) >= YAW_STILL_HOLD_MS) {
            bias_alpha = YAW_BIAS_FAST_ALPHA;
        } else if (fabsf(gyroZ) < YAW_DRIFT_DEADZONE) {
            bias_alpha = YAW_BIAS_SLOW_ALPHA;
        }

        if (bias_alpha > 0.0f) {
            yaw_bias = yaw_bias * (1.0f - bias_alpha) + filtered_yaw * bias_alpha;
        }
        float corrected_yaw = filtered_yaw - yaw_bias;

        // 2. Arduino-style yaw/pitch mapping for mouse movement
        double dx = (double)(-corrected_yaw) * MOUSE_SENSITIVITY;
        double dz = (double)filtered_roll * MOUSE_SENSITIVITY;
        int32_t raw_x = (int32_t)dx;
        int32_t raw_y = (int32_t)dz;

        int32_t mapped_x = map_value(raw_x, YAW_INPUT_MIN, YAW_INPUT_MAX, YAW_OUTPUT_MIN, YAW_OUTPUT_MAX);
        int32_t mapped_y = map_value(raw_y, ROLL_INPUT_MIN, ROLL_INPUT_MAX, ROLL_OUTPUT_MIN, ROLL_OUTPUT_MAX);

        int32_t delta_x = 0;
        int32_t delta_y = 0;
        if (!absolute_position_initialized) {
            mouse_map_state.prev_abs_x = mapped_x;
            mouse_map_state.prev_abs_y = mapped_y;
            absolute_position_initialized = true;
        } else {
            delta_x = mapped_x - mouse_map_state.prev_abs_x;
            delta_y = mapped_y - mouse_map_state.prev_abs_y;
            mouse_map_state.prev_abs_x = mapped_x;
            mouse_map_state.prev_abs_y = mapped_y;
        }

        delta_x *= MOTION_GAIN;
        delta_y *= MOTION_GAIN;

        // Додаємо залишок (аналог OffsetX/OffsetY у Arduino коді)
        int32_t desired_dx = delta_x + mouse_map_state.residual_dx;
        int32_t desired_dy = delta_y + mouse_map_state.residual_dy;

        int8_t mouse_x = (int8_t)clamp_i32(desired_dx, -127, 127);
        int8_t mouse_y = (int8_t)clamp_i32(desired_dy, -127, 127);

        mouse_map_state.residual_dx = desired_dx - mouse_x;
        mouse_map_state.residual_dy = desired_dy - mouse_y;

        // 3. Читаємо та обробляємо EMG з покращеною фільтрацією
        int raw_emg_voltage = read_emg_voltage();
        int filtered_emg_voltage = emg_filter_update(raw_emg_voltage);
        
        // Використовуємо покращену детекцію активації
        bool current_emg_active = emg_detect_activation(filtered_emg_voltage);
        
        uint8_t buttons = 0;
        if (current_emg_active) {
            buttons |= 0x01; // Біт 0 = Left Click
        }
        
        // Оновлюємо стан для логіки відправки
        bool emg_state_changed = (current_emg_active != left_click_pressed);
        left_click_pressed = current_emg_active;
        uint32_t current_time = loop_time_ms;

    #if EMG_LIVE_LOG_ENABLED
        static uint32_t last_live_log_ms = 0;
        if (emg_filter.baseline_initialized && current_time - last_live_log_ms >= EMG_LIVE_LOG_INTERVAL_MS) {
            last_live_log_ms = current_time;
            float live_act_th = emg_filter.dynamic_activation_threshold_mv > 0.0f ?
                    emg_filter.dynamic_activation_threshold_mv : (float)EMG_ACTIVATION_THRESHOLD_MV;
            float live_rel_th = emg_filter.dynamic_release_threshold_mv > 0.0f ?
                    emg_filter.dynamic_release_threshold_mv : (float)EMG_RELEASE_THRESHOLD_MV;
            int live_diff = filtered_emg_voltage - (int)emg_filter.baseline_voltage;
            ESP_LOGI(TAG, "EMG LIVE: raw=%d mV, filt=%d mV, base=%.1f mV, diff=%d mV, thr=%.1f/%.1f, noise=%.1f, active=%s",
                 raw_emg_voltage, filtered_emg_voltage, emg_filter.baseline_voltage, live_diff,
                 live_act_th, live_rel_th, emg_filter.noise_floor_mv,
                 current_emg_active ? "YES" : "NO");
        }
    #endif

        // 4. Відправка даних тільки при справжніх змінах з контролем часу
        bool movement_detected = (mouse_x != 0 || mouse_y != 0);
        
        // Відправляємо дані тільки якщо:
        // 1) Змінилася кнопка (негайно)
        // 2) Є рух И пройшов мінімальний час з останньої відправки
        bool should_send = emg_state_changed || 
                          (movement_detected && (current_time - last_movement_time >= movement_timeout_ms));
        
        if (should_send && ble_connected) {
            send_mouse(buttons, mouse_x, mouse_y, 0);
            
            if (movement_detected) {
                last_movement_time = current_time;
            }
            
            // Логування тільки для значущих подій
            if (movement_detected || emg_state_changed) {
                // ESP_LOGI(TAG, "AIR MOUSE - dx: %d, dy: %d, Buttons: %02X, mapped(%ld,%ld)",
                //          mouse_x, mouse_y, buttons, (long)mapped_x, (long)mapped_y);
            }
        } else if (emg_state_changed && !ble_connected) {
            ESP_LOGW(TAG, "Not connected - Button change detected: %02X", buttons);
        }
        
        // Діагностика тільки при значних змінах кутів (рівень DEBUG)
        static float last_logged_yaw = 0.0f;
        static float last_logged_roll = 0.0f;
        if (fabs(corrected_yaw - last_logged_yaw) > 2.0f || fabs(filtered_roll - last_logged_roll) > 2.0f) {
            ESP_LOGI(TAG, "YPR mapping - Yaw: %.2f, Roll: %.2f, delta(%ld,%ld), residual(%ld,%ld)",
                     corrected_yaw, filtered_roll, (long)delta_x, (long)delta_y,
                     (long)mouse_map_state.residual_dx, (long)mouse_map_state.residual_dy);
            last_logged_yaw = corrected_yaw;
            last_logged_roll = filtered_roll;
        }
        
        // Додаткове діагностичне логування для EMG (тільки при змінах)
        if (emg_filter.baseline_initialized) {
            static int last_logged_diff = 0;
            int current_diff = filtered_emg_voltage - (int)emg_filter.baseline_voltage;
            if (abs(current_diff - last_logged_diff) > 20 || emg_state_changed) {
                ESP_LOGD(TAG, "EMG: Raw=%d, Filtered=%d, Baseline=%.1f, Diff=%d, Active=%s", 
                         raw_emg_voltage, filtered_emg_voltage, emg_filter.baseline_voltage, 
                         current_diff, current_emg_active ? "YES" : "NO");
                last_logged_diff = current_diff;
            }
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
        ble_connected = true;
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
        ble_connected = false;
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