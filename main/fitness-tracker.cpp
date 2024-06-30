#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "sdkconfig.h"
#include <esp_timer.h>
#include <esp_log.h>
#include <esp_system.h>
#include <driver/i2c.h>
#include <driver/gpio.h>
#include <unity.h>
#include <mpu6050.h>
#include "heart_rate_algorithm.h"
#include <string.h>
#include "ui.h"

#define SENSOR_DEBUG 0

const char *TAG = "fitness-tracker";
const char *TAG1 = "mpu6050";
const char *TAG2 = "max30102";

int selecting_item = 0;
int current_screen = 1;
float stride_length = 0.8;
int heart_rate_threshold = 100;
int scale = 0;

LGFX lcd;

// mpu6050 configuration
#define I2C_MASTER_SCL_IO 19      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 18      /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_1  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

#define SAMPLE_COUNT 50
#define CALIBRATION_INTERVAL (5000) // 10 seconds in microseconds

typedef struct
{
    float x;
    float y;
    float z;
} mpu6050_data_t;

static QueueHandle_t queue;
static mpu6050_handle_t mpu6050_dev = NULL;
static float threshX = 0;
static float threshY = 0;
static float threshZ = 0;
static int step_count = 0;
static int step_flag = 0;

void mpu_i2c_bus_init(void)
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "I2C config returned error");

    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "I2C install returned error");
}

void i2c_sensor_mpu6050_init(void)
{
    esp_err_t ret;

    mpu_i2c_bus_init();
    mpu6050_dev = mpu6050_create(I2C_MASTER_NUM, MPU6050_I2C_ADDRESS);
    TEST_ASSERT_NOT_NULL_MESSAGE(mpu6050_dev, "MPU6050 create returned NULL");

    ret = mpu6050_config(mpu6050_dev, ACCE_FS_4G, GYRO_FS_500DPS);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    ret = mpu6050_wake_up(mpu6050_dev);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}

void sort_array(float *array, size_t size)
{
    for (size_t i = 0; i < size - 1; i++)
    {
        for (size_t j = 0; j < size - i - 1; j++)
        {
            if (array[j] > array[j + 1])
            {
                float temp = array[j];
                array[j] = array[j + 1];
                array[j + 1] = temp;
            }
        }
    }
}

void calibrate_thresholds(mpu6050_data_t *data)
{
    float x_vals[SAMPLE_COUNT], y_vals[SAMPLE_COUNT], z_vals[SAMPLE_COUNT];
    for (int i = 0; i < SAMPLE_COUNT; i++)
    {
        x_vals[i] = data[i].x;
        y_vals[i] = data[i].y;
        z_vals[i] = data[i].z;
    }

    // sort_array(x_vals, SAMPLE_COUNT);
    // sort_array(y_vals, SAMPLE_COUNT);
    // sort_array(z_vals, SAMPLE_COUNT);
    float sumX = 0, sumY = 0, sumZ = 0;
    for (int i = 0; i < SAMPLE_COUNT; i++)
    {
        sumX += x_vals[i];
        sumY += y_vals[i];
        sumZ += z_vals[i];
    }
    threshX = sumX / SAMPLE_COUNT;
    threshY = sumY / SAMPLE_COUNT;
    threshZ = sumZ / SAMPLE_COUNT;
    // threshX = (x_vals[0] + x_vals[SAMPLE_COUNT - 1]) / 2;
    // threshY = (y_vals[0] + y_vals[SAMPLE_COUNT - 1]) / 2;
    // threshZ = (z_vals[0] + z_vals[SAMPLE_COUNT - 1]) / 2;

    if (SENSOR_DEBUG)
        ESP_LOGI(TAG1, "Calibration complete: threshX=%.2f, threshY=%.2f, threshZ=%.2f", threshX, threshY, threshZ);
}

void step_counting_task(void *arg)
{
    mpu6050_acce_value_t acce;

    while (1)
    {
        // last_calibration_time = esp_timer_get_time();
        esp_err_t ret = mpu6050_get_acce(mpu6050_dev, &acce);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG1, "Failed to get accelerometer data");
            continue;
        }
        float x = acce.acce_x;
        float y = acce.acce_y;
        float z = acce.acce_z;

        // Print the X, Y, Z values
        if (SENSOR_DEBUG)
            ESP_LOGI(TAG1, "X: %.2f, Y: %.2f, Z: %.2f", x, y, z);

        if ((threshX != 0) || (threshY != 0) || (threshZ != 0))
        {
            if ((x > threshX) && (y > threshY) && (z > threshZ))
            {   
                step_count++;
                step_flag = 1;
                if (SENSOR_DEBUG)
                    ESP_LOGI(TAG1, "Step flag: %d", step_flag);
            }
            else if ((x < threshX) && (y < threshY) && (z < threshZ))
            {
                if (step_flag == 1)
                {
                    step_count++;
                    step_flag = 0;
                    if (SENSOR_DEBUG)
                        ESP_LOGI(TAG1, "Step flag: %d", step_flag);
                    if (SENSOR_DEBUG)
                        ESP_LOGW(TAG1, "Step detected. Total steps: %d", step_count);
                }
            }
        }

        mpu6050_data_t data = {x, y, z};
        // ESP_LOGI(TAG1, "Free queue space: %d", uxQueueSpacesAvailable(queue));
        BaseType_t status = xQueueSendToBack(queue, &data, pdMS_TO_TICKS(100));
        if (status != pdTRUE)
        {
            ESP_LOGE(TAG1, "Failed to send data to queue");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void calibration_task(void *arg)
{
    mpu6050_data_t data_buffer[SAMPLE_COUNT];
    vTaskDelay(pdMS_TO_TICKS(5000));
    while (1)
    {
        mpu6050_data_t item;
        for (int i = 0; i < SAMPLE_COUNT; i++)
        {
            xQueueReceive(queue, &item, portMAX_DELAY);
            data_buffer[i] = item;
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        xQueueReset(queue);
        calibrate_thresholds(data_buffer);
        // ESP_LOGW(TAG1,"finished collecting data\n");
        //  for (int i = 0; i < SAMPLE_COUNT; i++) {
        //    printf("Data %d: x=%.2f, y=%.2f, z=%.2f\n", i+1, data_buffer[i].x, data_buffer[i].y, data_buffer[i].z);
        //  }
        vTaskDelay(pdMS_TO_TICKS(CALIBRATION_INTERVAL));
    }
}

void mpu6050_init()
{
    queue = xQueueCreate(SAMPLE_COUNT, sizeof(mpu6050_data_t));
    if (queue == NULL)
    {
        ESP_LOGE(TAG1, "Failed to create queue");
    }
    else
    {
        ESP_LOGI(TAG1, "Queue created successfully");
    }
    i2c_sensor_mpu6050_init();
    xTaskCreate(step_counting_task, "step_counting_task", 4096, NULL, 5, NULL);
    xTaskCreate(calibration_task, "calibration_task", 4096, NULL, 5, NULL);
}

// max30102 configuration
#define MAX30102_I2C_SCL 22           // GPIO number used for I2C master clock
#define MAX30102_I2C_SDA 21           // GPIO number used for I2C master data
#define MAX30102_GPIO_INT 23          // GPIO number used for MAX30102 interrupt
#define I2C_MASTER_NUM_2 I2C_NUM_0    // I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip
#define MAX30102_I2C_FREQ_HZ 50000    // I2C master clock frequency
#define MAX30102_I2C_TX_BUF_DISABLE 0 // I2C master doesn't need buffer
#define MAX30102_I2C_RX_BUF_DISABLE 0
#define MAX30102_I2C_TIMEOUT_MS 1000
#define DEBUG 1
#define MAX30102_ADDR 0x57 // MAX30102 I2C address

#define get_millis() (esp_timer_get_time() / 1000)
#define RATE_SIZE 4
static uint8_t rates[RATE_SIZE]; // Array of heart rates
static uint8_t rateSpot = 0;
static long lastBeat = 0; // Time at which the last beat occurred

static float beatsPerMinute;
static int beatAvg;

esp_err_t max30102_i2c_init()
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER,
    conf.sda_io_num = MAX30102_I2C_SDA,
    conf.scl_io_num = MAX30102_I2C_SCL,
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE,
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE,
    conf.master.clk_speed = 50000,
    conf.clk_flags = 0;
    i2c_param_config(I2C_MASTER_NUM_2, &conf);

    return i2c_driver_install(I2C_MASTER_NUM_2, conf.mode, MAX30102_I2C_RX_BUF_DISABLE, MAX30102_I2C_TX_BUF_DISABLE, 0);
}

esp_err_t max30102_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM_2, MAX30102_ADDR, &reg_addr, 1, data, len, MAX30102_I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t max30102_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM_2, MAX30102_ADDR, write_buf, sizeof(write_buf), MAX30102_I2C_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
}

esp_err_t max30102_gpio_intr_init()
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << MAX30102_GPIO_INT);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    return ESP_OK;
}

void gpio_intr_task(void *arg)
{
    uint8_t byte[6];
    int32_t data[2];
    for (;;)
    {
        ESP_ERROR_CHECK(max30102_register_read(0x07, byte, 6));
        data[0] = ((byte[0] << 16 | byte[1] << 8 | byte[2]) & 0x03ffff);
        data[1] = ((byte[3] << 16 | byte[4] << 8 | byte[5]) & 0x03ffff);
        if (SENSOR_DEBUG)
            printf("Red: %ld, IR: %ld\n", data[0], data[1]);
        vTaskDelay(20 / portTICK_PERIOD_MS);
        if (checkForBeat(data[1]) == true)
        {
            // We sensed a beat!
            long delta = get_millis() - lastBeat;
            if (SENSOR_DEBUG)
                printf("Delta: %ld\n", delta);
            lastBeat = get_millis();

            beatsPerMinute = 60 / (delta / 1000.0);

            if (beatsPerMinute < 255 && beatsPerMinute > 20)
            {
                rates[rateSpot++] = (uint8_t)beatsPerMinute; // Store this reading in the array
                rateSpot %= RATE_SIZE;                       // Wrap variable

                // Take average of readings
                beatAvg = 0;
                for (uint8_t x = 0; x < RATE_SIZE; x++)
                    beatAvg += rates[x];
                beatAvg /= RATE_SIZE;
            }
        }

        if (SENSOR_DEBUG)
            ESP_LOGI(TAG2, "IR= %ld, BPM= %f, Avg BPM= %d\n", data[1], beatsPerMinute, beatAvg);

        if (data[1] < 50000)
        {
            if (SENSOR_DEBUG)
                ESP_LOGI(TAG2, " No finger?\n");
        }
    }
}

void max30102_init()
{
    ESP_ERROR_CHECK(max30102_i2c_init());
    ESP_LOGI(TAG2, "MAX30102 I2C initialized successfully");

    // reset
    ESP_ERROR_CHECK(max30102_register_write_byte(0x09, 0x40));
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // Interrupt Enable
    ESP_ERROR_CHECK(max30102_register_write_byte(0x02, 0xc0)); // enable interrupts: A_FULL: FIFO Almost Full Flag and PPG_RDY: New FIFO Data Ready
    ESP_ERROR_CHECK(max30102_register_write_byte(0x03, 0x02)); // enable interrupt: DIE_TEMP_RDY: Internal Temperature Ready Flag

    // FIFO
    ESP_ERROR_CHECK(max30102_register_write_byte(0x04, 0x00)); // clear FIFO Write Pointer
    ESP_ERROR_CHECK(max30102_register_write_byte(0x05, 0x00)); // clear FIFO Overflow Counter
    ESP_ERROR_CHECK(max30102_register_write_byte(0x06, 0x00)); // clear FIFO Read Pointer

    // FIFO Configuration
    ESP_ERROR_CHECK(max30102_register_write_byte(0x08, 0x0f)); // SMP_AVE = 0b000: 1 averaging, FIFO_ROLLOVER_EN = 0, FIFO_A_FULL = 0xf

    // Mode Configuration
    ESP_ERROR_CHECK(max30102_register_write_byte(0x09, 0x03)); // MODE = 0b011: SpO2 mode

    // SpO2 Configuration
    ESP_ERROR_CHECK(max30102_register_write_byte(0x0a, 0x47)); // SPO2_ADC_RGE = 0b10: 8192, SPO2_SR = 0b001: 100 SAMPLES PER SECOND,
                                                               // LED_PW = 0b11: PULSE WIDTH 411, ADC RESOLUTION 18

    // LED Pulse Amplitude
    ESP_ERROR_CHECK(max30102_register_write_byte(0x0c, 0x50)); // LED1_PA(red) = 0x24, LED CURRENT 16mA
    ESP_ERROR_CHECK(max30102_register_write_byte(0x0d, 0x50)); // LED2_PA(IR) = 0x24, LED CURRENT 16mA
    // ESP_ERROR_CHECK(max30102_register_write_byte(0x10, 0x50)); // PILOT_PA = 0x24, LED CURRENT 16mA

    // clear PPG_RDY ! Cannot receive the first interrupt without clearing !
    uint8_t data;
    ESP_ERROR_CHECK(max30102_register_read(0x00, &data, 1));
    ESP_LOGI(TAG2, "Interrupt Status 1: 0x%x", data);
    ESP_ERROR_CHECK(max30102_register_read(0x01, &data, 1));
    ESP_LOGI(TAG2, "Interrupt Status 2: 0x%x", data);
    xTaskCreate(gpio_intr_task, "gpio_intr_task", 2048, NULL, 10, NULL);
}

// encoder configuration
#define ENCODER_A_GPIO GPIO_NUM_32
#define ENCODER_B_GPIO GPIO_NUM_33
#define ENCODER_BUTTON_GPIO GPIO_NUM_25

static volatile int16_t last_count = 0;
static volatile int16_t encoder_count = 0;
static volatile int8_t last_state_encoded = 0;
static volatile int8_t state_encoded = 0;
static volatile int64_t button_press_time = 0;
uint32_t button_pressed = 0;

SemaphoreHandle_t button_semaphore;
TaskHandle_t task1_handle = NULL;
TaskHandle_t task2_handle = NULL;

static void IRAM_ATTR encoder_isr_handler(void *arg)
{
    int8_t state_encoded = gpio_get_level(ENCODER_A_GPIO);
    if (state_encoded != last_state_encoded)
    {
        if (gpio_get_level(ENCODER_B_GPIO) != state_encoded)
        {
            encoder_count++;
        }
        else
        {
            encoder_count--;
        }
    }
    last_state_encoded = state_encoded;
}

static void IRAM_ATTR button_isr_handler(void *arg)
{
    if (gpio_get_level(ENCODER_BUTTON_GPIO) == 0)
    {
        button_press_time = esp_timer_get_time();
    }
    else
    {
        if (esp_timer_get_time() - button_press_time > 500000)
        {
            button_pressed = 2; // Long press
        }
        else
        {
            button_pressed = 1; // Short press
        }
        xSemaphoreGiveFromISR(button_semaphore, NULL);
    }
}

void rotary_task(void *pvParameter)
{
    while (1)
    {
        if (xSemaphoreTake(button_semaphore, portMAX_DELAY) == pdTRUE)
        {
            if (button_pressed == 1)
            {
                printf("Button pressed\n");
                if (current_screen == 1 && selecting_item == 2)
                {
                    current_screen = 3;
                }
                else if (current_screen == 1 && selecting_item == 1)
                {
                    current_screen = 2;
                }
                else if (current_screen == 2)
                {
                    heart_rate_threshold += scale;
                    scale = 0;
                    update_screen_1(&lcd, beatAvg, step_count, selecting_item + 1, stride_length);
                    current_screen = 1;
                }
                else if (current_screen == 3)
                {   
                    stride_length = stride_length + (scale * 0.1);
                    scale = 0;
                    update_screen_1(&lcd, beatAvg, step_count, selecting_item + 1, stride_length);
                    current_screen = 1;
                }
            }
            else if (button_pressed == 2)
            {
                printf("Button long pressed\n");
            }
        }
    }
}

void button_task(void *pvParameter)
{
    while (1)
    {
        if (last_count != encoder_count)
        {
            if (encoder_count > last_count)
            {
                printf("Clockwise\n");
                if (current_screen == 1)
                {
                    if (selecting_item < 3)
                        selecting_item++;
                    else
                    {
                        selecting_item = 1;
                    }
                }
                else
                {
                    scale++;
                }
            }
            else
            {
                printf("Counter-clockwise\n");
                if (current_screen == 1)
                {
                    if (selecting_item > 1)
                        selecting_item--;
                    else
                    {
                        selecting_item = 3;
                    }
                }
                else
                {
                    scale--;
                }
            }
        }
        last_count = encoder_count;
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void rotary_encoder_init()
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_ANYEDGE;                                      // Interrupt on both rising and falling edges
    io_conf.mode = GPIO_MODE_INPUT;                                             // Set as input mode
    io_conf.pin_bit_mask = (1ULL << ENCODER_A_GPIO) | (1ULL << ENCODER_B_GPIO); // Set pins
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;                               // Disable pull-down mode
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;                                    // Enable pull-up mode
    gpio_config(&io_conf);

    // Configure GPIO for button
    gpio_config_t button_conf = {};
    button_conf.intr_type = GPIO_INTR_ANYEDGE;                // Interrupt on both rising and falling edges
    button_conf.mode = GPIO_MODE_INPUT;                       // Set as input mode
    button_conf.pin_bit_mask = (1ULL << ENCODER_BUTTON_GPIO); // Set pin
    button_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;         // Disable pull-down mode
    button_conf.pull_up_en = GPIO_PULLUP_ENABLE;              // Enable pull-up mode
    gpio_config(&button_conf);

    // Install ISR service
    gpio_install_isr_service(0);
    gpio_isr_handler_add(ENCODER_A_GPIO, encoder_isr_handler, (void *)ENCODER_A_GPIO);
    gpio_isr_handler_add(ENCODER_B_GPIO, encoder_isr_handler, (void *)ENCODER_B_GPIO);
    gpio_isr_handler_add(ENCODER_BUTTON_GPIO, button_isr_handler, (void *)ENCODER_BUTTON_GPIO);

    button_semaphore = xSemaphoreCreateBinary();
    xTaskCreate(&rotary_task, "rotary_task", 1024, NULL, 5, &task1_handle);
    xTaskCreate(&button_task, "button_task", 1024, NULL, 5, &task2_handle);
}

void sys_monitor_task(void *arg)
{   
    // char info_buf[1000];
    while (1)
    {
        ESP_LOGI(TAG, "RAM left: %ld bytes", esp_get_free_heap_size());
        // vTaskGetRunTimeStats(info_buf);
        // printf("%s\n", info_buf);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

extern "C"
{
    void app_main(void)
    {
        mpu6050_init();
        max30102_init();
        rotary_encoder_init();
        lcd.init();
        xTaskCreate(sys_monitor_task, "sys_monitor_task", 8192, NULL, 5, NULL);
        update_screen_1(&lcd, 0, 0, selecting_item, stride_length);
        vTaskDelay(100);
        while (1)
        {
            if (current_screen == 1)
            {
                // printf("running screen 1\n");
                update_screen_1(&lcd, beatAvg, step_count, selecting_item, stride_length);
            }
            else if (current_screen == 2)
                update_screen_2(&lcd, &heart_rate_threshold, &scale);
            else if (current_screen == 3)
                update_screen_3(&lcd, &stride_length, &scale);
            vTaskDelay(20);
        }
    }
}