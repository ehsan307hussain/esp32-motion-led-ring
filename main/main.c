#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"
#include "led_strip.h"
#include "led_strip_rmt.h"

// ESP32 + MPU6050 (GY-521) + 37 WS2812B LEDs, ESP-IDF 6.1
#define LED_GPIO           18
#define LED_COUNT          37
#define START_LED          10   // Original LED 11 (zero-based index)
#define SDA_GPIO           21
#define SCL_GPIO           22
#define MPU_ADDR           0x68
#define REVERSE_LEDS       false

#define PI_F               3.14159265358979323846f
#define DEG_TO_RAD         (PI_F / 180.0f)
#define GRAVITY            9.81f

// 37 LEDs at approximately 30 LEDs/meter make a ~0.196m radius circle.
// This is an effective radius; adjust later if the mounting geometry differs.
#define RING_RADIUS        0.20f
#define ROLL_FACTOR        (5.0f / 7.0f)
#define VELOCITY_DAMPING   1.1f
#define ROLL_RESISTANCE    0.35f
#define STATIC_THRESHOLD   0.25f
#define STOP_SPEED         0.025f
#define MAX_ANGULAR_SPEED  8.0f

// Experimental shake impulse, using Y-axis high-pass acceleration.
#define SHAKE_GAIN         0.35f
#define SHAKE_THRESHOLD    0.25f
#define SHAKE_SIGN         1.0f

#define FILTER_ALPHA       0.98f
#define GYRO_DEADBAND      1.5f
#define SPEED_SLOW_LIMIT   0.8f   // ball angular speed (rad/s)
#define SPEED_FAST_LIMIT   2.0f
#define FRAME_MS           10

static const char *TAG = "PHYSICS_RING";
static led_strip_handle_t strip;
static i2c_master_bus_handle_t bus;
static i2c_master_dev_handle_t mpu;

// Physics angle along ring, in radians. Unbounded: permits multiple laps.
static float ball_angle = 0.0f;
static float ball_speed = 0.0f;

// Measured frame orientation, in continuous degrees.
static float level_angle = 0.0f;
static float gyro_bias = 0.0f;
static float rotation_angle = 0.0f;
static float smooth_ay = 0.0f;

static float clampf_local(float v, float lo, float hi)
{
    return fminf(hi, fmaxf(lo, v));
}

static float wrap180(float degrees)
{
    float a = fmodf(degrees + 180.0f, 360.0f);
    if (a < 0.0f) a += 360.0f;
    return a - 180.0f;
}

static int16_t unpack16(uint8_t hi, uint8_t lo)
{
    return (int16_t)(((uint16_t)hi << 8) | lo);
}

static void init_leds(void)
{
    led_strip_config_t config = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = LED_COUNT,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags.invert_out = false
    };
    led_strip_rmt_config_t rmt = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10000000,
        .mem_block_symbols = 64,
        .flags.with_dma = false
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&config, &rmt, &strip));
    ESP_ERROR_CHECK(led_strip_clear(strip));
}

static void init_mpu(void)
{
    i2c_master_bus_config_t cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = SDA_GPIO,
        .scl_io_num = SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&cfg, &bus));

    i2c_device_config_t dev = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU_ADDR,
        .scl_speed_hz = 100000
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev, &mpu));
    uint8_t wake[] = {0x6B, 0x00};
    ESP_ERROR_CHECK(i2c_master_transmit(mpu, wake, sizeof(wake), 1000));
    vTaskDelay(pdMS_TO_TICKS(100));
}

static esp_err_t read_sensor(float *ay, float *az, float *gx)
{
    uint8_t reg = 0x3B;
    uint8_t data[14];
    esp_err_t err = i2c_master_transmit_receive(
        mpu, &reg, 1, data, sizeof(data), 1000);
    if (err != ESP_OK) return err;
    *ay = (float)unpack16(data[2], data[3]);
    *az = (float)unpack16(data[4], data[5]);
    *gx = (float)unpack16(data[8], data[9]);
    return ESP_OK;
}

static void calibrate_sensor(void)
{
    ESP_LOGI(TAG, "Hold the ring UPRIGHT and STILL during calibration");
    float angle_sum = 0.0f, gyro_sum = 0.0f, ay_sum = 0.0f;
    int samples = 0;
    for (int i = 0; i < 150; ++i) {
        float ay, az, gx;
        if (read_sensor(&ay, &az, &gx) == ESP_OK) {
            angle_sum += atan2f(ay, az) / DEG_TO_RAD;
            gyro_sum += gx;
            ay_sum += ay;
            ++samples;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (samples == 0) ESP_ERROR_CHECK(ESP_FAIL);
    level_angle = angle_sum / samples;
    gyro_bias = gyro_sum / samples;
    smooth_ay = ay_sum / samples;
    rotation_angle = 0.0f;
    ball_angle = 0.0f;
    ball_speed = 0.0f;
    ESP_LOGI(TAG, "Calibration: level=%.2f gyro_bias=%.1f",
             level_angle, gyro_bias);
}

// Combines gyro integration with accelerometer correction across full turns.
static float update_rotation(float ay, float az, float gx, float dt)
{
    float measured = atan2f(ay, az) / DEG_TO_RAD;
    float relative = wrap180(measured - level_angle);
    float rate = (gx - gyro_bias) / 131.0f; // default +/-250 deg/s
    if (fabsf(rate) < GYRO_DEADBAND) rate = 0.0f;

    float predicted = rotation_angle + rate * dt;
    float error = wrap180(relative - predicted);
    rotation_angle = predicted + (1.0f - FILTER_ALPHA) * error;
    return rotation_angle * DEG_TO_RAD;
}

static float detect_shake(float ay, float dt)
{
    float alpha = dt / (0.22f + dt);
    smooth_ay += alpha * (ay - smooth_ay);
    float dynamic_g = (ay - smooth_ay) / 16384.0f;
    if (fabsf(dynamic_g) < SHAKE_THRESHOLD) return 0.0f;
    return -SHAKE_SIGN * SHAKE_GAIN * dynamic_g * GRAVITY;
}

static void update_physics(float ring_tilt, float shake_accel, float dt)
{
    // Gravity pulls ball toward the world's bottom point of the circular track.
    // Opposite sides of the ring naturally oscillate around the lower equilibrium.
    float gravity_accel = -(ROLL_FACTOR * GRAVITY / RING_RADIUS) *
                          sinf(ball_angle - ring_tilt);
    float external_accel = (ROLL_FACTOR / RING_RADIUS) * shake_accel;
    float drive = gravity_accel + external_accel;
    float net = drive - VELOCITY_DAMPING * ball_speed;

    if (fabsf(ball_speed) > STOP_SPEED) {
        net -= copysignf(ROLL_RESISTANCE, ball_speed);
    } else if (fabsf(drive) < STATIC_THRESHOLD) {
        ball_speed = 0.0f;
        return;
    }

    float next_speed = ball_speed + net * dt;
    if (fabsf(ball_speed) > STOP_SPEED &&
        ball_speed * next_speed < 0.0f &&
        fabsf(drive) < ROLL_RESISTANCE) {
        next_speed = 0.0f;
    }

    ball_speed = clampf_local(next_speed,
                              -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);
    ball_angle += ball_speed * dt;
    // No edge collisions: the track is now a closed 37-LED circle.
}

static int ball_to_led(void)
{
    float offset = ball_angle * (float)LED_COUNT / (2.0f * PI_F);
    int index = START_LED + (int)roundf(offset);
    index %= LED_COUNT;
    if (index < 0) index += LED_COUNT;
    if (REVERSE_LEDS) index = (LED_COUNT - index) % LED_COUNT;
    return index;
}

static void render_ball(void)
{
    float speed = fabsf(ball_speed);
    uint32_t red = 0, green = 0, blue = 0;
    if (speed < SPEED_SLOW_LIMIT) {
        green = 75;
    } else if (speed < SPEED_FAST_LIMIT) {
        red = 75;
        green = 50;
    } else {
        red = 90;
    }

    ESP_ERROR_CHECK(led_strip_clear(strip));
    ESP_ERROR_CHECK(led_strip_set_pixel(
        strip, ball_to_led(), red, green, blue));
    ESP_ERROR_CHECK(led_strip_refresh(strip));
}

void app_main(void)
{
    ESP_LOGI(TAG, "37-LED gravity-based ring simulation");
    init_leds();
    init_mpu();
    calibrate_sensor();

    int64_t previous = esp_timer_get_time();
    int log_counter = 0;
    while (1) {
        int64_t now = esp_timer_get_time();
        float dt = (now - previous) / 1000000.0f;
        previous = now;
        dt = clampf_local(dt, 0.001f, 0.03f);

        float ay, az, gx;
        if (read_sensor(&ay, &az, &gx) == ESP_OK) {
            float tilt = update_rotation(ay, az, gx, dt);
            float shake = detect_shake(ay, dt);
            update_physics(tilt, shake, dt);
            render_ball();

            if (++log_counter >= 30) {
                ESP_LOGI(TAG,
                         "Tilt:%7.1f Ball:%7.2f Speed:%6.2f LED:%2d Shake:%5.2f",
                         tilt / DEG_TO_RAD, ball_angle, ball_speed,
                         ball_to_led(), shake);
                log_counter = 0;
            }
        } else {
            ESP_LOGW(TAG, "MPU read failed");
        }
        vTaskDelay(pdMS_TO_TICKS(FRAME_MS));
    }
}