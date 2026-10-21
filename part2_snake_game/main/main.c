#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"
#include "led_strip.h"
#include "led_strip_rmt.h"

#define LED_PIN                18
#define LED_COUNT              37
#define START_LED              0  // Physical LED 11 (zero-based)
#define SDA_PIN                21
#define SCL_PIN                22
#define MPU_ADDR               0x68
#define FRAME_MS               10

// Change to -1 if rotating the ring moves the snake in the wrong direction.
#define DIRECTION_SIGN         1
#define GYRO_DEADBAND_DPS      3.0f
#define STEP_DEGREES           (360.0f / LED_COUNT)
#define SLOW_DPS               15.0f
#define FAST_DPS               70.0f
#define MIN_FOOD_DISTANCE      3  // Aim for three LED steps from nearest snake end
#define MAX_STEPS_PER_FRAME    8

static const char *TAG = "SNAKE_RING";
static led_strip_handle_t strip;
static i2c_master_bus_handle_t i2c_bus;
static i2c_master_dev_handle_t mpu;

// The snake is a contiguous arc on the 37-LED circle.
// left_end -> (left_end + snake_length - 1) is the other end.
static int left_end = START_LED;
static int snake_length = 1;
static int food_led = -1;
static int active_side = 1;  // -1: left tip leads, +1: right tip leads
static bool game_won = false;
static float gyro_bias = 0.0f;
static float movement_degrees = 0.0f;
static float filtered_speed = 0.0f;

static int wrap_index(int i)
{
    i %= LED_COUNT;
    return (i < 0) ? i + LED_COUNT : i;
}

static int right_end_index(void)
{
    return wrap_index(left_end + snake_length - 1);
}

static bool is_snake_led(int index)
{
    for (int i = 0; i < snake_length; ++i) {
        if (wrap_index(left_end + i) == index) return true;
    }
    return false;
}

// Shortest circular separation between LEDs 0..36.
static int circular_distance(int a, int b)
{
    int clockwise = wrap_index(a - b);
    int counterclockwise = wrap_index(b - a);
    return clockwise < counterclockwise ? clockwise : counterclockwise;
}

static int16_t read_i16(uint8_t hi, uint8_t lo)
{
    return (int16_t)(((uint16_t)hi << 8) | lo);
}

static void init_strip(void)
{
    led_strip_config_t cfg = {
        .strip_gpio_num = LED_PIN,
        .max_leds = LED_COUNT,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags.invert_out = false,
    };
    led_strip_rmt_config_t rmt = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10000000,
        .mem_block_symbols = 64,
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&cfg, &rmt, &strip));
    ESP_ERROR_CHECK(led_strip_clear(strip));
}

static void init_sensor(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2c_bus));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU_ADDR,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &dev_cfg, &mpu));

    uint8_t wake[] = {0x6B, 0x00};
    ESP_ERROR_CHECK(i2c_master_transmit(mpu, wake, sizeof(wake), 1000));
    vTaskDelay(pdMS_TO_TICKS(100));
}

static esp_err_t read_gyro_x(float *raw_gx)
{
    uint8_t reg = 0x43;  // GYRO_XOUT_H
    uint8_t bytes[2];
    esp_err_t err = i2c_master_transmit_receive(
        mpu, &reg, 1, bytes, sizeof(bytes), 1000);
    if (err == ESP_OK) {
        *raw_gx = (float)read_i16(bytes[0], bytes[1]);
    }
    return err;
}

static void calibrate_gyro(void)
{
    ESP_LOGI(TAG, "Keep ring completely still for calibration");
    float total = 0.0f;
    int count = 0;
    for (int i = 0; i < 150; ++i) {
        float raw;
        if (read_gyro_x(&raw) == ESP_OK) {
            total += raw;
            ++count;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_ERROR_CHECK(count ? ESP_OK : ESP_FAIL);
    gyro_bias = total / count;
    ESP_LOGI(TAG, "Gyro X bias %.1f", gyro_bias);
}

static void select_food(void)
{
    int candidates[LED_COUNT];
    int count = 0;
    int left = wrap_index(left_end);
    int right = right_end_index();

    // Prefer food exactly 3 positions from its CLOSEST end.
    // This avoids demanding a large rotation of the wired ring.
    for (int i = 0; i < LED_COUNT; ++i) {
        if (is_snake_led(i)) continue;
        int d_left = circular_distance(i, left);
        int d_right = circular_distance(i, right);
        int nearest = d_left < d_right ? d_left : d_right;
        if (nearest == MIN_FOOD_DISTANCE) {
            candidates[count++] = i;
        }
    }

    // As the snake fills the ring, there may not be a free LED at distance 3.
    // Prefer the closest remaining LED, so finishing the game stays possible.
    if (count == 0) {
        int best_distance = LED_COUNT;
        for (int i = 0; i < LED_COUNT; ++i) {
            if (is_snake_led(i)) continue;
            int d_left = circular_distance(i, left);
            int d_right = circular_distance(i, right);
            int nearest = d_left < d_right ? d_left : d_right;
            if (nearest < best_distance) {
                best_distance = nearest;
                count = 0;
            }
            if (nearest == best_distance) candidates[count++] = i;
        }
        ESP_LOGI(TAG, "Food distance relaxed (ring nearly full)");
    }

    if (count == 0) {
        food_led = -1;
        return;
    }
    food_led = candidates[esp_random() % (unsigned)count];
    ESP_LOGI(TAG, "New food LED %d (snake length %d/%d)",
             food_led + 1, snake_length, LED_COUNT);
}

static void reset_game(void)
{
    left_end = START_LED;
    snake_length = 1;
    food_led = -1;
    active_side = 1;
    game_won = false;
    movement_degrees = 0.0f;
    filtered_speed = 0.0f;
    select_food();
    ESP_LOGI(TAG, "NEW GAME: Rotate the ring to move the snake");
}

static void move_snake_one_led(int direction)
{
    if (game_won) return;

    active_side = direction;
    int target = direction > 0
        ? wrap_index(right_end_index() + 1)
        : wrap_index(left_end - 1);
    bool ate = target == food_led;

    if (direction < 0) {
        // Left tip advances into target. If no food, the rightmost tip vacates.
        left_end = target;
    } else if (!ate) {
        // Right tip advances; tail on the left vacates unless food is eaten.
        left_end = wrap_index(left_end + 1);
    }

    if (ate) {
        ++snake_length;
        ESP_LOGI(TAG, "FOOD EATEN! Snake length %d/%d", snake_length, LED_COUNT);
        printf("EVENT:EAT\n");
        fflush(stdout);
        if (snake_length >= LED_COUNT) {
            game_won = true;
            food_led = -1;
            ESP_LOGI(TAG, "YOU WIN! Full ring!");
            printf("EVENT:WIN\n");
            fflush(stdout);
        } else {
            select_food();
        }
    }
}

static void update_game(float signed_dps, float dt)
{
    float speed = fabsf(signed_dps);
    filtered_speed = 0.75f * filtered_speed + 0.25f * speed;
    if (game_won) return;

    if (speed < GYRO_DEADBAND_DPS) return;
    movement_degrees += signed_dps * DIRECTION_SIGN * dt;

    int steps = 0;
    while (fabsf(movement_degrees) >= STEP_DEGREES &&
           steps++ < MAX_STEPS_PER_FRAME && !game_won) {
        int direction = movement_degrees > 0.0f ? 1 : -1;
        move_snake_one_led(direction);
        movement_degrees -= direction * STEP_DEGREES;
    }
}

static void render_game(void)
{
    ESP_ERROR_CHECK(led_strip_clear(strip));
    for (int i = 0; i < snake_length; ++i) {
        ESP_ERROR_CHECK(led_strip_set_pixel(strip,
            wrap_index(left_end + i), 0, 18, 0));
    }

    if (!game_won) {
        int head = active_side < 0 ? wrap_index(left_end) : right_end_index();
        uint32_t red = 0, green = 0, blue = 0;
        if (filtered_speed < SLOW_DPS) {
            green = 100;           // Slow: bright green tip
        } else if (filtered_speed < FAST_DPS) {
            red = 85; green = 65; // Medium: yellow tip
        } else {
            blue = 110;           // Fast: blue tip
        }
        ESP_ERROR_CHECK(led_strip_set_pixel(strip, head, red, green, blue));
        if (food_led >= 0) {
            ESP_ERROR_CHECK(led_strip_set_pixel(strip, food_led, 100, 0, 0));
        }
    }
    ESP_ERROR_CHECK(led_strip_refresh(strip));
}

static void victory_animation(void)
{
    // All RGB values are deliberately modest for a 5V/2A LED supply.
    static const uint8_t colors[][3] = {
        {100,   0,   0}, // red
        {  0, 100,   0}, // green
        {  0,   0, 100}, // blue
        { 85,  55,   0}, // yellow
        { 55,   0,  80}, // purple
        {  0,  75,  65}, // cyan
        { 85,  25,   0}, // orange
    };
    for (int pass = 0; pass < 3; ++pass) {
        for (unsigned c = 0; c < sizeof(colors) / sizeof(colors[0]); ++c) {
            for (int led = 0; led < LED_COUNT; ++led) {
                ESP_ERROR_CHECK(led_strip_set_pixel(strip, led,
                    colors[c][0], colors[c][1], colors[c][2]));
            }
            ESP_ERROR_CHECK(led_strip_refresh(strip));
            vTaskDelay(pdMS_TO_TICKS(150));
        }
    }
    ESP_ERROR_CHECK(led_strip_clear(strip));
    ESP_ERROR_CHECK(led_strip_refresh(strip));
    vTaskDelay(pdMS_TO_TICKS(1000));
}

void app_main(void)
{
    init_strip();
    init_sensor();
    calibrate_gyro();
    reset_game();
    render_game();

    int64_t previous = esp_timer_get_time();
    int log_count = 0;
    while (true) {
        int64_t now = esp_timer_get_time();
        float dt = (now - previous) / 1000000.0f;
        previous = now;
        if (dt < 0.001f) dt = 0.001f;
        if (dt > 0.03f) dt = 0.03f;

        float raw;
        if (read_gyro_x(&raw) == ESP_OK && !game_won) {
            float signed_dps = (raw - gyro_bias) / 131.0f;
            update_game(signed_dps, dt);
            render_game();
            if (++log_count >= 30) {
                ESP_LOGI(TAG, "rate=%6.1f dps, length=%d/%d, food LED=%d",
                         signed_dps, snake_length, LED_COUNT, food_led + 1);
                log_count = 0;
            }
        }
        if (game_won) {
            victory_animation();
            reset_game();
            render_game();
            previous = esp_timer_get_time();
        }
        vTaskDelay(pdMS_TO_TICKS(FRAME_MS));
    }
}