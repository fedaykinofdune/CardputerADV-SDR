/* Cardputer ADV ST7789V2: 135x240 panel used landscape (240x135) on SPI3. */
#include "cardputer.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_st7789.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define PIN_MOSI 35
#define PIN_SCLK 36
#define PIN_CS 37
#define PIN_DC 34
#define PIN_RST 33
#define PIN_BL 38

static esp_lcd_panel_handle_t panel;
static SemaphoreHandle_t done;
static uint16_t *strip;

static bool on_done(esp_lcd_panel_io_handle_t io, esp_lcd_panel_io_event_data_t *e, void *ctx) {
    (void)io; (void)e; (void)ctx;
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(done, &woken);
    return woken == pdTRUE;
}

void cp_lcd_init(void) {
    done = xSemaphoreCreateBinary();
    strip = heap_caps_malloc(CP_LCD_W * CP_STRIP_H * 2, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    assert(strip);
    const spi_bus_config_t bus = {
        .mosi_io_num = PIN_MOSI, .miso_io_num = -1, .sclk_io_num = PIN_SCLK,
        .quadwp_io_num = -1, .quadhd_io_num = -1,
        .max_transfer_sz = CP_LCD_W * CP_STRIP_H * 2,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &bus, SPI_DMA_CH_AUTO));
    const esp_lcd_panel_io_spi_config_t io_cfg = {
        .cs_gpio_num = PIN_CS, .dc_gpio_num = PIN_DC, .spi_mode = 0,
        .pclk_hz = 40 * 1000 * 1000, .trans_queue_depth = 4,
        .on_color_trans_done = on_done, .lcd_cmd_bits = 8, .lcd_param_bits = 8,
    };
    esp_lcd_panel_io_handle_t io;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI3_HOST, &io_cfg, &io));
    const esp_lcd_panel_dev_config_t dev = {
        .reset_gpio_num = PIN_RST, .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB, .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io, &dev, &panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel, true));
    cp_lcd_flip(false);
    /* Clear before the backlight comes on so no noise flashes. */
    for (unsigned i = 0; i < CP_LCD_W * CP_STRIP_H; i++) strip[i] = 0;
    for (int y = 0; y < CP_LCD_H; y += CP_STRIP_H)
        cp_lcd_blit(0, y, CP_LCD_W, y + CP_STRIP_H > CP_LCD_H ? CP_LCD_H - y : CP_STRIP_H, strip);
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));
    const gpio_config_t bl = {.pin_bit_mask = 1ULL << PIN_BL, .mode = GPIO_MODE_OUTPUT};
    ESP_ERROR_CHECK(gpio_config(&bl));
    gpio_set_level(PIN_BL, 1);
}

/* Landscape with the keyboard below; flipped turns the image 180 degrees. */
void cp_lcd_flip(bool flipped) {
    esp_lcd_panel_swap_xy(panel, true);
    esp_lcd_panel_mirror(panel, !flipped, flipped);
    esp_lcd_panel_set_gap(panel, 40, flipped ? 52 : 53);
}

uint16_t *cp_lcd_strip(void) { return strip; }

void cp_lcd_blit(int x, int y, int w, int h, const uint16_t *pixels) {
    if (w <= 0 || h <= 0) return;
    esp_lcd_panel_draw_bitmap(panel, x, y, x + w, y + h, pixels);
    /* One transfer in flight: the caller reuses its buffer immediately. */
    xSemaphoreTake(done, pdMS_TO_TICKS(100));
}
