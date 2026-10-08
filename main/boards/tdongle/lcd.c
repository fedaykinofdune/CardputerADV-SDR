/* T-Dongle S3 ST7735 (0.96", 80x160 visible inside 132x162 RAM), used
 * landscape on SPI2. Only panel IO is needed: init list, window, RAMWR. */
#include "tdongle.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define PIN_MOSI 3
#define PIN_SCLK 5
#define PIN_CS 4
#define PIN_DC 2
#define PIN_RST 1
#define PIN_BL 38 /* active low */
/* Visible window inside the controller RAM, in landscape (MV) addressing. */
#define GAP_X 1
#define GAP_Y 26
/* MADCTL: MV (row/column exchange) + BGR; MY or MX picks which end is up. */
#define MADCTL_NORMAL 0xA8
#define MADCTL_FLIPPED 0x68

static esp_lcd_panel_io_handle_t io;
static SemaphoreHandle_t done;
static uint16_t *strip;

static bool on_done(esp_lcd_panel_io_handle_t h, esp_lcd_panel_io_event_data_t *e, void *ctx) {
    (void)h; (void)e; (void)ctx;
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(done, &woken);
    return woken == pdTRUE;
}

static void cmd(uint8_t c, const uint8_t *data, size_t n) { esp_lcd_panel_io_tx_param(io, c, data, n); }

void td_lcd_init(void) {
    done = xSemaphoreCreateBinary();
    strip = heap_caps_malloc(TD_LCD_W * TD_STRIP_H * 2, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    assert(strip);
    /* Backlight off until the first frame is drawn. */
    const gpio_config_t out = {.pin_bit_mask = (1ULL << PIN_BL) | (1ULL << PIN_RST), .mode = GPIO_MODE_OUTPUT};
    ESP_ERROR_CHECK(gpio_config(&out));
    gpio_set_level(PIN_BL, 1);
    const spi_bus_config_t bus = {
        .mosi_io_num = PIN_MOSI, .miso_io_num = -1, .sclk_io_num = PIN_SCLK,
        .quadwp_io_num = -1, .quadhd_io_num = -1,
        .max_transfer_sz = TD_LCD_W * TD_STRIP_H * 2,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));
    const esp_lcd_panel_io_spi_config_t io_cfg = {
        .cs_gpio_num = PIN_CS, .dc_gpio_num = PIN_DC, .spi_mode = 0,
        .pclk_hz = 40 * 1000 * 1000, .trans_queue_depth = 4,
        .on_color_trans_done = on_done, .lcd_cmd_bits = 8, .lcd_param_bits = 8,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_cfg, &io));
    gpio_set_level(PIN_RST, 0);
    esp_rom_delay_us(10000);
    gpio_set_level(PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));
    /* LilyGO's ST7735 init (frame rate, power, gamma) for this panel. */
    static const struct { uint8_t cmd, n, data[16]; uint16_t delay_ms; } init[] = {
        {0x11, 0, {0}, 120},                                     /* SLPOUT */
        {0xB1, 3, {0x05, 0x3A, 0x3A}, 0},                        /* FRMCTR1 */
        {0xB2, 3, {0x05, 0x3A, 0x3A}, 0},                        /* FRMCTR2 */
        {0xB3, 6, {0x05, 0x3A, 0x3A, 0x05, 0x3A, 0x3A}, 0},      /* FRMCTR3 */
        {0xB4, 1, {0x03}, 0},                                    /* INVCTR */
        {0xC0, 3, {0x62, 0x02, 0x04}, 0},                        /* PWCTR1 */
        {0xC1, 1, {0xC0}, 0},                                    /* PWCTR2 */
        {0xC2, 2, {0x0D, 0x00}, 0},                              /* PWCTR3 */
        {0xC3, 2, {0x8D, 0x6A}, 0},                              /* PWCTR4 */
        {0xC4, 2, {0x8D, 0xEE}, 0},                              /* PWCTR5 */
        {0xC5, 1, {0x0E}, 0},                                    /* VMCTR1 */
        {0x21, 0, {0}, 0},                                       /* INVON */
        {0x3A, 1, {0x05}, 0},                                    /* COLMOD: 16 bit */
        {0xE0, 16, {0x10, 0x0E, 0x02, 0x03, 0x0E, 0x07, 0x02, 0x07, 0x0A, 0x12, 0x27, 0x37, 0x00, 0x0D, 0x0E, 0x10}, 0},
        {0xE1, 16, {0x10, 0x0E, 0x03, 0x03, 0x0F, 0x06, 0x02, 0x08, 0x0A, 0x13, 0x26, 0x36, 0x00, 0x0D, 0x0E, 0x10}, 0},
        {0x36, 1, {MADCTL_NORMAL}, 0},                           /* MADCTL */
        {0x13, 0, {0}, 10},                                      /* NORON */
    };
    for (unsigned i = 0; i < sizeof(init) / sizeof(init[0]); i++) {
        cmd(init[i].cmd, init[i].n ? init[i].data : NULL, init[i].n);
        if (init[i].delay_ms) vTaskDelay(pdMS_TO_TICKS(init[i].delay_ms));
    }
    /* Clear before the display and backlight come on so no noise flashes. */
    for (unsigned i = 0; i < TD_LCD_W * TD_STRIP_H; i++) strip[i] = 0;
    for (int y = 0; y < TD_LCD_H; y += TD_STRIP_H)
        td_lcd_blit(0, y, TD_LCD_W, y + TD_STRIP_H > TD_LCD_H ? TD_LCD_H - y : TD_STRIP_H, strip);
    cmd(0x29, NULL, 0); /* DISPON */
    vTaskDelay(pdMS_TO_TICKS(20));
}

void td_lcd_backlight(bool on) { gpio_set_level(PIN_BL, on ? 0 : 1); }

/* The visible window is centred in RAM, so both orientations share the gap. */
void td_lcd_flip(bool flipped) {
    const uint8_t m = flipped ? MADCTL_FLIPPED : MADCTL_NORMAL;
    cmd(0x36, &m, 1);
}

uint16_t *td_lcd_strip(void) { return strip; }

void td_lcd_blit(int x, int y, int w, int h, const uint16_t *pixels) {
    if (w <= 0 || h <= 0) return;
    const int x0 = x + GAP_X, x1 = x + w - 1 + GAP_X, y0 = y + GAP_Y, y1 = y + h - 1 + GAP_Y;
    const uint8_t ca[4] = {(uint8_t)(x0 >> 8), (uint8_t)x0, (uint8_t)(x1 >> 8), (uint8_t)x1};
    const uint8_t ra[4] = {(uint8_t)(y0 >> 8), (uint8_t)y0, (uint8_t)(y1 >> 8), (uint8_t)y1};
    cmd(0x2A, ca, 4); /* CASET */
    cmd(0x2B, ra, 4); /* RASET */
    esp_lcd_panel_io_tx_color(io, 0x2C, pixels, (size_t)w * h * 2);
    /* One transfer in flight: the caller reuses its buffer immediately. */
    xSemaphoreTake(done, pdMS_TO_TICKS(100));
}
