/* T-Dongle S3 BOOT button and APA102 LED. */
#include "tdongle.h"
#include "driver/gpio.h"

#define PIN_BUTTON 0 /* BOOT, external pull-up; held at reset it enters download mode */
#define PIN_LED_DATA 40
#define PIN_LED_CLK 39

void td_button_init(void) {
    const gpio_config_t in = {.pin_bit_mask = 1ULL << PIN_BUTTON, .mode = GPIO_MODE_INPUT,
                              .pull_up_en = GPIO_PULLUP_ENABLE};
    gpio_config(&in);
}

bool td_button_down(void) { return gpio_get_level(PIN_BUTTON) == 0; }

static void led_byte(uint8_t v) {
    for (int i = 7; i >= 0; i--) {
        gpio_set_level(PIN_LED_DATA, (v >> i) & 1);
        gpio_set_level(PIN_LED_CLK, 1);
        gpio_set_level(PIN_LED_CLK, 0);
    }
}

void td_led_init(void) {
    const gpio_config_t out = {.pin_bit_mask = (1ULL << PIN_LED_DATA) | (1ULL << PIN_LED_CLK),
                               .mode = GPIO_MODE_OUTPUT};
    gpio_config(&out);
    gpio_set_level(PIN_LED_CLK, 0);
    td_led_set(0, 0, 0, 0);
}

/* One APA102: start frame, 0b111 + 5-bit brightness, blue, green, red, end frame. */
void td_led_set(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness) {
    static uint32_t last = ~0u;
    const uint32_t now = (uint32_t)r | (uint32_t)g << 8 | (uint32_t)b << 16 | (uint32_t)(brightness & 31) << 24;
    if (now == last) return;
    last = now;
    for (int i = 0; i < 4; i++) led_byte(0x00);
    led_byte(0xE0 | (brightness & 31));
    led_byte(b);
    led_byte(g);
    led_byte(r);
    for (int i = 0; i < 4; i++) led_byte(0xFF);
}
