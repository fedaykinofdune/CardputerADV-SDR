/* M5Stack Cardputer ADV: on-device spectrum/waterfall UI for the S3 backend. */
#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Receiver services (targets/esp32s3/receiver.c). */
const uint32_t *sdr_local_capture(unsigned n, unsigned divider);
unsigned sdr_local_freq(void);
void sdr_local_tune(unsigned mhz);
int sdr_local_gain(void); /* -1: hardware AGC */
unsigned sdr_local_gain_max(void);
void sdr_local_set_gain(int code);
unsigned sdr_local_bandwidth(void); /* MHz, 0: automatic */
void sdr_local_set_bandwidth(unsigned mhz);

/* UI entry points. cardputer_init() runs before GPIO discovery so the board
 * pins stay reserved; cardputer_step() runs one frame in the idle slot and
 * returns false when it did nothing (the caller then yields). */
void cardputer_init(void);
bool cardputer_step(bool host_active);

/* Board drivers. */
#define CP_LCD_W 240
#define CP_LCD_H 135
void cp_lcd_init(void);
void cp_lcd_flip(bool flipped);
/* Blits w*h big-endian RGB565 pixels; the buffer may be reused on return. */
void cp_lcd_blit(int x, int y, int w, int h, const uint16_t *pixels);
uint16_t *cp_lcd_strip(void); /* DMA scratch of CP_LCD_W * CP_STRIP_H pixels */
#define CP_STRIP_H 16

void cp_kbd_init(void);
/* Returns the next pressed key as ASCII (or CP_KEY_*), 0 when none. */
int cp_kbd_read(void);
#define CP_KEY_ENTER '\n'
#define CP_KEY_BACKSPACE '\b'
#define CP_KEY_ESC 0x1b
#define CP_KEY_TAB '\t'
#define CP_KEY_UP 0x100
#define CP_KEY_DOWN 0x101
#define CP_KEY_LEFT 0x102
#define CP_KEY_RIGHT 0x103
#define CP_KEY_DEL 0x104

/* Big-endian RGB565 for the ST7789 byte stream. */
static inline uint16_t cp_rgb(unsigned r, unsigned g, unsigned b) {
    uint16_t c = (uint16_t)(((r & 0xf8) << 8) | ((g & 0xfc) << 3) | (b >> 3));
    return (uint16_t)((c >> 8) | (c << 8));
}
