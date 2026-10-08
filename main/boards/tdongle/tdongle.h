/* LilyGO T-Dongle S3: on-device spectrum UI for the S3 backend on a 160x80
 * ST7735 display, one button (BOOT) and an APA102 RGB LED. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "sdr_local.h"
#include "sdr_view.h"

/* ST7735, landscape. */
#define TD_LCD_W 160
#define TD_LCD_H 80
#define TD_STRIP_H 16
void td_lcd_init(void);
void td_lcd_flip(bool flipped);
/* Blits w*h big-endian RGB565 pixels; the buffer may be reused on return. */
void td_lcd_blit(int x, int y, int w, int h, const uint16_t *pixels);
uint16_t *td_lcd_strip(void); /* DMA scratch of TD_LCD_W * TD_STRIP_H pixels */
void td_lcd_backlight(bool on);

/* BOOT button (GPIO0, active low). */
void td_button_init(void);
bool td_button_down(void);

/* APA102 LED: 8-bit colour, brightness 0..31 (0 = off). */
void td_led_init(void);
void td_led_set(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness);
