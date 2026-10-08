/* M5Stack Cardputer ADV: on-device spectrum/waterfall UI for the S3 backend. */
#pragma once
#include <stdbool.h>
#include <stdint.h>

#include "sdr_local.h"
#include "sdr_view.h"

/* Board drivers. */
#define CP_LCD_W 240
#define CP_LCD_H 135
void cp_lcd_init(void);
void cp_lcd_flip(bool flipped);
/* Blits w*h big-endian RGB565 pixels; the buffer may be reused on return. */
void cp_lcd_blit(int x, int y, int w, int h, const uint16_t *pixels);
uint16_t *cp_lcd_strip(void); /* DMA scratch of CP_LCD_W * CP_STRIP_H pixels */
#define CP_STRIP_H 16

/* Speaker: ES8311 codec + I2S, 31.25 kHz mono (both channels). The DMA ring
 * loops on its own, so cp_audio_put works with interrupts masked (core 1). */
#define CP_AUDIO_RATE 31250
bool cp_audio_init(void);
bool cp_audio_start(void);
void cp_audio_stop(void);
void cp_audio_put(int32_t sample); /* IRAM; clipped to int16 */
void cp_audio_pause(void); /* between runs: silence the already played part of the ring */
void cp_audio_tone(unsigned hz, unsigned amplitude); /* refill the ring with a tone */

/* Listen mode (listen.c): continuous FM/AM demodulation into the speaker.
 * The screen holds still during an IQ run; any key ends the run, takes
 * effect, and the run restarts. */
bool cp_listen_enter(unsigned khz); /* false: speaker unavailable */
bool cp_listen_active(void);
void cp_listen_step(bool host_active);

/* Shared with listen.c (ui.c). */
#define cp_fill sv_fill
#define cp_text sv_text
void cp_ui_help(const char *const *lines, int n);
void cp_ui_frozen(unsigned listen_khz); /* last spectrum, marker at the listen frequency */
void cp_ui_resume(void);                /* listen mode ended: back to the live spectrum */
#define CP_PANEL_Y 68                   /* listen panel replaces the waterfall */

bool cp_kbd_pending(void); /* IRAM; keyboard interrupt line asserted */
bool cp_kbd_irq_ok(void);  /* the line was seen working */

void cp_kbd_init(void);
void *cp_i2c_bus(void); /* shared system I2C bus (i2c_master_bus_handle_t) */
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
#define cp_rgb sv_rgb
