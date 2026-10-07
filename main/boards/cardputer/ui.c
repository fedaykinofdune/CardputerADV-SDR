/* Cardputer ADV on-device receiver: spectrum + waterfall from burst captures.
 *
 * Each frame takes one snapshot (SEGMENTS x FFT_N samples), averages the
 * Hann-windowed FFT powers (Welch) and draws them. The UI only runs in the
 * command loop's idle slot and pauses while a USB host holds the serial
 * lease, so the browser viewer and host tools keep working unchanged.
 * Listen mode (listen.c) takes over the loop while it plays audio. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cardputer.h"
#include "font5x7.h"
#include "esp_private/esp_gpio_reserve.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "ring_capture.h"

#define FFT_N 256
#define FFT_LOG2 8
#define SEGMENTS 16
#define BIN0 ((FFT_N - CP_LCD_W) / 2) /* drop the filtered band edges */

/* Screen layout (240 x 135). */
#define STATUS_Y 0
#define STATUS_H 11
#define SPEC_Y 11
#define SPEC_H 48
#define AXIS_Y 59
#define AXIS_H 9
#define WF_Y 68
#define WF_H (CP_LCD_H - WF_Y)

#define FRAME_US 33000
#define KEY_US 15000
#define SAVE_US 3000000

static const struct { unsigned divider, msps; } spans[] = {{0, 80}, {1, 40}, {6, 16}};
static const unsigned steps[] = {1, 2, 5, 10, 20};

/* Persistent settings. */
static struct {
    uint32_t freq;
    uint8_t span, step, flip, dc_fix, avg;
    int8_t gain; /* -1: hardware AGC */
    int8_t ref_db, range_db;
} cfg = {2437, 0, 3, 0, 1, 2, -1, -20, 70};

static bool ok, paused, peak_hold = true, marker, help = true, dirty_axis = true, auto_ref = true;
static bool host_shown, help_drawn, sniff;
static int sniff_cycles = -1;
static const char *note; /* one-off status message */
static int64_t note_until;
static unsigned listen_mark; /* kHz, 0: none */
static int64_t next_frame, next_key, save_at;
static float *win, *re, *im, *tw_re, *tw_im, *acc, *spec, *peak;
static uint8_t *wf;
static unsigned wf_top; /* ring index of the newest waterfall row */
static uint16_t lut[256];
static char status_line[48], entry[8];
static enum { MODE_RUN, MODE_FREQ, MODE_QUIT } mode;

/* ---------- drawing helpers into the DMA strip ---------- */
void cp_fill(uint16_t *buf, int w, int h, uint16_t c) {
    for (int i = 0; i < w * h; i++) buf[i] = c;
}
/* 5x7 glyphs in a 6x8 cell, scaled up by an integer factor. */
void cp_text(uint16_t *buf, int w, int h, int x, int y, const char *s, uint16_t fg, int scale) {
    for (; *s; s++, x += 6 * scale) {
        unsigned ch = (unsigned char)*s;
        if (ch < 0x20 || ch > 0x7e) ch = '?';
        if (y + 8 * scale <= 0 || y >= h) continue;
        for (int cx = 0; cx < 5 * scale; cx++) {
            uint8_t col = cp_font5x7[ch - 0x20][cx / scale];
            for (int cy = 0; cy < 8 * scale; cy++) {
                int px = x + cx, py = y + cy;
                if ((col >> (cy / scale)) & 1 && px >= 0 && px < w && py >= 0 && py < h) buf[py * w + px] = fg;
            }
        }
    }
}
#define fill cp_fill
static void text(uint16_t *buf, int w, int h, int x, int y, const char *s, uint16_t fg) {
    cp_text(buf, w, h, x, y, s, fg, 1);
}
static int text_w(const char *s) { return (int)strlen(s) * 6; }

/* Black - blue - cyan - yellow - red - white. */
static void build_lut(void) {
    static const uint8_t stops[][3] = {{0, 0, 0}, {0, 0, 140}, {0, 170, 220}, {240, 230, 0}, {255, 40, 0}, {255, 255, 255}};
    for (int i = 0; i < 256; i++) {
        float t = i / 255.0f * 5;
        int s = (int)t;
        if (s > 4) s = 4;
        float f = t - s;
        lut[i] = cp_rgb((unsigned)(stops[s][0] + (stops[s + 1][0] - stops[s][0]) * f),
                        (unsigned)(stops[s][1] + (stops[s + 1][1] - stops[s][1]) * f),
                        (unsigned)(stops[s][2] + (stops[s + 1][2] - stops[s][2]) * f));
    }
}

/* ---------- DSP ---------- */
static void fft_init(void) {
    float sum = 0;
    for (int n = 0; n < FFT_N; n++) {
        win[n] = 0.5f - 0.5f * cosf(2 * (float)M_PI * n / FFT_N);
        sum += win[n];
    }
    /* Normalise so a full-scale tone reads 0 dBFS (10-bit samples). */
    for (int n = 0; n < FFT_N; n++) win[n] /= sum * 512.0f;
    for (int k = 0; k < FFT_N / 2; k++) {
        tw_re[k] = cosf(2 * (float)M_PI * k / FFT_N);
        tw_im[k] = -sinf(2 * (float)M_PI * k / FFT_N);
    }
}
static void fft(void) {
    for (unsigned i = 1, j = 0; i < FFT_N; i++) {
        unsigned bit = FFT_N >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) {
            float t = re[i]; re[i] = re[j]; re[j] = t;
            t = im[i]; im[i] = im[j]; im[j] = t;
        }
    }
    for (unsigned len = 2; len <= FFT_N; len <<= 1) {
        unsigned half = len >> 1, stride = FFT_N / len;
        for (unsigned i = 0; i < FFT_N; i += len)
            for (unsigned k = 0; k < half; k++) {
                float wr = tw_re[k * stride], wi = tw_im[k * stride];
                unsigned a = i + k, b = a + half;
                float tr = re[b] * wr - im[b] * wi, ti = re[b] * wi + im[b] * wr;
                re[b] = re[a] - tr; im[b] = im[a] - ti;
                re[a] += tr; im[a] += ti;
            }
    }
}
static inline int s10(uint32_t v) { return (int)(v << 22) >> 22; }

/* One capture -> spec[] in dBFS per display column. */
static bool measure(void) {
    const uint32_t *w = sdr_local_capture(FFT_N * SEGMENTS, spans[cfg.span].divider);
    if (!w) return false;
    memset(acc, 0, FFT_N * sizeof(float));
    for (int s = 0; s < SEGMENTS; s++) {
        const uint32_t *p = w + s * FFT_N;
        /* The LO leaks a DC offset; removing each segment's mean takes it
         * out without notching real signals at the centre frequency. */
        int si = 0, sq = 0;
        if (cfg.dc_fix)
            for (int n = 0; n < FFT_N; n++) {si += s10(p[n]); sq += s10(p[n] >> 10);}
        float mi = (float)si / FFT_N, mq = (float)sq / FFT_N;
        for (int n = 0; n < FFT_N; n++) {
            /* Conjugate: on the S3, RF above the LO arrives at negative
             * frequency (the web viewer and bridge do the same). */
            re[n] = (s10(p[n]) - mi) * win[n];
            im[n] = (mq - s10(p[n] >> 10)) * win[n];
        }
        fft();
        for (int k = 0; k < FFT_N; k++) acc[k] += re[k] * re[k] + im[k] * im[k];
    }
    static const float alpha[] = {1.0f, 0.5f, 0.3f, 0.15f};
    float a = alpha[cfg.avg & 3];
    for (int x = 0; x < CP_LCD_W; x++) {
        int k = (x + BIN0 + FFT_N / 2) & (FFT_N - 1); /* fftshift */
        float db = 10 * log10f(acc[k] / SEGMENTS + 1e-12f);
        spec[x] = isfinite(spec[x]) && a < 1 ? spec[x] + a * (db - spec[x]) : db;
        if (!peak_hold || !isfinite(peak[x]) || spec[x] > peak[x]) peak[x] = spec[x];
        else peak[x] -= 0.15f; /* slow decay */
    }
    return true;
}

static void autoscale(void) {
    float lo = 0, hi = -200;
    float sorted[CP_LCD_W];
    memcpy(sorted, spec, sizeof(sorted));
    /* Median-ish noise floor via partial selection. */
    for (int i = 0; i <= CP_LCD_W / 2; i++)
        for (int j = i + 1; j < CP_LCD_W; j++)
            if (sorted[j] < sorted[i]) {float t = sorted[i]; sorted[i] = sorted[j]; sorted[j] = t;}
    lo = sorted[CP_LCD_W / 2];
    for (int x = 0; x < CP_LCD_W; x++) if (spec[x] > hi) hi = spec[x];
    int ref = (int)ceilf((hi + 6) / 5) * 5, bottom = (int)floorf((lo - 8) / 5) * 5;
    if (ref - bottom < 30) bottom = ref - 30;
    if (ref > 10) ref = 10;
    if (ref < -110) ref = -110;
    cfg.ref_db = (int8_t)ref;
    cfg.range_db = (int8_t)(ref - bottom > 100 ? 100 : ref - bottom);
}

/* ---------- screens ---------- */
static float column_mhz(int x) {
    return cfg.freq + (x + BIN0 - FFT_N / 2) * (float)spans[cfg.span].msps / FFT_N;
}

static void draw_status(bool host) {
    char line[48], gain[8];
    if (cfg.gain < 0) snprintf(gain, sizeof(gain), "AGC");
    else snprintf(gain, sizeof(gain), "G%d", cfg.gain);
    if (host) snprintf(line, sizeof(line), "%uMHz  USB host in control", (unsigned)cfg.freq);
    else if (mode == MODE_FREQ) snprintf(line, sizeof(line), "Tune to: %s_ MHz  [Enter]", entry);
    else if (mode == MODE_QUIT) snprintf(line, sizeof(line), "Reboot (Launcher)? y/n");
    else if (note && esp_timer_get_time() < note_until) snprintf(line, sizeof(line), "%s", note);
    else snprintf(line, sizeof(line), "%uMHz %uM %s st%u%s%s%s", (unsigned)cfg.freq,
                  spans[cfg.span].msps, gain, steps[cfg.step], paused ? " HOLD" : "",
                  sdr_local_bandwidth() ? " BW" : "", sniff ? " SNIFF" : "");
    if (!strcmp(line, status_line)) return;
    strcpy(status_line, line);
    uint16_t *b = cp_lcd_strip();
    fill(b, CP_LCD_W, STATUS_H, cp_rgb(20, 24, 40));
    text(b, CP_LCD_W, STATUS_H, 2, 2, line, mode != MODE_RUN ? cp_rgb(255, 210, 0) : cp_rgb(230, 240, 255));
    if (mode == MODE_RUN && !host) text(b, CP_LCD_W, STATUS_H, CP_LCD_W - 6 * 3 - 2, 2, "?=h", cp_rgb(120, 130, 160));
    cp_lcd_blit(0, STATUS_Y, CP_LCD_W, STATUS_H, b);
}

static void draw_axis(void) {
    uint16_t *b = cp_lcd_strip();
    fill(b, CP_LCD_W, AXIS_H, 0);
    uint16_t c = cp_rgb(160, 170, 190);
    char s[16];
    for (int i = 0; i < 3; i++) {
        int x = i * (CP_LCD_W - 1) / 2;
        snprintf(s, sizeof(s), "%.0f", column_mhz(x));
        int tx = i == 0 ? 0 : i == 1 ? x - text_w(s) / 2 : CP_LCD_W - text_w(s);
        text(b, CP_LCD_W, AXIS_H, tx, 1, s, c);
    }
    snprintf(s, sizeof(s), "ref%d", cfg.ref_db);
    text(b, CP_LCD_W, AXIS_H, CP_LCD_W / 4 - text_w(s) / 2, 1, s, cp_rgb(90, 110, 90));
    snprintf(s, sizeof(s), "%ddB", cfg.range_db);
    text(b, CP_LCD_W, AXIS_H, 3 * CP_LCD_W / 4 - text_w(s) / 2, 1, s, cp_rgb(90, 110, 90));
    cp_lcd_blit(0, AXIS_Y, CP_LCD_W, AXIS_H, b);
}

static int db_to_y(float db) {
    float t = (cfg.ref_db - db) / cfg.range_db;
    int y = (int)(t * (SPEC_H - 1));
    return y < 0 ? 0 : y > SPEC_H - 1 ? SPEC_H - 1 : y;
}

static void draw_spectrum(void) {
    uint16_t grid = cp_rgb(30, 36, 50), fillc = cp_rgb(0, 70, 40), line = cp_rgb(60, 255, 120);
    uint16_t pk = cp_rgb(255, 200, 0), mk = cp_rgb(255, 80, 200);
    int mx = 0;
    for (int x = 1; x < CP_LCD_W; x++) if (spec[x] > spec[mx]) mx = x;
    /* Listen frequency, when listen mode is on. */
    int lx = listen_mark ? (int)lroundf((listen_mark / 1000.0f - cfg.freq) * FFT_N / spans[cfg.span].msps) + FFT_N / 2 - BIN0 : -1;
    uint16_t *b = cp_lcd_strip();
    for (int y0 = 0; y0 < SPEC_H; y0 += CP_STRIP_H) {
        int h = SPEC_H - y0 < CP_STRIP_H ? SPEC_H - y0 : CP_STRIP_H;
        for (int yy = 0; yy < h; yy++) {
            int y = y0 + yy;
            bool hgrid = y % 12 == 0;
            for (int x = 0; x < CP_LCD_W; x++)
                b[yy * CP_LCD_W + x] = hgrid || x % 60 == 0 || x == CP_LCD_W / 2 ? grid : 0;
        }
        for (int x = 0; x < CP_LCD_W; x++) {
            int y = db_to_y(spec[x]);
            int prev = x ? db_to_y(spec[x - 1]) : y;
            int top = y < prev ? y : prev, bot = y < prev ? prev : y;
            for (int yy = 0; yy < h; yy++) {
                int yl = y0 + yy;
                uint16_t *p = &b[yy * CP_LCD_W + x];
                if (yl > bot) *p = fillc;
                if (yl >= top && yl <= bot) *p = line;
            }
            if (peak_hold) {
                int py = db_to_y(peak[x]) - y0;
                if (py >= 0 && py < h) b[py * CP_LCD_W + x] = pk;
            }
            if (marker && x == mx)
                for (int yy = 0; yy < h; yy++) if ((y0 + yy) % 3 == 0) b[yy * CP_LCD_W + x] = mk;
            if (x == lx)
                for (int yy = 0; yy < h; yy++) if ((y0 + yy) % 4 < 2) b[yy * CP_LCD_W + x] = cp_rgb(255, 40, 40);
        }
        if (marker && y0 == 0) {
            char s[24];
            snprintf(s, sizeof(s), "%.1f %.0fdB", column_mhz(mx), spec[mx]);
            int tx = mx + 4 + text_w(s) > CP_LCD_W ? mx - 4 - text_w(s) : mx + 4;
            text(b, CP_LCD_W, h, tx, 2, s, mk);
        }
        cp_lcd_blit(0, SPEC_Y + y0, CP_LCD_W, h, b);
    }
}

static void push_waterfall(void) {
    wf_top = (wf_top + WF_H - 1) % WF_H;
    uint8_t *row = wf + wf_top * CP_LCD_W;
    float lo = cfg.ref_db - cfg.range_db;
    for (int x = 0; x < CP_LCD_W; x++) {
        float t = (spec[x] - lo) / cfg.range_db * 255;
        row[x] = t < 0 ? 0 : t > 255 ? 255 : (uint8_t)t;
    }
}

static void draw_waterfall(void) {
    uint16_t *b = cp_lcd_strip();
    for (int y0 = 0; y0 < WF_H; y0 += CP_STRIP_H) {
        int h = WF_H - y0 < CP_STRIP_H ? WF_H - y0 : CP_STRIP_H;
        for (int yy = 0; yy < h; yy++) {
            const uint8_t *row = wf + ((wf_top + y0 + yy) % WF_H) * CP_LCD_W;
            for (int x = 0; x < CP_LCD_W; x++) b[yy * CP_LCD_W + x] = lut[row[x]];
        }
        cp_lcd_blit(0, WF_Y + y0, CP_LCD_W, h, b);
    }
}

static const char *const help_lines[] = {
    "ESP-SDR  Cardputer ADV",
    ", /  tune -/+ step   ; .  step",
    "f    type MHz   s  span 80/40/16",
    "1-9 0  Wi-Fi ch 1-9, 0=ch13",
    "g - =  AGC / manual gain",
    "b    RF filter   r  autoscale",
    "[ ]  ref level   a  averaging",
    "m    marker      p  peak hold",
    "d    DC fix      space  hold",
    "l    LISTEN on speaker (marker)",
    "n    sniffer tone  o flip  q quit",
    "USB host apps still work.",
};

void cp_ui_help(const char *const *lines, int n) {
    uint16_t *b = cp_lcd_strip();
    for (int y0 = 0; y0 < CP_LCD_H; y0 += CP_STRIP_H) {
        int h = CP_LCD_H - y0 < CP_STRIP_H ? CP_LCD_H - y0 : CP_STRIP_H;
        fill(b, CP_LCD_W, h, cp_rgb(10, 12, 24));
        for (int i = 0; i < n; i++)
            text(b, CP_LCD_W, h, 3, 3 + i * 11 - y0, lines[i], i ? cp_rgb(220, 230, 240) : cp_rgb(60, 255, 120));
        cp_lcd_blit(0, y0, CP_LCD_W, h, b);
    }
}
static void draw_help(void) { cp_ui_help(help_lines, sizeof(help_lines) / sizeof(help_lines[0])); }

/* ---------- input ---------- */
static void changed(void) { save_at = esp_timer_get_time() + SAVE_US; }
static void retune(unsigned mhz) {
    if (mhz < 100 || mhz > 6000) return;
    cfg.freq = mhz;
    sdr_local_tune(mhz);
    dirty_axis = true;
    for (int x = 0; x < CP_LCD_W; x++) peak[x] = NAN;
    changed();
}
static void restart_view(void) { status_line[0] = 0; dirty_axis = true; }

/* ---------- listen mode and sniffer hooks ---------- */
void cp_ui_frozen(unsigned listen_khz) {
    listen_mark = listen_khz;
    draw_axis();
    draw_spectrum();
}

void cp_ui_resume(void) {
    listen_mark = 0;
    sdr_local_tune(cfg.freq);
    memset(wf, 0, WF_H * CP_LCD_W); /* the IQ run used the ring banks */
    restart_view();
    next_frame = 0;
    if (sniff && !cp_audio_start()) sniff = false;
    sniff_cycles = -1;
}

static void say(const char *s) { note = s; note_until = esp_timer_get_time() + 3000000; }

static void listen(void) {
    int mx = 0;
    for (int x = 1; x < CP_LCD_W; x++) if (spec[x] > spec[mx]) mx = x;
    float mhz = marker && isfinite(spec[mx]) ? column_mhz(mx) : (float)cfg.freq;
    if (!cp_listen_enter((unsigned)lroundf(mhz * 1000))) say("Speaker init failed");
    status_line[0] = 0;
}

/* Pitch follows the strongest signal's height above the noise floor; a
 * whole number of cycles per audio buffer keeps it click-free. */
static void sniff_update(void) {
    uint16_t hist[64] = {0}; /* 2 dB bins from -140 dBFS */
    float top = -200;
    for (int x = 0; x < CP_LCD_W; x++) {
        if (!isfinite(spec[x])) return;
        int bin = (int)((spec[x] + 140) / 2);
        hist[bin < 0 ? 0 : bin > 63 ? 63 : bin]++;
        if (spec[x] > top) top = spec[x];
    }
    int bin = 0;
    for (unsigned seen = 0; bin < 63 && (seen += hist[bin]) < CP_LCD_W / 2; bin++) {}
    float snr = top - (bin * 2 - 140);
    int cycles = snr < 6 ? 0 : 2 + (int)((snr - 6) / 3); /* 130 Hz per cycle per buffer */
    if (cycles > 23) cycles = 23;
    if (cycles != sniff_cycles) {
        sniff_cycles = cycles;
        cp_audio_tone((unsigned)cycles * CP_AUDIO_RATE / 240, 6000);
    }
}

static void key(int k) {
    if (help) {help = false; restart_view(); return;}
    if (mode == MODE_FREQ) {
        size_t n = strlen(entry);
        if (k >= '0' && k <= '9' && n < 4) {entry[n] = (char)k; entry[n + 1] = 0;}
        else if (k == CP_KEY_BACKSPACE && n) entry[n - 1] = 0;
        else if (k == CP_KEY_ENTER) {if (n) retune((unsigned)atoi(entry)); mode = MODE_RUN;}
        else if (k == CP_KEY_ESC || k == '`') mode = MODE_RUN;
        return;
    }
    if (mode == MODE_QUIT) {
        if (k == 'y' || k == 'Y') {
            nvs_handle_t h;
            if (nvs_open("cardputer", NVS_READWRITE, &h) == ESP_OK) {
                nvs_set_blob(h, "cfg", &cfg, sizeof(cfg)); nvs_commit(h); nvs_close(h);
            }
            esp_restart();
        }
        mode = MODE_RUN;
        return;
    }
    unsigned step = steps[cfg.step];
    switch (k) {
    case ',': case CP_KEY_LEFT: retune(cfg.freq - step); break;
    case '/': case CP_KEY_RIGHT: retune(cfg.freq + step); break;
    case ';': case CP_KEY_UP: cfg.step = (uint8_t)((cfg.step + 1) % 5); changed(); break;
    case '.': case CP_KEY_DOWN: cfg.step = (uint8_t)((cfg.step + 4) % 5); changed(); break;
    case 'f': mode = MODE_FREQ; entry[0] = 0; break;
    case '1': case '2': case '3': case '4': case '5': case '6': case '7': case '8': case '9':
        retune(2412 + 5 * (unsigned)(k - '1')); break;
    case '0': retune(2472); break;
    case 's': cfg.span = (uint8_t)((cfg.span + 1) % 3); dirty_axis = true; changed();
        for (int x = 0; x < CP_LCD_W; x++) {spec[x] = NAN; peak[x] = NAN;} break;
    case 'g': cfg.gain = cfg.gain < 0 ? (int8_t)(sdr_local_gain_max() / 2) : -1;
        sdr_local_set_gain(cfg.gain); changed(); break;
    case '-': case '=': {
        int g = cfg.gain < 0 ? (int)sdr_local_gain_max() / 2 : cfg.gain;
        g += k == '=' ? 2 : -2;
        if (g < 0) g = 0;
        if (g > (int)sdr_local_gain_max()) g = (int)sdr_local_gain_max();
        cfg.gain = (int8_t)g; sdr_local_set_gain(g); changed(); break;
    }
    case 'b': sdr_local_set_bandwidth(sdr_local_bandwidth() ? 0 : 20); break;
    case 'r': auto_ref = true; break;
    case '[': cfg.ref_db = (int8_t)(cfg.ref_db - 5); dirty_axis = true; changed(); break;
    case ']': if (cfg.ref_db < 20) cfg.ref_db = (int8_t)(cfg.ref_db + 5); dirty_axis = true; changed(); break;
    case 'a': cfg.avg = (uint8_t)((cfg.avg + 1) & 3); changed(); break;
    case 'p': peak_hold = !peak_hold; break;
    case 'm': marker = !marker; break;
    case 'd': cfg.dc_fix = !cfg.dc_fix; changed(); break;
    case ' ': paused = !paused; break;
    case 'o': cfg.flip = !cfg.flip; cp_lcd_flip(cfg.flip); restart_view(); changed(); break;
    case 'h': case '?': case CP_KEY_TAB: help = true; break;
    case 'q': mode = MODE_QUIT; break;
    case 'l': listen(); break;
    case 'n':
        sniff = !sniff;
        if (sniff && !cp_audio_start()) {sniff = false; say("Speaker init failed");}
        if (!sniff) cp_audio_stop();
        sniff_cycles = -1;
        break;
    }
}

/* ---------- entry points ---------- */
void cardputer_init(void) {
    /* LCD, backlight, I2C, keyboard IRQ, SD and audio pins stay out of the
     * host-controllable GPIO set. */
    esp_gpio_reserve(BIT64(8) | BIT64(9) | BIT64(11) | BIT64(12) | BIT64(14) | BIT64(33) | BIT64(34) |
                     BIT64(35) | BIT64(36) | BIT64(37) | BIT64(38) | BIT64(39) | BIT64(40) | BIT64(41) |
                     BIT64(42) | BIT64(43) | BIT64(44) | BIT64(46));
    size_t f = FFT_N * sizeof(float);
    win = malloc(f); re = malloc(f); im = malloc(f); acc = malloc(f);
    tw_re = malloc(f / 2); tw_im = malloc(f / 2);
    spec = malloc(CP_LCD_W * sizeof(float)); peak = malloc(CP_LCD_W * sizeof(float));
    /* Heap is scarce beside the 192 KiB RF ring, so the waterfall history
     * borrows ring bank 0. Only host-driven RING/SPEC/IQS runs write the
     * banks, and the UI clears the history whenever a host session ends. */
    _Static_assert(WF_H * CP_LCD_W <= 0x10000, "waterfall exceeds one ring bank");
    wf = (uint8_t *)ring_capture_bank(0);
    memset(wf, 0, WF_H * CP_LCD_W);
    ok = win && re && im && acc && tw_re && tw_im && spec && peak && wf;
    if (!ok) return;
    nvs_handle_t h;
    size_t len = sizeof(cfg);
    if (nvs_open("cardputer", NVS_READONLY, &h) == ESP_OK) {
        typeof(cfg) saved;
        if (nvs_get_blob(h, "cfg", &saved, &len) == ESP_OK && len == sizeof(cfg) && saved.span < 3 &&
            saved.step < 5 && saved.freq >= 100 && saved.freq <= 6000 && saved.range_db >= 20)
            cfg = saved;
        nvs_close(h);
    }
    for (int x = 0; x < CP_LCD_W; x++) {spec[x] = NAN; peak[x] = NAN;}
    fft_init();
    build_lut();
    cp_lcd_init();
    cp_lcd_flip(cfg.flip);
    cp_kbd_init();
    sdr_local_tune(cfg.freq);
    if (cfg.gain >= 0) sdr_local_set_gain(cfg.gain);
    draw_help();
    help_drawn = true;
}

bool cardputer_step(bool host_active) {
    if (!ok) return false;
    if (cp_listen_active()) {cp_listen_step(host_active); return true;}
    int64_t now = esp_timer_get_time();
    if (now >= next_key) {
        next_key = now + KEY_US;
        for (int k; !cp_listen_active() && (k = cp_kbd_read());) key(k);
        if (cp_listen_active()) return true;
    }
    if (help) {
        if (!help_drawn) {draw_help(); help_drawn = true;}
        return false;
    }
    help_drawn = false;
    if (save_at && now >= save_at) {
        save_at = 0;
        nvs_handle_t h;
        if (nvs_open("cardputer", NVS_READWRITE, &h) == ESP_OK) {
            nvs_set_blob(h, "cfg", &cfg, sizeof(cfg)); nvs_commit(h); nvs_close(h);
        }
    }
    if (host_active) {
        /* The host may retune; follow it so the display stays truthful. */
        cfg.freq = sdr_local_freq();
        if (!host_shown) {
            host_shown = true; mode = MODE_RUN; status_line[0] = 0; draw_status(true);
            if (sniff) {sniff = false; cp_audio_stop();}
        }
        return false;
    }
    if (host_shown) {
        host_shown = false; cfg.freq = sdr_local_freq(); restart_view();
        memset(wf, 0, WF_H * CP_LCD_W);
    }
    if (now < next_frame) {draw_status(false); return false;}
    next_frame = now + FRAME_US;
    draw_status(false);
    if (dirty_axis) {dirty_axis = false; draw_axis();}
    if (!paused) {
        if (!measure()) return false;
        if (auto_ref && isfinite(spec[0])) {auto_ref = false; autoscale(); dirty_axis = true;}
        push_waterfall();
        if (sniff) sniff_update();
    }
    draw_spectrum();
    draw_waterfall();
    vTaskDelay(1);
    return true;
}
