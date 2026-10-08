/* T-Dongle S3 on-device receiver: three pages on a 160x80 screen, one button.
 *
 *   SPEC  spectrum + waterfall; tap zooms onto the strongest signal
 *         (80 -> 40 -> 16 MHz span, then back to the whole 2.4 GHz band)
 *   CHAN  Wi-Fi channel airtime survey (1-13) with the quietest of 1/6/11;
 *         tap resets the statistics
 *   HUNT  strongest-signal meter for finding a transmitter; tap locks onto
 *         its frequency
 *
 * Hold the button (0.5 s) for the next page, hold 2 s to turn the picture
 * upside down. The APA102 LED glows from blue (quiet) to red (strong signal).
 * Like the Cardputer UI it runs in the command loop's idle slot and stands
 * aside while a USB host holds the serial lease. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "tdongle.h"
#include "esp_private/esp_gpio_reserve.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"

#define W TD_LCD_W
#define H TD_LCD_H
#define STATUS_H 9
#define SPEC_Y STATUS_H
#define SPEC_H 34
#define AXIS_Y (SPEC_Y + SPEC_H)
#define AXIS_H 8
#define WF_Y (AXIS_Y + AXIS_H)
#define WF_H (H - WF_Y)

#define FRAME_US 40000
#define SAVE_US 3000000
#define HINT_US 3000000
#define HOLD_PAGE_US 500000
#define HOLD_FLIP_US 2000000
#define SCALE_US 1000000

#define BAND_MHZ 2442 /* 80 MHz span: 2404.5 - 2479.5 MHz, Wi-Fi channels 1-13 */
#define CHANNELS 13

enum { PAGE_SPEC, PAGE_CHAN, PAGE_HUNT, PAGES };
static const char *const page_hint[PAGES] = {"tap: zoom  hold: page", "tap: reset  hold: page", "tap: lock  hold: page"};

/* Persistent settings (NVS "tdongle"/"cfg"). */
static struct {
    uint16_t centre;
    uint8_t span, page, flip;
} cfg = {BAND_MHZ, 0, PAGE_SPEC, 0};

static bool ok, host_shown, help = true;
static int64_t next_frame, save_at, hint_until, press_at, next_scale;
static bool press_void; /* press overlapped a host session: no action */
static unsigned host_freq; /* frequency on the host screen */
static unsigned view_centre; /* view the smoothed data belongs to */
static uint8_t view_span;
static float *bins, *cols, *scratch;
static float floor_db = -90, top_db = -90;
static int top_bin;  /* strongest bin this frame (meter) */
static int peak_bin; /* strongest smoothed bin (zoom / lock target, shown peak) */
static int8_t ref_db = -20, range_db = 70;
static uint8_t *wf;
static unsigned wf_top;
static uint16_t lut[256];
static bool wf_clear = true;
/* Survey. */
static uint32_t frames, busy_frames[CHANNELS];
static int64_t survey_us, survey_last;
static float live[CHANNELS];
/* Hunt. */
static float lock_mhz, level_db = -90, level_peak = -90;

/* ---------- drawing ---------- */
#define rgb sv_rgb
static void text(uint16_t *b, int h, int x, int y, const char *s, uint16_t c) { sv_text(b, W, h, x, y, s, c, 1); }

static void draw_status(const char *left, const char *right, uint16_t c) {
    uint16_t *b = td_lcd_strip();
    sv_fill(b, W, STATUS_H, rgb(20, 24, 40));
    text(b, STATUS_H, 1, 1, left, c);
    if (right) text(b, STATUS_H, W - 1 - sv_text_w(right, 1), 1, right, rgb(120, 130, 160));
    td_lcd_blit(0, 0, W, STATUS_H, b);
}

static int db_to_y(float db, int h) {
    float t = (ref_db - db) / range_db;
    int y = (int)(t * (h - 1));
    return y < 0 ? 0 : y > h - 1 ? h - 1 : y;
}

/* Hint text over the bottom rows of a strip that ends at the screen bottom. */
static void hint_overlay(uint16_t *b, int y0, int h) {
    if (esp_timer_get_time() >= hint_until) return;
    int y = H - 8 - y0;
    if (y + 8 <= 0 || y >= h) return;
    for (int yy = y < 0 ? 0 : y; yy < h; yy++)
        for (int x = 0; x < W; x++) b[yy * W + x] = rgb(10, 12, 24);
    text(b, h, (W - sv_text_w(page_hint[cfg.page], 1)) / 2, y, page_hint[cfg.page], rgb(255, 210, 0));
}

static void draw_help(void) {
    static const char *const lines[] = {
        "ESP-SDR  T-Dongle S3", "tap   action on page", "hold  next page", "hold 2s  flip screen",
        "pages SPEC CHAN HUNT", "USB: esp-sdr viewer", "  works as usual",
    };
    uint16_t *b = td_lcd_strip();
    for (int y0 = 0; y0 < H; y0 += TD_STRIP_H) {
        int h = H - y0 < TD_STRIP_H ? H - y0 : TD_STRIP_H;
        sv_fill(b, W, h, rgb(10, 12, 24));
        for (int i = 0; i < (int)(sizeof(lines) / sizeof(lines[0])); i++)
            text(b, h, 2, 2 + i * 11 - y0, lines[i], i ? rgb(220, 230, 240) : rgb(60, 255, 120));
        td_lcd_blit(0, y0, W, h, b);
    }
}

static void draw_spec(void) {
    char s[32];
    snprintf(s, sizeof(s), "%u %uM pk%.1f %.0f", cfg.centre, sv_spans[cfg.span].msps,
             sv_bin_mhz(view_centre, view_span, (float)peak_bin), bins[peak_bin]);
    draw_status(s, NULL, rgb(230, 240, 255));
    uint16_t grid = rgb(30, 36, 50), fillc = rgb(0, 70, 40), line = rgb(60, 255, 120);
    uint16_t *b = td_lcd_strip();
    for (int y0 = 0; y0 < SPEC_H; y0 += TD_STRIP_H) {
        int h = SPEC_H - y0 < TD_STRIP_H ? SPEC_H - y0 : TD_STRIP_H;
        for (int yy = 0; yy < h; yy++)
            for (int x = 0; x < W; x++)
                b[yy * W + x] = (y0 + yy) % 11 == 0 || x % 40 == 0 || x == W / 2 ? grid : 0;
        for (int x = 0; x < W; x++) {
            int y = db_to_y(cols[x], SPEC_H), prev = x ? db_to_y(cols[x - 1], SPEC_H) : y;
            int top = y < prev ? y : prev, bot = y < prev ? prev : y;
            for (int yy = 0; yy < h; yy++) {
                int yl = y0 + yy;
                if (yl > bot) b[yy * W + x] = fillc;
                if (yl >= top && yl <= bot) b[yy * W + x] = line;
            }
        }
        td_lcd_blit(0, SPEC_Y + y0, W, h, b);
    }
    sv_fill(b, W, AXIS_H, 0);
    for (int i = 0; i < 3; i++) {
        int x = i * (W - 1) / 2;
        snprintf(s, sizeof(s), "%.0f", sv_bin_mhz(view_centre, view_span, (float)x * SV_BINS / W));
        int tx = i == 0 ? 0 : i == 1 ? x - sv_text_w(s, 1) / 2 : W - sv_text_w(s, 1);
        text(b, AXIS_H, tx, 0, s, rgb(160, 170, 190));
    }
    td_lcd_blit(0, AXIS_Y, W, AXIS_H, b);
    for (int y0 = 0; y0 < WF_H; y0 += TD_STRIP_H) {
        int h = WF_H - y0 < TD_STRIP_H ? WF_H - y0 : TD_STRIP_H;
        for (int yy = 0; yy < h; yy++) {
            const uint8_t *row = wf + ((wf_top + y0 + yy) % WF_H) * W;
            for (int x = 0; x < W; x++) b[yy * W + x] = lut[row[x]];
        }
        hint_overlay(b, WF_Y + y0, h);
        td_lcd_blit(0, WF_Y + y0, W, h, b);
    }
}

static int best_channel(void) {
    static const int candidates[] = {1, 6, 11};
    int best = 1;
    for (int i = 0; i < 3; i++)
        if (busy_frames[candidates[i] - 1] < busy_frames[best - 1]) best = candidates[i];
    return best;
}

static void draw_chan(void) {
    char s[32], r[16];
    int best = best_channel();
    if (frames) snprintf(s, sizeof(s), "airtime  best ch%d", best);
    else snprintf(s, sizeof(s), "airtime");
    snprintf(r, sizeof(r), "%lus", (unsigned long)(survey_us / 1000000));
    draw_status(s, r, rgb(230, 240, 255));
    const int top = STATUS_H + 2, bars = 56, label_y = H - 9;
    uint16_t *b = td_lcd_strip();
    for (int y0 = STATUS_H; y0 < H; y0 += TD_STRIP_H) {
        int h = H - y0 < TD_STRIP_H ? H - y0 : TD_STRIP_H;
        sv_fill(b, W, h, 0);
        for (int c = 0; c < CHANNELS; c++) {
            float air = frames ? (float)busy_frames[c] / frames : 0;
            int x0 = 2 + c * 12, bh = (int)(air * bars + 0.5f);
            if (air > 0 && bh < 1) bh = 1;
            uint16_t col = air < 0.2f ? rgb(0, 200, 90) : air < 0.5f ? rgb(240, 220, 0) : rgb(255, 60, 0);
            int live_y = top + bars - 1 - (int)(live[c] * (bars - 1) + 0.5f);
            for (int yy = 0; yy < h; yy++) {
                int y = y0 + yy;
                for (int x = x0; x < x0 + 9; x++) {
                    uint16_t *p = &b[yy * W + x];
                    if (y >= top + bars - bh && y < top + bars) *p = col;
                    else if (y >= top && y < top + bars) *p = rgb(16, 20, 30);
                    if (y == live_y && frames) *p = rgb(255, 255, 255);
                }
            }
            snprintf(s, sizeof(s), "%d", c + 1);
            uint16_t lc = c + 1 == best && frames ? rgb(60, 255, 120) : (c == 0 || c == 5 || c == 10) ? rgb(220, 230, 240) : rgb(110, 120, 140);
            text(b, h, x0 + 5 - sv_text_w(s, 1) / 2, label_y - y0, s, lc);
        }
        hint_overlay(b, y0, h);
        td_lcd_blit(0, y0, W, h, b);
    }
}

static void draw_hunt(void) {
    char s[32];
    float snr = level_db - floor_db;
    if (lock_mhz > 0) snprintf(s, sizeof(s), "HUNT  lock %.1f", lock_mhz);
    else snprintf(s, sizeof(s), "HUNT  %u %uM", cfg.centre, sv_spans[cfg.span].msps);
    draw_status(s, NULL, lock_mhz > 0 ? rgb(255, 120, 200) : rgb(230, 240, 255));
    float mhz = lock_mhz > 0 ? lock_mhz : sv_bin_mhz(view_centre, view_span, (float)peak_bin);
    int bar = (int)(snr * (W - 8) / 50);
    bar = bar < 0 ? 0 : bar > W - 8 ? W - 8 : bar;
    int pk = (int)((level_peak - floor_db) * (W - 8) / 50);
    pk = pk < 0 ? 0 : pk > W - 8 ? W - 8 : pk;
    uint16_t *b = td_lcd_strip();
    for (int y0 = STATUS_H; y0 < H; y0 += TD_STRIP_H) {
        int h = H - y0 < TD_STRIP_H ? H - y0 : TD_STRIP_H;
        sv_fill(b, W, h, rgb(8, 10, 20));
        snprintf(s, sizeof(s), "%.0f", level_db);
        sv_text(b, W, h, 4, 12 - y0, s, rgb(255, 255, 255), 3);
        text(b, h, 6 + sv_text_w(s, 3), 26 - y0, "dBFS", rgb(160, 170, 190));
        snprintf(s, sizeof(s), "%+.0f", snr);
        sv_text(b, W, h, W - 4 - sv_text_w(s, 2), 12 - y0, s, rgb(60, 255, 120), 2);
        text(b, h, W - 4 - sv_text_w("dB SNR", 1), 30 - y0, "dB SNR", rgb(160, 170, 190));
        snprintf(s, sizeof(s), "%.1f MHz", mhz);
        text(b, h, 4, 42 - y0, s, rgb(220, 230, 240));
        for (int yy = 0; yy < h; yy++) {
            int y = y0 + yy;
            if (y < 54 || y > 62) continue;
            for (int x = 0; x < W - 8; x++)
                b[yy * W + 4 + x] = y == 54 || y == 62 ? rgb(70, 80, 100)
                                  : x == pk ? rgb(255, 255, 255)
                                  : x < bar ? (x < 50 ? rgb(0, 200, 90) : x < 100 ? rgb(240, 220, 0) : rgb(255, 60, 0))
                                            : 0;
        }
        hint_overlay(b, y0, h);
        td_lcd_blit(0, y0, W, h, b);
    }
}

/* Whole screen, or only the strip holding the frequency (y 32-47). */
static void draw_host(bool freq_only) {
    char s[32];
    uint16_t *b = td_lcd_strip();
    host_freq = sdr_local_freq();
    snprintf(s, sizeof(s), "%u MHz", host_freq);
    for (int y0 = freq_only ? 32 : 0; y0 < (freq_only ? 48 : H); y0 += TD_STRIP_H) {
        int h = H - y0 < TD_STRIP_H ? H - y0 : TD_STRIP_H;
        sv_fill(b, W, h, rgb(10, 12, 24));
        text(b, h, 2, 24 - y0, "USB host in control", rgb(255, 210, 0));
        text(b, h, 2, 40 - y0, s, rgb(220, 230, 240));
        td_lcd_blit(0, y0, W, h, b);
    }
}

/* ---------- measurement ---------- */
static void hue(float t, uint8_t *r, uint8_t *g, uint8_t *b) {
    /* 0: blue, 0.33: green, 0.66: yellow, 1: red */
    t = t < 0 ? 0 : t > 1 ? 1 : t;
    if (t < 0.33f) {float f = t / 0.33f; *r = 0; *g = (uint8_t)(255 * f); *b = (uint8_t)(255 * (1 - f));}
    else if (t < 0.66f) {float f = (t - 0.33f) / 0.33f; *r = (uint8_t)(255 * f); *g = 255; *b = 0;}
    else {float f = (t - 0.66f) / 0.34f; *r = 255; *g = (uint8_t)(255 * (1 - f)); *b = 0;}
}

/* Noise floor for the survey: the 10th percentile of the band. The median
 * lands inside the signals when channels 1, 6 and 11 are all on air (their
 * central 16 MHz covers ~63 % of the bins), which would read as idle. */
static float survey_floor(const float *db) {
    const int k = SV_BINS / 10;
    memcpy(scratch, db, SV_BINS * sizeof(float));
    for (int i = 0; i <= k; i++)
        for (int j = i + 1; j < SV_BINS; j++)
            if (scratch[j] < scratch[i]) {float t = scratch[i]; scratch[i] = scratch[j]; scratch[j] = t;}
    return scratch[k];
}

static void survey(const float *db) {
    int64_t now = esp_timer_get_time();
    const float busy_db = survey_floor(db) + 5;
    if (survey_last) survey_us += now - survey_last < 200000 ? now - survey_last : 200000;
    survey_last = now;
    frames++;
    for (int c = 0; c < CHANNELS; c++) {
        /* Mean power over the central 16 MHz of the 20 MHz channel. */
        float fc = 2412 + 5 * c;
        int a = (int)ceilf(sv_mhz_bin(BAND_MHZ, 0, fc - 8)), z = (int)floorf(sv_mhz_bin(BAND_MHZ, 0, fc + 8));
        if (a < 0) a = 0;
        if (z > SV_BINS - 1) z = SV_BINS - 1;
        float p = 0;
        for (int i = a; i <= z; i++) p += powf(10, db[i] / 10);
        bool busy = 10 * log10f(p / (z - a + 1)) > busy_db;
        busy_frames[c] += busy;
        live[c] += ((float)busy - live[c]) * 0.05f;
    }
}

static void reset_survey(void) {
    frames = 0;
    survey_us = survey_last = 0;
    memset(busy_frames, 0, sizeof(busy_frames));
    memset(live, 0, sizeof(live));
}

static bool measure(void) {
    /* CHAN always needs the whole band; SPEC and HUNT use the zoomed view. */
    unsigned centre = cfg.page == PAGE_CHAN ? BAND_MHZ : cfg.centre;
    uint8_t span = cfg.page == PAGE_CHAN ? 0 : cfg.span;
    /* Every frame: a no-op when already there, and it undoes any retune or
     * FOFS offset a host left behind, even one never seen as host_active. */
    sdr_local_tune(centre);
    if (centre != view_centre || span != view_span) {
        view_centre = centre; view_span = span;
        for (int i = 0; i < SV_BINS; i++) bins[i] = NAN;
        wf_clear = true;
        next_scale = 0;
        lock_mhz = 0;
        survey_last = 0;
    }
    const float *db = sv_measure(span, true);
    if (!db) return false;
    floor_db = sv_floor(db, SV_BINS, scratch);
    top_bin = peak_bin = 0;
    for (int i = 0; i < SV_BINS; i++) {
        bins[i] = isfinite(bins[i]) ? bins[i] + 0.4f * (db[i] - bins[i]) : db[i];
        if (db[i] > db[top_bin]) top_bin = i;
        if (bins[i] > bins[peak_bin]) peak_bin = i;
    }
    top_db = db[top_bin];
    if (centre == BAND_MHZ && span == 0) survey(db);
    /* Hunt level: the strongest bin, or the bins around the locked frequency. */
    float lvl = top_db;
    if (lock_mhz > 0) {
        int c = (int)lroundf(sv_mhz_bin(centre, span, lock_mhz));
        lvl = -200;
        for (int i = c - 2; i <= c + 2; i++)
            if (i >= 0 && i < SV_BINS && db[i] > lvl) lvl = db[i];
    }
    level_db += (lvl - level_db) * 0.5f;
    level_peak = lvl > level_peak ? lvl : level_peak - 0.3f;
    int64_t now = esp_timer_get_time();
    if (now >= next_scale) {
        next_scale = now + SCALE_US;
        sv_autoscale(bins, SV_BINS, scratch, &ref_db, &range_db);
    }
    sv_resample_max(bins, SV_BINS, cols, W);
    if (wf_clear) {memset(wf, 0, WF_H * W); wf_clear = false;}
    wf_top = (wf_top + WF_H - 1) % WF_H;
    uint8_t *row = wf + wf_top * W;
    float lo = ref_db - range_db;
    for (int x = 0; x < W; x++) {
        float t = (cols[x] - lo) / range_db * 255;
        row[x] = t < 0 ? 0 : t > 255 ? 255 : (uint8_t)t;
    }
    /* LED: hue from the strongest signal's height above the floor. */
    uint8_t r, g, b;
    float snr = (lock_mhz > 0 ? lvl : top_db) - floor_db;
    hue((snr - 6) / 34, &r, &g, &b);
    td_led_set(r, g, b, cfg.page == PAGE_HUNT ? 24 : snr < 6 ? 1 : 3);
    return true;
}

/* ---------- input ---------- */
static void changed(void) { save_at = esp_timer_get_time() + SAVE_US; }

static void tap(void) {
    if (cfg.page == PAGE_CHAN) {reset_survey(); return;}
    if (!view_centre) return; /* nothing measured yet (help screen, host at boot) */
    if (cfg.page == PAGE_SPEC) {
        /* Zoom onto the strongest signal; from 16 MHz back to the whole band.
         * The target goes 1/8 span right of the LO: removing each segment's
         * mean (the LO leak) would also cancel a carrier sitting on DC. */
        if (cfg.span + 1 < SV_SPANS) {
            float f = sv_bin_mhz(view_centre, view_span, (float)peak_bin) - sv_spans[cfg.span + 1].msps / 8.0f;
            long c = lroundf(f);
            cfg.centre = (uint16_t)(c < 100 ? 100 : c > 6000 ? 6000 : c);
            cfg.span++;
        } else {
            cfg.centre = BAND_MHZ;
            cfg.span = 0;
        }
        changed();
    } else {
        lock_mhz = lock_mhz > 0 ? 0 : sv_bin_mhz(view_centre, view_span, (float)peak_bin);
        level_peak = -200;
    }
}

static void next_page(void) {
    lock_mhz = 0; /* the lock belongs to HUNT, which is being left */
    cfg.page = (uint8_t)((cfg.page + 1) % PAGES);
    hint_until = esp_timer_get_time() + HINT_US;
    level_peak = -200;
    changed();
}

/* Returns a status line while the button is held, NULL otherwise. */
static const char *button(int64_t now, bool host_active) {
    bool down = td_button_down();
    if (down && !press_at) press_at = now;
    if (!press_at) return NULL;
    if (host_active) press_void = true; /* the screen can't show what a press does */
    int64_t held = now - press_at;
    if (down) {
        if (held < 30000 || press_void) return NULL; /* debounce */
        return held >= HOLD_FLIP_US ? "release: flip screen" : held >= HOLD_PAGE_US ? "release: next page" : NULL;
    }
    press_at = 0;
    if (press_void) {press_void = false; return NULL;}
    if (held < 30000) return NULL;
    if (help) {help = false; hint_until = now + HINT_US; return NULL;}
    if (held >= HOLD_FLIP_US) {cfg.flip = !cfg.flip; td_lcd_flip(cfg.flip); changed();}
    else if (held >= HOLD_PAGE_US) next_page();
    else tap();
    return NULL;
}

/* ---------- entry points ---------- */
void board_ui_init(void) {
    /* LCD, LED, button and SD pins stay out of the host-controllable GPIO set. */
    esp_gpio_reserve(BIT64(0) | BIT64(1) | BIT64(2) | BIT64(3) | BIT64(4) | BIT64(5) | BIT64(12) | BIT64(14) |
                     BIT64(16) | BIT64(17) | BIT64(18) | BIT64(21) | BIT64(38) | BIT64(39) | BIT64(40));
    bins = malloc(SV_BINS * sizeof(float));
    scratch = malloc(SV_BINS * sizeof(float));
    cols = malloc(W * sizeof(float));
    /* As on the Cardputer, the waterfall borrows ring bank 0 (heap is scarce
     * beside the RF ring) and restarts after a host session. */
    _Static_assert(WF_H * W <= 0x10000, "waterfall exceeds one ring bank");
    wf = (uint8_t *)ring_capture_bank(0);
    ok = sv_init() && bins && scratch && cols && wf;
    if (!ok) return;
    nvs_handle_t h;
    size_t len = sizeof(cfg);
    if (nvs_open("tdongle", NVS_READONLY, &h) == ESP_OK) {
        typeof(cfg) saved;
        if (nvs_get_blob(h, "cfg", &saved, &len) == ESP_OK && len == sizeof(cfg) && saved.span < SV_SPANS &&
            saved.page < PAGES && saved.centre >= 100 && saved.centre <= 6000)
            cfg = saved;
        nvs_close(h);
    }
    sv_waterfall_lut(lut);
    td_button_init();
    td_led_init();
    td_lcd_init();
    td_lcd_flip(cfg.flip);
    draw_help();
    td_lcd_backlight(true);
    hint_until = esp_timer_get_time() + 3000000; /* help screen time */
}

bool board_ui_step(bool host_active) {
    if (!ok) return false;
    int64_t now = esp_timer_get_time();
    const char *holding = button(now, host_active);
    if (save_at && now >= save_at) {
        save_at = 0;
        nvs_handle_t h;
        if (nvs_open("tdongle", NVS_READWRITE, &h) == ESP_OK) {
            nvs_set_blob(h, "cfg", &cfg, sizeof(cfg)); nvs_commit(h); nvs_close(h);
        }
    }
    if (host_active) {
        if (!host_shown) {host_shown = true; draw_host(false); td_led_set(120, 0, 255, 2);}
        else if (sdr_local_freq() != host_freq) draw_host(true); /* one 16-row strip */
        return false;
    }
    if (host_shown) {host_shown = false; wf_clear = true; next_frame = 0;}
    if (help) {
        if (now < hint_until) return false;
        help = false;
        hint_until = now + HINT_US;
    }
    if (now < next_frame) return false;
    next_frame = now + FRAME_US;
    if (!measure()) return false;
    if (cfg.page == PAGE_SPEC) draw_spec();
    else if (cfg.page == PAGE_CHAN) draw_chan();
    else draw_hunt();
    if (holding) draw_status(holding, NULL, rgb(255, 210, 0));
    vTaskDelay(1);
    return true;
}
