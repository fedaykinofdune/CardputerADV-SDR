/* Cardputer ADV listen mode: NFM / AM / WFM demodulation into the speaker.
 *
 * The S3 IQS path (16 MS/s, LO tuned fs/4 below, +fs/4 shift, two-stage FIR)
 * hands every decimated sample to demod() instead of USB frames. demod runs
 * on core 1 (or core 0 with interrupts masked), so it is IRAM code on DRAM
 * data in integer math, and it writes audio straight into the I2S DMA ring.
 * A key press raises the keyboard interrupt line, which ends the run; the
 * key takes effect and the next run starts. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cardputer.h"
#include "esp_attr.h"
#include "nvs.h"

#define PANEL_H (CP_LCD_H - CP_PANEL_Y)
#define LO_OFFSET_KHZ 4000u /* fs/4 at 16 MS/s */
#define FS_HZ 16000000u
#define MIN_KHZ (100000u + LO_OFFSET_KHZ)
#define MAX_KHZ (6000000u + LO_OFFSET_KHZ)

enum { NFM, AM, WFM, MODES };
static const char *const mode_names[MODES] = {"NFM", "AM", "WFM"};
static const unsigned steps_khz[] = {1, 5, 10, 25, 100, 1000};
#define NSTEPS (sizeof(steps_khz) / sizeof(steps_khz[0]))
/* Volume in 1/16 (16 = unity), about 3 dB apart. */
static const uint8_t vol_q4[] = {0, 1, 2, 3, 4, 6, 8, 11, 16, 23, 32, 45, 64, 90, 128, 181};
#define NVOL (sizeof(vol_q4) / sizeof(vol_q4[0]))
/* Squelch on the mean |step| of the demodulator output; noise alone sits
 * near 22000 (random phase), a clean carrier far below. 0 = always open. */
static const uint16_t sql_open[] = {65535, 17000, 14000, 11000, 8500, 6500, 5000, 3800, 2800, 2000};
#define NSQL (sizeof(sql_open) / sizeof(sql_open[0]))

/* Persistent listen settings (NVS "cardputer"/"listen"). */
static struct {
    uint8_t mode, vol, sql, step, live;
} lc = {NFM, 8, 3, 3, 0};

static bool active, loaded, entering, stuck_poll;
static unsigned khz, fails, stuck;
static char entry[10], msg[40];

/* ---------- demodulator (core 1, IRAM/DRAM only) ---------- */
static struct {
    int32_t pi, pq, prev, lp, dcacc, acc;
    uint32_t pwr, noise, amdc, n;
    uint32_t sub, sub_log2, alpha, gain, open_thr, close_thr;
    bool am, open;
} dm;

/* atan2 in units of pi/32768: octant reduction, then
 * atan(t) ~ pi/4 t + 0.273 t (1 - t) on [0, 1] (error < 0.004 rad). */
IRAM_ATTR static int32_t phase(int32_t y, int32_t x) {
    int32_t ax = x < 0 ? -x : x, ay = y < 0 ? -y : y;
    int32_t mx = ax > ay ? ax : ay, mn = ax > ay ? ay : ax;
    if (!mx) return 0;
    int bits = 32 - __builtin_clz((uint32_t)mx);
    if (bits > 16) {mx >>= bits - 16; mn >>= bits - 16;}
    if (!mx) return 0;
    int32_t t = (mn << 15) / mx;
    int32_t a = (t >> 2) + ((2848 * ((t * (32768 - t)) >> 15)) >> 15);
    if (ay > ax) a = 16384 - a;
    if (x < 0) a = 32768 - a;
    return y < 0 ? -a : a;
}

IRAM_ATTR static void demod(int32_t i, int32_t q) {
    q = -q; /* S3: RF above the LO arrives at negative frequency */
    uint32_t p = (uint32_t)(i * i) + (uint32_t)(q * q);
    dm.pwr += (p >> 10) - (dm.pwr >> 10);
    int32_t a;
    if (dm.am) {
        int32_t ai = i < 0 ? -i : i, aq = q < 0 ? -q : q;
        int32_t mag = ai > aq ? ai + ((aq * 3) >> 3) : aq + ((ai * 3) >> 3);
        dm.amdc += (uint32_t)mag - (dm.amdc >> 11);
        int32_t c = (int32_t)(dm.amdc >> 11);
        a = c > 16 ? ((mag - c) << 13) / c : 0; /* modulation depth, 100 % = 8192 */
        if (a > 32767) a = 32767;
    } else {
        int32_t re = i * dm.pi + q * dm.pq, im = q * dm.pi - i * dm.pq;
        dm.pi = i; dm.pq = q;
        a = phase(im, re); /* phase step per sample, pi = 32768 */
    }
    int32_t d = a - dm.prev;
    dm.prev = a;
    dm.noise += (uint32_t)(d < 0 ? -d : d) - (dm.noise >> 8);
    uint32_t nz = dm.noise >> 8;
    if (dm.open) {if (nz > dm.close_thr) dm.open = false;}
    else if (nz < dm.open_thr) dm.open = true;
    if (dm.sub > 1) { /* WFM: 250 kS/s down to the audio rate */
        dm.acc += a;
        if (++dm.n < dm.sub) return;
        a = dm.acc >> dm.sub_log2;
        dm.acc = 0; dm.n = 0;
    }
    dm.dcacc += a - (dm.dcacc >> 10); /* FM: tuning offset; AM: residual DC */
    a -= dm.dcacc >> 10;
    dm.lp += ((a - dm.lp) * (int32_t)dm.alpha) >> 12; /* de-emphasis / audio low-pass */
    cp_audio_put(dm.open ? (dm.lp * (int32_t)dm.gain) >> 4 : 0);
}

IRAM_ATTR static bool never(void) { return false; }

static unsigned decimation(void) { return lc.mode == WFM ? 64u : 512u; }

static void demod_setup(bool reset) {
    /* One-pole low-pass, alpha = 1 - exp(-2 pi fc / 31250) in Q12:
     * NFM 3 kHz, AM 4 kHz, WFM 75 us de-emphasis (2.1 kHz). */
    static const uint16_t alpha[MODES] = {1854, 2264, 1422};
    if (reset) {
        memset(&dm, 0, sizeof(dm));
        dm.open = lc.sql == 0;
    }
    dm.pi = dm.pq = 0; /* the FIR restarts: no phase step across runs */
    dm.am = lc.mode == AM;
    dm.sub = lc.mode == WFM ? 8 : 1;
    dm.sub_log2 = lc.mode == WFM ? 3 : 0;
    dm.alpha = alpha[lc.mode];
    dm.gain = vol_q4[lc.vol];
    dm.open_thr = sql_open[lc.sql];
    dm.close_thr = lc.sql ? sql_open[lc.sql] + sql_open[lc.sql] / 4 : 65535;
}

/* ---------- screen ---------- */
static void fmt_freq(char *s, size_t n, unsigned k) { snprintf(s, n, "%u.%03u", k / 1000, k % 1000); }

static float level_db(void) {
    /* FIR output: 10-bit full scale x 32 = 16384 -> 0 dBFS */
    return 10 * log10f((dm.pwr + 1.0f) / (16384.0f * 16384.0f));
}

static void draw_status(void) {
    uint16_t *b = cp_lcd_strip();
    char line[48];
    cp_fill(b, CP_LCD_W, 11, cp_rgb(20, 24, 40));
    uint16_t c = cp_rgb(60, 255, 120);
    if (entering) {snprintf(line, sizeof(line), "Listen at: %s_ MHz [Enter]", entry); c = cp_rgb(255, 210, 0);}
    else if (msg[0]) {snprintf(line, sizeof(line), "%s", msg); c = cp_rgb(255, 120, 80);}
    else snprintf(line, sizeof(line), "LISTEN  speaker on%s", lc.live ? "  live meter" : "");
    cp_text(b, CP_LCD_W, 11, 2, 2, line, c, 1);
    if (!entering) cp_text(b, CP_LCD_W, 11, CP_LCD_W - 6 * 3 - 2, 2, "?=h", cp_rgb(120, 130, 160), 1);
    cp_lcd_blit(0, 0, CP_LCD_W, 11, b);
}

static void draw_panel(void) {
    static const uint16_t mode_rgb[MODES][3] = {{80, 200, 255}, {255, 180, 60}, {200, 120, 255}};
    char f[16], line[48];
    fmt_freq(f, sizeof(f), khz);
    float db = level_db();
    int bar = (int)((db + 100) * 160 / 100);
    bar = bar < 0 ? 0 : bar > 160 ? 160 : bar;
    uint16_t *b = cp_lcd_strip();
    for (int y0 = 0; y0 < PANEL_H; y0 += CP_STRIP_H) {
        int h = PANEL_H - y0 < CP_STRIP_H ? PANEL_H - y0 : CP_STRIP_H;
        cp_fill(b, CP_LCD_W, h, cp_rgb(8, 10, 20));
        cp_text(b, CP_LCD_W, h, 4, 4 - y0, f, cp_rgb(255, 255, 255), 3);
        cp_text(b, CP_LCD_W, h, 6 + (int)strlen(f) * 18, 18 - y0, "MHz", cp_rgb(160, 170, 190), 1);
        const uint16_t *m = mode_rgb[lc.mode];
        cp_text(b, CP_LCD_W, h, CP_LCD_W - 4 - (int)strlen(mode_names[lc.mode]) * 12, 4 - y0,
                mode_names[lc.mode], cp_rgb(m[0], m[1], m[2]), 2);
        snprintf(line, sizeof(line), "vol %u  sql %u  step %uk", lc.vol, lc.sql, steps_khz[lc.step]);
        cp_text(b, CP_LCD_W, h, 4, 32 - y0, line, cp_rgb(220, 230, 240), 1);
        if (lc.mode != AM) { /* FM discriminator DC = offset from the tuned frequency */
            float hz = (float)(dm.dcacc >> 10) * (FS_HZ / decimation()) / 65536.0f;
            snprintf(line, sizeof(line), "ofs%+.1fk", hz / 1000);
            cp_text(b, CP_LCD_W, h, CP_LCD_W - 4 - (int)strlen(line) * 6, 32 - y0, line, cp_rgb(160, 170, 190), 1);
        }
        cp_text(b, CP_LCD_W, h, 4, 44 - y0, "sig", cp_rgb(160, 170, 190), 1);
        for (int y = 44; y < 51; y++) {
            int yy = y - y0;
            if (yy < 0 || yy >= h) continue;
            for (int x = 0; x < 162; x++)
                b[yy * CP_LCD_W + 26 + x] = x == 0 || x == 161 || y == 44 || y == 50 ? cp_rgb(70, 80, 100)
                                            : x <= bar ? (x < 80 ? cp_rgb(0, 200, 90) : x < 125 ? cp_rgb(240, 220, 0) : cp_rgb(255, 60, 0))
                                                       : 0;
        }
        snprintf(line, sizeof(line), "%.0fdB%s", db, dm.open ? "" : " sq");
        cp_text(b, CP_LCD_W, h, 194, 44 - y0, line, cp_rgb(220, 230, 240), 1);
        cp_text(b, CP_LCD_W, h, 4, 57 - y0, "l back m mode -= vol []sql ;. step", cp_rgb(100, 110, 140), 1);
        cp_lcd_blit(0, CP_PANEL_Y + y0, CP_LCD_W, h, b);
    }
}

static void draw_all(void) {
    cp_ui_frozen(khz);
    draw_status();
    draw_panel();
}

static const char *const help_lines[] = {
    "LISTEN  speaker demodulator",
    ", /  tune -/+ step   ; .  step",
    "f    type MHz (e.g. 2437.125)",
    "m    mode NFM / AM / WFM",
    "- =  volume     [ ]  squelch",
    "v    live signal meter on/off",
    "l `  back to the spectrum",
    "The screen pauses while audio",
    "plays; keys act at once.",
    "ofs: FM carrier offset, tune",
    "  toward it.  USB host stops it.",
};

/* ---------- control ---------- */
static void save(void) {
    nvs_handle_t h;
    if (nvs_open("cardputer", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_blob(h, "listen", &lc, sizeof(lc)); nvs_commit(h); nvs_close(h);
    }
}

static void tune(unsigned k) {
    if (k < MIN_KHZ) k = MIN_KHZ;
    if (k > MAX_KHZ) k = MAX_KHZ;
    khz = k;
    sdr_local_tune_khz(khz - LO_OFFSET_KHZ);
    cp_ui_frozen(khz);
}

static void leave(void) {
    active = false;
    entering = false;
    cp_audio_stop();
    save();
    cp_ui_resume();
}

static bool help_shown;

static void key(int k) {
    msg[0] = 0;
    if (help_shown) {help_shown = false; draw_all(); return;}
    if (entering) {
        size_t n = strlen(entry);
        if (((k >= '0' && k <= '9') || (k == '.' && !strchr(entry, '.'))) && n < sizeof(entry) - 1) {
            entry[n] = (char)k; entry[n + 1] = 0;
        } else if (k == CP_KEY_BACKSPACE && n) entry[n - 1] = 0;
        else if (k == CP_KEY_ENTER) {
            entering = false;
            if (n) tune((unsigned)lroundf(strtof(entry, NULL) * 1000));
        } else if (k == CP_KEY_ESC || k == '`') entering = false;
        return;
    }
    switch (k) {
    case ',': case CP_KEY_LEFT: tune(khz - steps_khz[lc.step]); break;
    case '/': case CP_KEY_RIGHT: tune(khz + steps_khz[lc.step]); break;
    case ';': case CP_KEY_UP: lc.step = (uint8_t)((lc.step + 1) % NSTEPS); break;
    case '.': case CP_KEY_DOWN: lc.step = (uint8_t)((lc.step + NSTEPS - 1) % NSTEPS); break;
    case 'f': entering = true; entry[0] = 0; break;
    case 'm': lc.mode = (uint8_t)((lc.mode + 1) % MODES); demod_setup(true); break;
    case '-': if (lc.vol) lc.vol--; break;
    case '=': if (lc.vol + 1u < NVOL) lc.vol++; break;
    case '[': if (lc.sql) lc.sql--; dm.open = lc.sql == 0; break;
    case ']': if (lc.sql + 1u < NSQL) lc.sql++; break;
    case 'v': lc.live = !lc.live; break;
    case 'h': case '?': case CP_KEY_TAB:
        help_shown = true;
        cp_ui_help(help_lines, sizeof(help_lines) / sizeof(help_lines[0]));
        break;
    case 'l': case '`': case CP_KEY_ESC: leave(); break;
    }
}

bool cp_listen_active(void) { return active; }

bool cp_listen_enter(unsigned k) {
    if (!loaded) {
        loaded = true;
        nvs_handle_t h;
        size_t len = sizeof(lc);
        if (nvs_open("cardputer", NVS_READONLY, &h) == ESP_OK) {
            typeof(lc) saved;
            if (nvs_get_blob(h, "listen", &saved, &len) == ESP_OK && len == sizeof(lc) && saved.mode < MODES &&
                saved.vol < NVOL && saved.sql < NSQL && saved.step < NSTEPS)
                lc = saved;
            nvs_close(h);
        }
    }
    if (!cp_audio_start()) return false;
    active = true;
    entering = help_shown = false;
    fails = stuck = 0;
    msg[0] = 0;
    demod_setup(true);
    tune(k);
    draw_status();
    draw_panel();
    return true;
}

void cp_listen_step(bool host_active) {
    if (host_active) {leave(); return;}
    demod_setup(false);
    /* Open-ended while the key line works (a key ends the run); short runs
     * when the meter should move or keys must be polled. Audio keeps
     * playing under the help screen. */
    bool irq = cp_kbd_irq_ok() && !stuck_poll;
    const ring_config_t rc = {
        .mode = RING_MODE_IQ, .rate = 6, .iq_dec = decimation(), .iq_bits = 16, .iq_shift = 0,
        .iq_rot = true, .duration_ms = lc.live && !help_shown ? 1000 : irq ? 0 : 250,
        .iq_sink = demod, .stop_poll = irq ? cp_kbd_pending : never,
    };
    ring_result_t r;
    sdr_local_iq_run(&rc, &r);
    cp_audio_pause();
    if (r.stopped_by_host) {leave(); return;}
    if (r.status != RING_OK) {
        snprintf(msg, sizeof(msg), "IQ run error %u/%u", (unsigned)r.status, (unsigned)r.detail);
        if (++fails >= 10) {leave(); return;}
    } else fails = 0;
    bool any = false;
    for (int k; active && (k = cp_kbd_read());) {key(k); any = true;}
    if (!active) return;
    /* A line that stays low without events would end every run at once. */
    if (!any && cp_kbd_pending() && ++stuck > 3) stuck_poll = true;
    else if (any) stuck = 0;
    if (help_shown) return;
    draw_status();
    draw_panel();
}
