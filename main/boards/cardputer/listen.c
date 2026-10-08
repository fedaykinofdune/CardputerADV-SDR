/* Cardputer ADV listen mode: NFM / AM / WFM / CW demodulation into the
 * speaker, and the scanner's random hop between busy signals (key j).
 *
 * The S3 IQS path (16 MS/s, LO tuned fs/4 below, +fs/4 shift, two-stage FIR)
 * hands every decimated sample to demod() instead of USB frames. demod runs
 * on core 1 (or core 0 with interrupts masked), so it is IRAM code on DRAM
 * data in integer math, and it writes audio straight into the I2S DMA ring.
 * A key press raises the keyboard interrupt line, which ends the run; the
 * key takes effect and the next run starts.
 *
 * Scanning: scan.c sweeps the band and lists candidates. Each pick gets a
 * short wide probe to centre the carrier, then plays in 1 s runs. After
 * every run the statistics demod gathered decide: a keyed carrier switches
 * to CW, a solid carrier with no modulation is skipped for good, and the
 * scanner hops when the signal is gone, the dwell is up, or on j. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cardputer.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"

#define PANEL_H (CP_LCD_H - CP_PANEL_Y)
#define LO_OFFSET_KHZ 4000u /* fs/4 at 16 MS/s */
#define FS_HZ 16000000u
#define MIN_KHZ (100000u + LO_OFFSET_KHZ)
#define MAX_KHZ (6000000u + LO_OFFSET_KHZ)

enum { NFM, AM, WFM, CW, MODES };
static const char *const mode_names[MODES] = {"NFM", "AM", "WFM", "CW"};
static const unsigned steps_khz[] = {1, 5, 10, 25, 100, 1000};
#define NSTEPS (sizeof(steps_khz) / sizeof(steps_khz[0]))
/* Volume in 1/16 (16 = unity), about 3 dB apart. */
static const uint8_t vol_q4[] = {0, 1, 2, 3, 4, 6, 8, 11, 16, 23, 32, 45, 64, 90, 128, 181};
#define NVOL (sizeof(vol_q4) / sizeof(vol_q4[0]))
/* Squelch on the mean |step| of the FM discriminator; noise alone sits near
 * 22000 (random phase), a clean carrier far below. 0 = always open. */
static const uint16_t sql_open[] = {65535, 17000, 14000, 11000, 8500, 6500, 5000, 3800, 2800, 2000};
#define NSQL (sizeof(sql_open) / sizeof(sql_open[0]))

/* CW: 500 Hz low-pass on I/Q, beat note 700 Hz, AGC to a fixed level. The
 * beat oscillator follows the carrier's measured offset, so the note stays
 * near 700 Hz between 1 kHz tuning steps. */
#define CW_ALPHA 392        /* 1 - exp(-2 pi 500 / 31250), Q12 */
#define BFO_STEP 96207267u  /* 700 Hz at 31.25 kHz, 2^32 per cycle */
#define CW_PITCH 700
#define CW_LEVEL 8000

/* Scanner decisions, from per-run statistics. */
#define PRESENT_NZ 11000    /* discriminator noise below this: a carrier is on the channel */
#define TICK 32             /* statistics tick, audio samples (about 1 ms) */
#define TICKS_PER_S 976u    /* 31250 / TICK */
#define DEAD_S 5            /* solid carrier with flat audio this long: dead */
#define DEAD_SPREAD 12      /* % run-to-run audio activity spread that counts as flat */
#define WIDE_DEV_HZ 6000    /* mean deviation in the probe above this: wide FM */
#define QUIET_S 8           /* no carrier this long: hop */
#define STALE_US 120000000  /* sweep again after two minutes */

/* Persistent listen settings (NVS "cardputer"/"listen"). */
static struct {
    uint8_t mode, vol, sql, step, live;
} lc = {NFM, 8, 3, 3, 0};

static bool active, loaded, entering, stuck_poll;
static bool halted; /* captures kept failing: the error stays up until a key */
static uint8_t mode; /* demodulator in use: lc.mode, or the scanner's choice */
static unsigned khz, fails, stuck;
static unsigned lo_mhz; /* the LO listen or the sweep last set */
static char entry[10], msg[40];
static int64_t msg_until; /* 0: msg stays until a key */

static enum { SC_OFF, SC_SWEEP, SC_PROBE, SC_PLAY } sc;
static bool probing; /* SC_PROBE run: wide and muted */
static cp_scan_hit_t hit;
static unsigned sw_pass, sw_mhz;
static uint8_t sc_secs, sc_quiet, sc_dead;
static uint16_t sc_acts[DEAD_S]; /* audio activity of the last solid-carrier runs */
static int64_t swept_at;

/* ---------- demodulator (core 1, IRAM/DRAM only) ---------- */
static struct {
    int32_t pi, pq, prev, aprev, lp, dcacc, acc, ci, cq, env;
    uint32_t pwr, pf, pk, fl, fq, noise, amdc, n, tk, bfo, bfo_step;
    uint32_t sub, sub_log2, tick_len, alpha, gain, open_thr, close_thr;
    bool am, cw, open, key;
} dm;

/* Per-run statistics for the scanner, zeroed before every run. */
static struct {
    uint32_t ticks, present, keyable, on, edges, fn, actn;
    int64_t fsum;  /* discriminator sum while a carrier is present: its offset */
    uint64_t act;  /* |audio| sum while a carrier is present: modulation plus noise */
    uint64_t nz;   /* discriminator noise per tick: noise alone */
} rs;

/* sin(2 pi k / 64) in Q14 for the CW beat oscillator. */
DRAM_ATTR static const int16_t sin64[64] = {
    0, 1606, 3196, 4756, 6270, 7723, 9102, 10394, 11585, 12665, 13623, 14449, 15137, 15679, 16069, 16305,
    16384, 16305, 16069, 15679, 15137, 14449, 13623, 12665, 11585, 10394, 9102, 7723, 6270, 4756, 3196, 1606,
    0, -1606, -3196, -4756, -6270, -7723, -9102, -10394, -11585, -12665, -13623, -14449, -15137, -15679, -16069, -16305,
    -16384, -16305, -16069, -15679, -15137, -14449, -13623, -12665, -11585, -10394, -9102, -7723, -6270, -4756, -3196, -1606,
};

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
    dm.pf += (p >> 7) - (dm.pf >> 7); /* fast power for keying, about 4 ms */
    /* FM discriminator, phase step per sample (pi = 32768). Every mode runs
     * it: its noise tells the scanner a carrier is there, and it is the
     * squelch in all modes but AM. */
    int32_t re = i * dm.pi + q * dm.pq, im = q * dm.pi - i * dm.pq;
    dm.pi = i; dm.pq = q;
    int32_t f = phase(im, re), d = f - dm.prev;
    dm.prev = f;
    dm.fq += (uint32_t)(d < 0 ? -d : d) - (dm.fq >> 8);
    uint32_t nz = dm.fq >> 8;
    bool here = nz < PRESENT_NZ;
    if (here) {rs.fsum += f; rs.fn++;}
    if (++dm.tk >= dm.tick_len) {
        dm.tk = 0;
        /* Keying: fast power against a peak that sinks and a floor that
         * rises toward it (about 0.5 s), with hysteresis. */
        uint32_t pf = dm.pf;
        dm.pk = pf > dm.pk ? pf : dm.pk - ((dm.pk - pf) >> 9) - (dm.pk > pf);
        dm.fl = pf < dm.fl ? pf : dm.fl + ((pf - dm.fl) >> 9) + (pf > dm.fl);
        uint32_t swing = dm.pk - dm.fl;
        bool on = pf > dm.fl + (dm.key ? swing >> 3 : swing >> 2);
        rs.ticks++;
        rs.present += here;
        rs.nz += nz;
        if (dm.pk / 4 > dm.fl) {rs.keyable++; rs.on += on; rs.edges += on != dm.key;}
        dm.key = on;
        dm.env -= (dm.env >> 9) + (dm.env > 64); /* CW AGC release, down to the floor */
    }
    int32_t a = f;
    if (dm.am) {
        int32_t ai = i < 0 ? -i : i, aq = q < 0 ? -q : q;
        int32_t mag = ai > aq ? ai + ((aq * 3) >> 3) : aq + ((ai * 3) >> 3);
        dm.amdc += (uint32_t)mag - (dm.amdc >> 11);
        int32_t c = (int32_t)(dm.amdc >> 11);
        a = c > 16 ? (mag - c) * 8192 / c : 0; /* modulation depth, 100 % = 8192 */
        if (a > 32767) a = 32767;
        d = a - dm.aprev; /* AM squelches on the envelope's noise */
        dm.aprev = a;
        dm.noise += (uint32_t)(d < 0 ? -d : d) - (dm.noise >> 8);
        nz = dm.noise >> 8;
    } else if (dm.cw) {
        dm.ci += ((i * 16 - dm.ci) * CW_ALPHA) >> 12;
        dm.cq += ((q * 16 - dm.cq) * CW_ALPHA) >> 12;
        int32_t ci = dm.ci >> 4, cq = dm.cq >> 4;
        unsigned k = (dm.bfo += dm.bfo_step) >> 26;
        int32_t y = (ci * sin64[(k + 16) & 63] - cq * sin64[k]) >> 14; /* Re{c e^jwt} */
        int32_t ai = ci < 0 ? -ci : ci, aq = cq < 0 ? -cq : cq;
        int32_t mag = ai > aq ? ai + ((aq * 3) >> 3) : aq + ((ai * 3) >> 3);
        if (mag > dm.env) dm.env = mag;
        a = y * CW_LEVEL / (dm.env > 64 ? dm.env : 64);
    }
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
    if (here) {rs.act += (uint32_t)(dm.lp < 0 ? -dm.lp : dm.lp); rs.actn++;}
    cp_audio_put(dm.open ? (dm.lp * (int32_t)dm.gain) >> 4 : 0);
}

IRAM_ATTR static bool never(void) { return false; }

static unsigned run_mode(void) { return probing ? WFM : mode; }
static unsigned decimation(void) { return run_mode() == WFM ? 64u : 512u; }

static void demod_setup(bool reset) {
    /* One-pole low-pass, alpha = 1 - exp(-2 pi fc / 31250) in Q12:
     * NFM and CW 3 kHz, AM 4 kHz, WFM 75 us de-emphasis (2.1 kHz). */
    static const uint16_t alpha[MODES] = {1854, 2264, 1422, 1854};
    unsigned m = run_mode();
    if (reset) {
        memset(&dm, 0, sizeof(dm));
        dm.open = lc.sql == 0;
        dm.fl = UINT32_MAX;
        dm.bfo_step = BFO_STEP;
    }
    dm.pi = dm.pq = 0; /* the FIR restarts: no phase step across runs */
    dm.am = m == AM;
    dm.cw = m == CW;
    dm.sub = m == WFM ? 8 : 1;
    dm.sub_log2 = m == WFM ? 3 : 0;
    dm.tick_len = TICK * dm.sub;
    dm.alpha = alpha[m];
    dm.gain = probing ? 0 : vol_q4[lc.vol];
    dm.open_thr = sql_open[lc.sql];
    dm.close_thr = lc.sql ? sql_open[lc.sql] + sql_open[lc.sql] / 4 : 65535;
    memset(&rs, 0, sizeof(rs));
}

/* ---------- screen ---------- */
static void fmt_freq(char *s, size_t n, unsigned k) { snprintf(s, n, "%u.%03u", k / 1000, k % 1000); }

static float level_db(void) {
    /* FIR output: 10-bit full scale x 32 = 16384 -> 0 dBFS */
    return 10 * log10f((dm.pwr + 1.0f) / (16384.0f * 16384.0f));
}

static void note(const char *s) {
    snprintf(msg, sizeof(msg), "%s", s);
    msg_until = esp_timer_get_time() + 3000000;
}

static void draw_status(void) {
    uint16_t *b = cp_lcd_strip();
    char line[48];
    cp_fill(b, CP_LCD_W, 11, cp_rgb(20, 24, 40));
    uint16_t c = cp_rgb(60, 255, 120);
    unsigned left;
    if (entering) {snprintf(line, sizeof(line), "Listen at: %s_ MHz [Enter]", entry); c = cp_rgb(255, 210, 0);}
    else if (msg[0] && (!msg_until || esp_timer_get_time() < msg_until)) {
        snprintf(line, sizeof(line), "%s", msg); c = cp_rgb(255, 120, 80);
    } else if (sc == SC_SWEEP) {
        snprintf(line, sizeof(line), "SCAN sweeping %u MHz", sw_mhz); c = cp_rgb(80, 200, 255);
    } else if (sc != SC_OFF) {
        cp_scan_found(&left);
        snprintf(line, sizeof(line), "SCAN %s  %u left  %us", cp_scan_band(khz), left, sc_secs);
        c = cp_rgb(80, 200, 255);
    } else snprintf(line, sizeof(line), "LISTEN  speaker on%s", lc.live ? "  live meter" : "");
    cp_text(b, CP_LCD_W, 11, 2, 2, line, c, 1);
    if (!entering) cp_text(b, CP_LCD_W, 11, CP_LCD_W - 6 * 3 - 2, 2, "?=h", cp_rgb(120, 130, 160), 1);
    cp_lcd_blit(0, 0, CP_LCD_W, 11, b);
}

static void draw_panel(void) {
    static const uint16_t mode_rgb[MODES][3] = {{80, 200, 255}, {255, 180, 60}, {200, 120, 255}, {255, 240, 120}};
    char f[16], line[48];
    bool sweeping = sc == SC_SWEEP;
    fmt_freq(f, sizeof(f), sweeping ? sw_mhz * 1000u : khz);
    const char *mname = sweeping ? "SCAN" : mode_names[mode];
    const uint16_t *m = sweeping ? mode_rgb[NFM] : mode_rgb[mode];
    float db = level_db();
    int bar = (int)((db + 100) * 160 / 100);
    bar = bar < 0 || sweeping ? 0 : bar > 160 ? 160 : bar;
    uint16_t *b = cp_lcd_strip();
    for (int y0 = 0; y0 < PANEL_H; y0 += CP_STRIP_H) {
        int h = PANEL_H - y0 < CP_STRIP_H ? PANEL_H - y0 : CP_STRIP_H;
        cp_fill(b, CP_LCD_W, h, cp_rgb(8, 10, 20));
        cp_text(b, CP_LCD_W, h, 4, 4 - y0, f, sweeping ? cp_rgb(120, 140, 170) : cp_rgb(255, 255, 255), 3);
        cp_text(b, CP_LCD_W, h, 6 + (int)strlen(f) * 18, 18 - y0, "MHz", cp_rgb(160, 170, 190), 1);
        cp_text(b, CP_LCD_W, h, CP_LCD_W - 4 - (int)strlen(mname) * 12, 4 - y0, mname, cp_rgb(m[0], m[1], m[2]), 2);
        if (sweeping) snprintf(line, sizeof(line), "vol %u  sql %u  hunting...", lc.vol, lc.sql);
        else snprintf(line, sizeof(line), "vol %u  sql %u  step %uk", lc.vol, lc.sql, steps_khz[lc.step]);
        cp_text(b, CP_LCD_W, h, 4, 32 - y0, line, cp_rgb(220, 230, 240), 1);
        if (!sweeping && (mode == NFM || mode == WFM)) { /* discriminator DC = offset from the tuned frequency */
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
        if (!sweeping) {
            snprintf(line, sizeof(line), "%.0fdB%s", db, dm.open ? "" : " sq");
            cp_text(b, CP_LCD_W, h, CP_LCD_W - 2 - (int)strlen(line) * 6, 44 - y0, line, cp_rgb(220, 230, 240), 1);
        }
        if (halted) cp_text(b, CP_LCD_W, h, 4, 57 - y0, "capture failed: l back, any key retry", cp_rgb(255, 120, 80), 1);
        else if (sc != SC_OFF) cp_text(b, CP_LCD_W, h, 4, 57 - y0, "j next  x never  l back  ,/ manual", cp_rgb(100, 110, 140), 1);
        else cp_text(b, CP_LCD_W, h, 4, 57 - y0, "l back m mode -= vol []sql j scan", cp_rgb(100, 110, 140), 1);
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
    "m    mode NFM / AM / WFM / CW",
    "- =  volume     [ ]  squelch",
    "j    scan: hop to a busy signal",
    "x    scan never picks this again",
    "v    live meter    l `  back",
    "Tuning keys stop the scan.",
    "ofs: FM carrier offset, tune",
    "  toward it.  USB host stops it.",
};

/* ---------- control ---------- */
static bool help_shown;

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
    lo_mhz = (khz - LO_OFFSET_KHZ) / 1000;
    if (!help_shown) cp_ui_frozen(khz);
}

/* The last run's carrier presence (%) and offset from the dial (Hz): the
 * discriminator's mean over the samples where a carrier was there. */
static int run_offset(unsigned *pres) {
    *pres = rs.ticks ? rs.present * 100u / rs.ticks : 0;
    if (*pres < 20 || !rs.fn) return 0;
    return (int)(rs.fsum * (int64_t)(FS_HZ / decimation()) / ((int64_t)rs.fn * 65536));
}

/* Retune to the carrier the discriminator sees off_hz away (1 kHz steps);
 * returns the kHz moved. */
static int afc(int off_hz) {
    int k = (off_hz + (off_hz < 0 ? -500 : 500)) / 1000;
    if (!k) return 0;
    tune((unsigned)((int)khz + k));
    if (mode == NFM || mode == WFM) /* the audio DC filter held the old offset */
        dm.dcacc -= (int32_t)((int64_t)k * 1000 * 65536 / (FS_HZ / decimation())) * 1024;
    return k;
}

/* CW: a carrier off_hz from the dial beats at off_hz + bfo; keep it at CW_PITCH. */
static void cw_track(int off_hz) {
    int hz = CW_PITCH - off_hz;
    hz = hz < 200 ? 200 : hz > 1200 ? 1200 : hz;
    dm.bfo_step = (uint32_t)(((uint64_t)hz << 32) / (FS_HZ / 512));
}

static void scan_start(void) {
    sc = SC_SWEEP;
    probing = false;
    cp_scan_begin();
    cp_audio_tone(0, 0); /* quiet while the LO hops */
}

/* Next random pick, or a fresh sweep when the list is used up or stale. */
static void scan_hop(const char *why) {
    if (why) note(why);
    cp_scan_hit_t h;
    if (esp_timer_get_time() - swept_at > STALE_US || !cp_scan_next(&h)) {scan_start(); return;}
    hit = h;
    mode = h.kind == CP_SCAN_DATA ? AM : h.kind == CP_SCAN_WIDEFM ? WFM : NFM;
    probing = h.kind == CP_SCAN_NARROW;
    sc = probing ? SC_PROBE : SC_PLAY;
    sc_secs = sc_quiet = sc_dead = 0;
    demod_setup(true);
    tune(h.khz);
}

static void scan_stop(void) {
    bool sweeping = sc == SC_SWEEP;
    sc = SC_OFF;
    if (probing || sweeping) {probing = false; demod_setup(true);}
    if (sweeping) tune(khz); /* the sweep moved the LO */
}

/* After each run while scanning: centre, classify, hop. */
static void scan_after_run(void) {
    if (rs.ticks < (probing ? 100u : 500u)) return; /* cut short by a key */
    unsigned t = rs.ticks, pres;
    int off = run_offset(&pres);
    if (sc == SC_PROBE) {
        /* Wide FM (headsets, AV senders) swings tens of kHz: play it as WFM. */
        uint64_t dev = rs.actn ? rs.act / rs.actn * (FS_HZ / decimation()) / 65536 : 0;
        if (off > -100000 && off < 100000) afc(off);
        probing = false;
        sc = SC_PLAY;
        if (pres >= 20 && dev >= WIDE_DEV_HZ) mode = WFM;
        demod_setup(true);
        return;
    }
    sc_secs++;
    bool keyed = rs.keyable * 10u >= t * 3u                /* 6 dB+ swings for 30 % of the run */
                 && rs.edges * 2u * TICKS_PER_S >= t * 3u   /* 1.5 to 40 key edges a second */
                 && rs.edges * TICKS_PER_S <= t * 40u
                 && rs.on * 10u >= rs.keyable && rs.on * 10u <= rs.keyable * 9u;
    if (keyed) cp_scan_alive(khz);
    if (keyed && mode == NFM && hit.kind == CP_SCAN_NARROW) {
        mode = CW;
        demod_setup(true);
        note("Keyed carrier: Morse? CW mode");
    }
    if ((mode == NFM || mode == CW) && pres >= 30 && off > -15000 && off < 15000) {
        int moved = off >= 700 || off <= -700 ? afc(off) : 0;
        if (mode == CW) cw_track(off - moved * 1000);
    }
    /* A solid carrier with nothing on it (often a spur) is not busy. Its
     * audio is the receiver noise alone: the same level run after run, and
     * a fixed fraction (about 0.21) of the discriminator noise. Speech
     * comes and goes. */
    if (mode == NFM && pres >= 90 && !keyed && rs.actn) {
        uint64_t act = rs.act / rs.actn;
        memmove(sc_acts, sc_acts + 1, sizeof(sc_acts) - sizeof(sc_acts[0]));
        sc_acts[DEAD_S - 1] = (uint16_t)(act > UINT16_MAX ? UINT16_MAX : act);
        if (sc_dead < DEAD_S) sc_dead++;
        unsigned lo = UINT16_MAX, hi = 0;
        for (unsigned k = 0; k < DEAD_S; k++) {
            if (sc_acts[k] < lo) lo = sc_acts[k];
            if (sc_acts[k] > hi) hi = sc_acts[k];
        }
        if (sc_dead >= DEAD_S) {
            /* A beacon's long carrier looks dead too, so only the third
             * verdict on a frequency keeps it out for good. */
            if ((hi - lo) * 100u < hi * DEAD_SPREAD || hi * 4u < rs.nz / t) {
                scan_hop(cp_scan_dead(khz) ? "Dead carrier: skipped for good" : "Dead carrier: next");
                return;
            }
            cp_scan_alive(khz);
        }
    } else sc_dead = 0;
    if (hit.kind != CP_SCAN_DATA && pres < 20 && !keyed) {
        if (++sc_quiet >= QUIET_S) {scan_hop("Signal gone: next"); return;}
    } else sc_quiet = 0;
    unsigned dwell = hit.kind == CP_SCAN_DATA ? 10 : mode == WFM ? 30 : 60;
    if (sc_secs >= dwell) scan_hop(NULL);
}

static void leave(bool host) {
    active = false;
    entering = false;
    sc = SC_OFF;
    probing = false;
    cp_audio_stop();
    save();
    /* A host that retuned keeps its frequency; else back to the spectrum's. */
    cp_ui_resume(host && sdr_local_freq() != lo_mhz);
}

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
    if (sc != SC_OFF && (k == ',' || k == '/' || k == CP_KEY_LEFT || k == CP_KEY_RIGHT || k == 'f' || k == 'm'))
        scan_stop(); /* the user takes the dial */
    switch (k) {
    case ',': case CP_KEY_LEFT: tune(khz - steps_khz[lc.step]); break;
    case '/': case CP_KEY_RIGHT: tune(khz + steps_khz[lc.step]); break;
    case ';': case CP_KEY_UP: lc.step = (uint8_t)((lc.step + 1) % NSTEPS); break;
    case '.': case CP_KEY_DOWN: lc.step = (uint8_t)((lc.step + NSTEPS - 1) % NSTEPS); break;
    case 'f': entering = true; entry[0] = 0; break;
    case 'm': mode = lc.mode = (uint8_t)((mode + 1) % MODES); demod_setup(true); break;
    case '-': if (lc.vol) lc.vol--; break;
    case '=': if (lc.vol + 1u < NVOL) lc.vol++; break;
    case '[': if (lc.sql) lc.sql--; dm.open = lc.sql == 0; break;
    case ']': if (lc.sql + 1u < NSQL) lc.sql++; break;
    case 'v': lc.live = !lc.live; break;
    case 'j':
        if (sc == SC_OFF) scan_start();
        else if (sc != SC_SWEEP) scan_hop(NULL);
        break;
    case 'x':
        if (sc == SC_SWEEP) break;
        cp_scan_never(khz);
        if (sc != SC_OFF) scan_hop("Skipped for good");
        else note("The scanner will skip this one");
        break;
    case 'h': case '?': case CP_KEY_TAB:
        help_shown = true;
        cp_ui_help(help_lines, sizeof(help_lines) / sizeof(help_lines[0]));
        break;
    case 'l': case '`': case CP_KEY_ESC: leave(false); break;
    }
}

bool cp_listen_active(void) { return active; }

bool cp_listen_enter(unsigned k, bool scan) {
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
    entering = help_shown = halted = probing = false;
    sc = SC_OFF;
    fails = stuck = 0;
    msg[0] = 0;
    mode = lc.mode;
    demod_setup(true);
    tune(k);
    if (scan) scan_start();
    draw_status();
    draw_panel();
    return true;
}

/* Status line text for a failed run, short enough for the 40-column line. */
static void run_error(const ring_result_t *r) {
    static const char *const names[] = {"OK", "ARG", "LATE", "AGE", "START", "END", "LEN", "XPORT"};
    const char *n = r->status < sizeof(names) / sizeof(names[0]) ? names[r->status] : "?";
    msg_until = 0;
    if (r->status == RING_FAIL_ARG) /* allocation-type failures: show the DMA heap */
        snprintf(msg, sizeof(msg), "IQ %s/%u dma %u/%u", n, (unsigned)r->detail,
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
    else
        snprintf(msg, sizeof(msg), "IQ %s/%u u%u L%u %c", n, (unsigned)r->detail, (unsigned)r->units,
                 (unsigned)r->late_max, ring_capture_dual_active() ? 'D' : 'S');
}

/* One sweep window per step, so serial commands still get through. */
static void sweep_step(void) {
    for (int k; active && (k = cp_kbd_read());) key(k);
    if (!active) return;
    if (sc == SC_SWEEP && !help_shown) {
        bool done = cp_scan_sweep(&sw_pass, &sw_mhz);
        lo_mhz = sw_mhz; /* the sweep moved the LO */
        if (done) {
            swept_at = esp_timer_get_time();
            unsigned n = cp_scan_found(NULL);
            if (n) {
                snprintf(msg, sizeof(msg), "Found %u signal%s", n, n == 1 ? "" : "s");
                msg_until = swept_at + 3000000;
                scan_hop(NULL);
            } else {
                note("Band quiet: sweeping again");
                cp_scan_begin();
            }
        }
    }
    if (sc == SC_SWEEP) cp_audio_tone(0, 0); /* the ring would loop its last audio */
    if (help_shown) {vTaskDelay(pdMS_TO_TICKS(20)); return;}
    draw_status();
    draw_panel();
    vTaskDelay(1); /* let the other tasks run between windows */
}

void cp_listen_step(bool host_active) {
    if (host_active) {leave(true); return;}
    if (halted) {
        int k = cp_kbd_read();
        if (!k) {vTaskDelay(pdMS_TO_TICKS(20)); return;}
        if (k == 'l' || k == '`' || k == CP_KEY_ESC) {leave(false); return;}
        halted = false; fails = 0; msg[0] = 0; /* any other key: try again */
        draw_status();
        draw_panel();
        return;
    }
    if (sc == SC_SWEEP) {sweep_step(); return;}
    demod_setup(false);
    /* Open-ended while the key line works (a key ends the run); short runs
     * when the meter should move, keys must be polled, or the scanner has
     * to decide. Audio keeps playing under the help screen. */
    bool irq = cp_kbd_irq_ok() && !stuck_poll;
    uint32_t ms = sc != SC_OFF ? (probing ? 250 : 1000) : lc.live && !help_shown ? 1000 : irq ? 0 : 250;
    const ring_config_t rc = {
        .mode = RING_MODE_IQ, .rate = 6, .iq_dec = decimation(), .iq_bits = 16, .iq_shift = 0,
        .iq_rot = true, .duration_ms = ms, .iq_sink = demod, .stop_poll = irq ? cp_kbd_pending : never,
    };
    ring_result_t r;
    sdr_local_iq_run(&rc, &r);
    cp_audio_pause();
    if (r.stopped_by_host) {leave(true); return;}
    if (r.status != RING_OK) {
        run_error(&r);
        /* Keep the reason on screen (it used to flash and vanish). */
        if (++fails >= 10) {halted = true; draw_status(); draw_panel(); return;}
    } else fails = 0;
    /* Keys first: a hop or skip the user asked for wins over the verdict. */
    uint8_t sc0 = sc, mode0 = mode;
    unsigned khz0 = khz;
    bool any = false;
    for (int k; active && (k = cp_kbd_read());) {key(k); any = true;}
    if (!active) return;
    if (r.status == RING_OK && sc == sc0 && mode == mode0 && khz == khz0) {
        if (sc != SC_OFF) scan_after_run();
        else if (mode == CW && rs.ticks >= 500u) { /* manual CW: hold the pitch */
            unsigned pres;
            int off = run_offset(&pres);
            if (pres >= 30 && off > -1000 && off < 1000) cw_track(off);
        }
    }
    /* A line that stays low without events would end every run at once. */
    if (!any && cp_kbd_pending() && ++stuck > 3) stuck_poll = true;
    else if (any) stuck = 0;
    if (help_shown) return;
    draw_status();
    draw_panel();
}
