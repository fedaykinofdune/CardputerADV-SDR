/* Cardputer ADV scanner: finds narrowband signals in the 13 cm ham band and
 * the 2.4 GHz ISM band for listen mode's random hop (key j).
 *
 * Only about 1.8-2.6 GHz reaches the radio, so HF Morse and number stations
 * are out of reach. Up here the audible traffic is analog FM (ham NBFM,
 * baby monitors, wireless mics and headsets) and CW beacons; the rest is
 * Wi-Fi and Bluetooth data.
 *
 * A sweep steps 16 MS/s snapshots (62.5 kHz bins) across 2300-2483.5 MHz in
 * PASSES passes. Odd passes shift the windows by half a step, so the DC hole
 * at each window's centre is seen by the other passes, and the passes spread
 * each frequency's frames over the whole sweep. Per bin and pass parity the
 * sweep counts the frames that stand HIT_DB above that frame's noise floor,
 * and by how much. A real signal shows up in both parities; a receiver spur
 * or IQ image tied to the LO lands on different bins in each, so it is
 * dropped. Peaks a few bins wide that were there in a fair share of the
 * frames become candidates; wide bursty regions are kept as a fallback
 * ("data chatter", played in AM). The tallies live in ring bank 1, which
 * only host ring runs and listen-mode IQ runs write, so the sweep needs no
 * heap. */
#include <math.h>
#include <string.h>
#include "cardputer.h"
#include "esp_random.h"
#include "nvs.h"
#include "ring_capture.h"

#define LO_KHZ 2300000u
#define HI_KHZ 2483500u
#define BIN_HZ 62500u
#define NBINS ((HI_KHZ - LO_KHZ) * 1000u / BIN_HZ)
#define SPAN16 2              /* sv_spans index of 16 MS/s */
#define CENTRE (SV_BINS / 2)  /* the LO's column in a snapshot */
#define WINDOWS 16
#define PASSES 4
#define FRAMES 4              /* snapshots per window and pass */
#define HIT_DB 8.0f
#define IMAGE_DB 20.0f        /* weaker than its mirror by this much: an IQ image */
#define MIN_SNR 10.0f
#define MAX_CANDS 20
#define NSKIP 16
#define NRECENT 4
#define NSUSPECT 8
#define STRIKES 3             /* dead verdicts before a frequency is skipped for good */

_Static_assert(SV_FFT_N == 256 && SV_BINS == 240, "62.5 kHz bins with the LO at column 120");

/* Per 62.5 kHz bin: dB above the floor summed over hit frames and over all
 * frames; frames and hits per pass parity. */
typedef struct { float sum, all; uint8_t n[2], hits[2]; } cell_t;
typedef struct { uint32_t khz; uint8_t kind, snr, pres, played; } cand_t;

_Static_assert(NBINS * sizeof(cell_t) + SV_BINS * sizeof(float) <= RING_BANK_STRIDE, "sweep tallies fit in one ring bank");
_Static_assert(PASSES / 2 * FRAMES * 2 < 256, "per-parity counts fit a byte");

static cell_t *map; /* NBINS cells */
static float *scratch;
static unsigned pass, win, frames, nc;
static cand_t cands[MAX_CANDS];
static struct { uint32_t khz[NSKIP]; uint8_t next; } skip; /* NVS "cardputer"/"scanskip" */
static bool skip_loaded;
static uint32_t recent[NRECENT];
static uint8_t recent_next;
/* Dead-carrier verdicts so far (RAM only): a beacon's long carrier looks
 * dead for a while, a spur looks dead every time. */
static uint32_t suspect[NSUSPECT];
static uint8_t strikes[NSUSPECT], suspect_next;

static unsigned dist(unsigned a, unsigned b) { return a > b ? a - b : b - a; }
static unsigned window_mhz(unsigned p, unsigned w) { return (p & 1 ? 2301u : 2307u) + 12u * w; }

/* Before the list is read or written: 'x' can come before any scan. */
static void skip_load(void) {
    if (skip_loaded) return;
    skip_loaded = true;
    nvs_handle_t h;
    size_t len = sizeof(skip);
    if (nvs_open("cardputer", NVS_READONLY, &h) == ESP_OK) {
        if (nvs_get_blob(h, "scanskip", &skip, &len) != ESP_OK || len != sizeof(skip) || skip.next >= NSKIP)
            memset(&skip, 0, sizeof(skip));
        nvs_close(h);
    }
}

void cp_scan_begin(void) {
    skip_load();
    map = (cell_t *)ring_capture_bank(1);
    scratch = (float *)(map + NBINS);
    memset(map, 0, NBINS * sizeof(cell_t));
    pass = win = frames = nc = 0;
    float *pano = cp_ui_pano(LO_KHZ, HI_KHZ);
    for (int x = 0; x < CP_LCD_W; x++) pano[x] = -200;
}

/* 10th percentile of a snapshot: the noise floor under Wi-Fi and carriers. */
static float floor_db(const float *db) {
    memcpy(scratch, db, SV_BINS * sizeof(float));
    const int k = SV_BINS / 10;
    for (int i = 0; i <= k; i++)
        for (int j = i + 1; j < SV_BINS; j++)
            if (scratch[j] < scratch[i]) {float t = scratch[i]; scratch[i] = scratch[j]; scratch[j] = t;}
    return scratch[k];
}

static unsigned frames_of(unsigned b) { return map[b].n[0] + map[b].n[1]; }
static unsigned hits_of(unsigned b) { return map[b].hits[0] + map[b].hits[1]; }
static float level(unsigned b) { return frames_of(b) ? map[b].sum / frames_of(b) : 0; } /* SNR x presence */
static float snr_of(unsigned b) { return hits_of(b) ? map[b].sum / hits_of(b) : 0; }
static float mean_all(unsigned b) { return frames_of(b) ? map[b].all / frames_of(b) : 0; }

/* Seen from both LO grids (where a parity covered the bin at all): a spur or
 * image that moves with the LO lights a bin from one grid only. */
static bool both_grids(unsigned b) {
    const cell_t *m = &map[b];
    for (unsigned p = 0; p < 2; p++)
        if (m->n[p] && m->hits[p] * 8u < m->n[p]) return false;
    return true;
}

static bool near_spur(unsigned khz) {
    unsigned r = khz % 40000u; /* harmonics of the 40 MHz crystal */
    return r < 150u || r > 40000u - 150u;
}

static bool skipped(unsigned khz) {
    for (unsigned i = 0; i < NSKIP; i++)
        if (skip.khz[i] && dist(khz, skip.khz[i]) <= 50u) return true;
    return false;
}

static unsigned weight(const cand_t *c) {
    unsigned w = (c->snr > 40 ? 40u : c->snr) * (c->pres + 25u);
    for (unsigned i = 0; i < NRECENT; i++)
        if (recent[i] && dist(c->khz, recent[i]) <= 50u) {w /= 8; break;} /* just played: rarely again */
    return w ? w : 1;
}

/* Keep the MAX_CANDS best, voice-capable ones before data. */
static unsigned rank(const cand_t *c) { return weight(c) + (c->kind == CP_SCAN_DATA ? 0u : 1u << 16); }

static void add(unsigned khz, unsigned kind, float snr, unsigned pres) {
    cand_t c = {khz, (uint8_t)kind, (uint8_t)(snr > 99 ? 99 : snr), (uint8_t)pres, 0};
    if (nc < MAX_CANDS) {cands[nc++] = c; return;}
    unsigned lo = 0;
    for (unsigned i = 1; i < nc; i++) if (rank(&cands[i]) < rank(&cands[lo])) lo = i;
    if (rank(&cands[lo]) < rank(&c)) cands[lo] = c;
}

/* Neighbour b belongs to a peak seen hits times at snr dB when it was there
 * at least half as often, at most 10 dB weaker and not clearly stronger. */
static bool member(unsigned b, unsigned hits, float snr) {
    float s = snr_of(b);
    return hits_of(b) * 2u >= hits && s >= snr - 10 && s <= snr + 3;
}

static void extract(void) {
    nc = 0;
    for (unsigned b = 1; b + 1 < NBINS; b++) {
        unsigned n = frames_of(b), hits = hits_of(b);
        if (n < 4 || hits * 10u < n) continue; /* there in under 10 % of frames */
        float l = level(b);
        if (l < level(b - 1) || l <= level(b + 1)) continue;
        float snr = snr_of(b);
        if (snr < MIN_SNR) continue;
        unsigned lo = b, hi = b;
        while (lo > 1 && b - lo < 64 && member(lo - 1, hits, snr)) lo--;
        while (hi + 2 < NBINS && hi - b < 64 && member(hi + 1, hits, snr)) hi++;
        unsigned width = hi - lo + 1, pres = hits * 100u / n, peak = b;
        unsigned kind = width <= 4 ? CP_SCAN_NARROW : width <= 8 ? CP_SCAN_WIDEFM : CP_SCAN_DATA;
        /* Parabolic fit over all frames, so weak peaks get one too. */
        float a0 = mean_all(b - 1), a1 = mean_all(b), a2 = mean_all(b + 1);
        float den = a0 - 2 * a1 + a2, delta = den < 0 ? 0.5f * (a0 - a2) / den : 0;
        delta = delta > 0.5f ? 0.5f : delta < -0.5f ? -0.5f : delta;
        unsigned khz = LO_KHZ + (unsigned)lroundf((peak + delta) * (BIN_HZ / 1000.0f));
        b = hi; /* one candidate per peak */
        /* Data is only worth a listen while it comes and goes. */
        if (kind == CP_SCAN_DATA ? pres > 90 : pres < 25) continue;
        if (!both_grids(peak) || near_spur(khz) || skipped(khz)) continue;
        add(khz, kind, snr, pres);
    }
}

bool cp_scan_sweep(unsigned *pass_out, unsigned *mhz_out) {
    unsigned c = window_mhz(pass, win);
    *pass_out = pass;
    *mhz_out = c;
    sdr_local_tune(c);
    float *pano = cp_ui_pano(LO_KHZ, HI_KHZ);
    int b0 = ((int)c - (int)(LO_KHZ / 1000)) * 16 - CENTRE; /* map bin of column 0 */
    for (unsigned f = 0; f < FRAMES; f++) {
        const float *db = sv_measure(SPAN16, true);
        if (!db) continue;
        frames++;
        float fl = floor_db(db);
        for (int x = 0; x < SV_BINS; x++) {
            if (x >= CENTRE - 2 && x <= CENTRE + 2) continue; /* DC removal hole */
            int xm = 2 * CENTRE - x;
            bool hit = db[x] - fl > HIT_DB && !(xm < SV_BINS && db[xm] > db[x] + IMAGE_DB);
            int b = b0 + x;
            if (b < 0 || b >= (int)NBINS) continue;
            cell_t *m = &map[b];
            m->n[pass & 1]++;
            m->all += db[x] - fl;
            if (hit) {m->hits[pass & 1]++; m->sum += db[x] - fl;}
            int px = b * CP_LCD_W / (int)NBINS;
            if (db[x] > pano[px]) pano[px] = db[x];
        }
    }
    cp_ui_frozen(c * 1000u); /* the panorama, marker on this window */
    if (++win < WINDOWS) return false;
    win = 0;
    cp_ui_pano_scale(pano);
    if (++pass < PASSES) return false;
    pass = 0;
    extract();
    return true;
}

static bool eligible(const cand_t *c, bool narrow_left) {
    return !c->played && (!narrow_left || c->kind != CP_SCAN_DATA);
}

unsigned cp_scan_found(unsigned *left) {
    unsigned n = 0;
    for (unsigned i = 0; i < nc; i++) n += !cands[i].played;
    if (left) *left = n;
    return nc;
}

bool cp_scan_next(cp_scan_hit_t *h) {
    bool narrow = false;
    for (unsigned i = 0; i < nc; i++) narrow |= !cands[i].played && cands[i].kind != CP_SCAN_DATA;
    uint32_t total = 0;
    for (unsigned i = 0; i < nc; i++) if (eligible(&cands[i], narrow)) total += weight(&cands[i]);
    if (!total) return false;
    uint32_t r = esp_random() % total;
    unsigned i = 0;
    for (;; i++) {
        if (!eligible(&cands[i], narrow)) continue;
        unsigned w = weight(&cands[i]);
        if (r < w) break;
        r -= w;
    }
    cand_t *c = &cands[i];
    c->played = 1;
    if (c->kind == CP_SCAN_DATA) /* one taste of data chatter per sweep */
        for (unsigned k = 0; k < nc; k++) if (cands[k].kind == CP_SCAN_DATA) cands[k].played = 1;
    recent[recent_next++ % NRECENT] = c->khz;
    *h = (cp_scan_hit_t){c->khz, c->kind, c->snr, c->pres};
    return true;
}

void cp_scan_never(unsigned khz) {
    skip_load();
    if (skipped(khz)) return;
    skip.khz[skip.next] = khz;
    skip.next = (uint8_t)((skip.next + 1) % NSKIP);
    for (unsigned i = 0; i < nc; i++) if (dist(cands[i].khz, khz) <= 50u) cands[i].played = 1;
    nvs_handle_t h;
    if (nvs_open("cardputer", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_blob(h, "scanskip", &skip, sizeof(skip)); nvs_commit(h); nvs_close(h);
    }
}

static int suspect_at(unsigned khz) {
    for (unsigned i = 0; i < NSUSPECT; i++)
        if (strikes[i] && dist(khz, suspect[i]) <= 50u) return (int)i;
    return -1;
}

bool cp_scan_dead(unsigned khz) {
    int i = suspect_at(khz);
    if (i < 0) {
        i = suspect_next;
        suspect_next = (uint8_t)((suspect_next + 1) % NSUSPECT);
        suspect[i] = khz;
        strikes[i] = 0;
    }
    for (unsigned k = 0; k < nc; k++) if (dist(cands[k].khz, khz) <= 50u) cands[k].played = 1;
    if (++strikes[i] < STRIKES) return false;
    strikes[i] = 0;
    cp_scan_never(khz);
    return true;
}

void cp_scan_alive(unsigned khz) {
    int i = suspect_at(khz);
    if (i >= 0) strikes[i] = 0;
}

/* Rough labels: 13 cm allocations differ by country (the US has 2300-2310
 * and 2390-2450 MHz), and 2400-2483.5 MHz is licence-free ISM almost everywhere. */
const char *cp_scan_band(unsigned khz) {
    if (khz >= 2400050u && khz <= 2400500u) return "QO-100 uplink";
    if (khz < 2400000u) return "13cm band";
    if (khz < 2450000u) return "13cm/ISM";
    return "2.4G ISM";
}
