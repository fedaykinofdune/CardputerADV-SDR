/* Shared on-device spectrum measurement and drawing helpers for board UIs.
 *
 * One measurement is one burst capture of SV_SEGMENTS x SV_FFT_N samples,
 * Hann windowed and Welch averaged. The outer bins sit in the anti-alias
 * roll-off, so only the middle SV_BINS are returned, lowest RF first. */
#pragma once
#include <stdbool.h>
#include <stdint.h>

#define SV_FFT_N 256
#define SV_SEGMENTS 16
#define SV_BINS 240

typedef struct { unsigned divider, msps; } sv_span_t;
#define SV_SPANS 3
extern const sv_span_t sv_spans[SV_SPANS]; /* 80, 40, 16 MS/s */

bool sv_init(void);
/* dBFS per bin (heap buffer, valid until the next call), NULL if the capture failed. */
const float *sv_measure(unsigned span, bool dc_fix);
/* Bin (fractional, 0..SV_BINS-1) <-> MHz for a view centred on centre_mhz. */
float sv_bin_mhz(unsigned centre_mhz, unsigned span, float bin);
float sv_mhz_bin(unsigned centre_mhz, unsigned span, float mhz);
/* Peak-preserving resample of n bins to w display columns (w <= n). */
void sv_resample_max(const float *db, int n, float *out, int w);
/* Median-ish noise floor of n values (scratch must hold n floats). */
float sv_floor(const float *db, int n, float *scratch);
/* Reference level and range that frame the strongest signal and the floor. */
void sv_autoscale(const float *db, int n, float *scratch, int8_t *ref, int8_t *range);
/* Black - blue - cyan - yellow - red - white, big-endian RGB565. */
void sv_waterfall_lut(uint16_t lut[256]);

/* Drawing into a w x h big-endian RGB565 buffer; 5x7 glyphs in a 6x8 cell. */
void sv_fill(uint16_t *buf, int w, int h, uint16_t c);
void sv_text(uint16_t *buf, int w, int h, int x, int y, const char *s, uint16_t fg, int scale);
static inline int sv_text_w(const char *s, int scale) {
    int n = 0;
    while (s[n]) n++;
    return n * 6 * scale;
}
static inline uint16_t sv_rgb(unsigned r, unsigned g, unsigned b) {
    uint16_t c = (uint16_t)(((r & 0xf8) << 8) | ((g & 0xfc) << 3) | (b >> 3));
    return (uint16_t)((c >> 8) | (c << 8));
}
