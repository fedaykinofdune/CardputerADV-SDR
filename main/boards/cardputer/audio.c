/* Cardputer ADV speaker: ES8311 codec (I2C 0x18) fed by I2S1, 16-bit stereo.
 *
 * The I2S driver links its DMA descriptors in a circle, so playback loops
 * without any interrupt. Samples are written straight into those buffers and
 * the GDMA "current descriptor" register says which one is playing; that is
 * all cp_audio_put needs, so it also works on core 1 or with interrupts
 * masked during an on-device IQ run. */
#include <math.h>
#include <string.h>
#include "cardputer.h"
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "esp_attr.h"
#include "esp_memory_utils.h"
#include "esp_private/esp_gpio_reserve.h"
#include "esp_rom_sys.h"
#include "soc/gdma_reg.h"
#include "soc/soc.h"

#define PIN_BCLK 41
#define PIN_DOUT 42
#define PIN_WS 43
#define ES8311_ADDR 0x18
#define NBUF 8
#define FRAMES 240 /* 7.68 ms per buffer */
#define GDMA_CHANNELS 5
#define GDMA_STRIDE (GDMA_OUT_DSCR_CH1_REG - GDMA_OUT_DSCR_CH0_REG)
#define GDMA_PERI_I2S1 4

typedef struct dma_desc {
    uint32_t ctrl;
    uint32_t *buf;
    struct dma_desc *next;
} dma_desc_t;

static i2s_chan_handle_t tx;
static i2c_master_dev_handle_t codec;
static uint32_t *bufs[NBUF];
static uint32_t descs[NBUF]; /* low 20 bits, as the GDMA link fields hold them */
static volatile uint32_t *dscr_reg;
static unsigned wb, wpos;
static bool found, running;

static bool codec_write(void) {
    /* As M5Unified for the Cardputer ADV (MCLK = BCLK x 8 = 256 fs), plus a
     * 16-bit I2S input word to match the slots. */
    static const uint8_t regs[][2] = {
        {0x00, 0x80}, {0x01, 0xB5}, {0x02, 0x18}, {0x09, 0x0C}, {0x0D, 0x01},
        {0x12, 0x00}, {0x13, 0x10}, {0x32, 0xBF}, {0x37, 0x08},
    };
    for (unsigned i = 0; i < sizeof(regs) / sizeof(regs[0]); i++)
        if (i2c_master_transmit(codec, regs[i], 2, 50) != ESP_OK) return false;
    return true;
}

bool cp_audio_init(void) {
    if (tx) return true;
    const i2c_device_config_t dev = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = ES8311_ADDR, .scl_speed_hz = 400000,
    };
    if (!cp_i2c_bus() || i2c_master_bus_add_device(cp_i2c_bus(), &dev, &codec) != ESP_OK) return false;
    i2s_chan_config_t chan = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    chan.dma_desc_num = NBUF;
    chan.dma_frame_num = FRAMES;
    if (i2s_new_channel(&chan, &tx, NULL) != ESP_OK) {tx = NULL; return false;}
    const i2s_std_config_t std = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(CP_AUDIO_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {.mclk = I2S_GPIO_UNUSED, .bclk = PIN_BCLK, .ws = PIN_WS, .dout = PIN_DOUT, .din = I2S_GPIO_UNUSED},
    };
    /* The board reservation covers these pins; hand them to the driver. */
    esp_gpio_revoke(BIT64(PIN_BCLK) | BIT64(PIN_DOUT) | BIT64(PIN_WS));
    if (i2s_channel_init_std_mode(tx, &std) != ESP_OK) {i2s_del_channel(tx); tx = NULL; return false;}
    return true;
}

/* Index of the buffer the DMA is on, -1 if unknown. */
IRAM_ATTR static int playing(void) {
    uint32_t d = *dscr_reg & 0xfffff;
    for (int k = 0; k < NBUF; k++) if (descs[k] == d) return k;
    return -1;
}

/* Walk the circular descriptor chain from the one the DMA is on. */
static bool find_ring(void) {
    for (unsigned ch = 0; ch < GDMA_CHANNELS; ch++) {
        if ((REG_READ(GDMA_OUT_PERI_SEL_CH0_REG + ch * GDMA_STRIDE) & 0x3f) != GDMA_PERI_I2S1) continue;
        dscr_reg = (volatile uint32_t *)(GDMA_OUT_DSCR_CH0_REG + ch * GDMA_STRIDE);
        const dma_desc_t *d = (const dma_desc_t *)*dscr_reg, *start;
        if (!esp_ptr_internal(d)) /* else the link start: low 20 bits of a DRAM address */
            d = (const dma_desc_t *)((SOC_DRAM_LOW & ~0xfffffu) | (REG_READ(GDMA_OUT_LINK_CH0_REG + ch * GDMA_STRIDE) & 0xfffff));
        start = d;
        for (unsigned k = 0; k < NBUF; k++) {
            if (!esp_ptr_internal(d) || ((uintptr_t)d & 3) || !esp_ptr_internal(d->buf)) return false;
            descs[k] = (uint32_t)d & 0xfffff;
            bufs[k] = d->buf;
            d = d->next;
        }
        return d == start;
    }
    return false;
}

bool cp_audio_start(void) {
    if (running) return true;
    if (!cp_audio_init()) return false;
    for (unsigned k = 0; found && k < NBUF; k++) memset(bufs[k], 0, FRAMES * 4);
    if (i2s_channel_enable(tx) != ESP_OK) return false;
    esp_rom_delay_us(500);
    if (!found) {
        found = find_ring();
        if (!found) {i2s_channel_disable(tx); return false;}
        for (unsigned k = 0; k < NBUF; k++) memset(bufs[k], 0, FRAMES * 4);
    }
    if (!codec_write()) {i2s_channel_disable(tx); return false;}
    int p = playing();
    wb = (unsigned)(p < 0 ? 0 : p + NBUF / 2) % NBUF;
    wpos = 0;
    running = true;
    return true;
}

void cp_audio_stop(void) {
    if (!running) return;
    running = false;
    i2s_channel_disable(tx);
}

IRAM_ATTR void cp_audio_put(int32_t s) {
    if (!running) return;
    if (s > 32767) s = 32767;
    else if (s < -32768) s = -32768;
    uint32_t v = (uint16_t)s;
    bufs[wb][wpos] = v | v << 16;
    if (++wpos < FRAMES) return;
    wpos = 0;
    unsigned next = wb + 1 == NBUF ? 0 : wb + 1;
    int p = playing();
    /* Stay at least two buffers ahead of the DMA (it may already have
     * fetched the next descriptor); after a gap, skip to half a ring ahead. */
    if (p >= 0 && (next + NBUF - (unsigned)p) % NBUF < 2) next = ((unsigned)p + NBUF / 2) % NBUF;
    wb = next;
}

/* Between IQ runs: silence what was already played, so a long pause plays
 * zeros instead of looping stale audio. Unplayed samples stay queued. */
void cp_audio_pause(void) {
    if (!running) return;
    int p = playing();
    if (p < 0) return;
    memset(&bufs[wb][wpos], 0, (FRAMES - wpos) * 4);
    for (unsigned k = (wb + 1) % NBUF; k != (unsigned)p && k != wb; k = (k + 1) % NBUF)
        memset(bufs[k], 0, FRAMES * 4);
}

/* A whole number of cycles per buffer keeps every buffer seamless, so the
 * tone can change pitch at any buffer boundary without a click. */
void cp_audio_tone(unsigned hz, unsigned amplitude) {
    if (!running) return;
    unsigned cycles = (hz * FRAMES + CP_AUDIO_RATE / 2) / CP_AUDIO_RATE;
    int p = playing();
    uint32_t *first = NULL;
    for (unsigned k = 0; k < NBUF; k++) {
        if ((int)k == p) continue;
        if (first) {memcpy(bufs[k], first, FRAMES * 4); continue;}
        first = bufs[k];
        for (unsigned n = 0; n < FRAMES; n++) {
            int32_t s = cycles ? (int32_t)(amplitude * sinf(2 * (float)M_PI * cycles * n / FRAMES)) : 0;
            uint32_t v = (uint16_t)s;
            first[n] = v | v << 16;
        }
    }
}
