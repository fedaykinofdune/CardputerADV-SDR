/* Services the S3 receiver offers to an on-device UI (targets/esp32s3/receiver.c),
 * and the entry points every board UI provides. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "ring_capture.h"

const uint32_t *sdr_local_capture(unsigned n, unsigned divider);
unsigned sdr_local_freq(void);
void sdr_local_tune(unsigned mhz); /* also clears any kHz offset */
void sdr_local_tune_khz(unsigned khz);
void sdr_local_iq_run(const ring_config_t *c, ring_result_t *r);
int sdr_local_gain(void); /* -1: hardware AGC */
unsigned sdr_local_gain_max(void);
void sdr_local_set_gain(int code);
unsigned sdr_local_bandwidth(void); /* MHz, 0: automatic */
void sdr_local_set_bandwidth(unsigned mhz);

/* Board UI. board_ui_init() runs before GPIO discovery so the board pins stay
 * reserved; board_ui_step() runs one frame in the command loop's idle slot
 * and returns false when it did nothing (the caller then yields). */
void board_ui_init(void);
bool board_ui_step(bool host_active);
