/*
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  RNode Linux
 *
 *  Copyright (c) 2025 Belousov Oleg aka R1CBU
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "csma.h"
#include "sx126x.h"
#include "util.h"
#include "rnode.h"

#define CSMA_CW_PER_BAND_WINDOWS    15
#define CSMA_SLOT_MAX_MS            100
#define CSMA_SLOT_MIN_MS            24
#define CSMA_SLOT_MIN_FAST_DELTA    18
#define CSMA_BAND_1_MAX_AIRTIME     7
#define CSMA_BAND_N_MIN_AIRTIME     85
#define CSMA_CW_BANDS               4
#define CSMA_SLOT_SYMBOLS           12
#define CSMA_SIFS_MS                0
#define CSMA_INFR_THRESHOLD_DB      11
#define CSMA_RFENV_RECAL_MS         2500
#define CSMA_RFENV_RECAL_LIMIT_DB   -83

#define PHY_HEADER_LORA_SYMBOLS     20
#define LORA_PREAMBLE_SYMBOLS_MIN   18
#define LORA_PREAMBLE_TARGET_MS     24
#define LORA_PREAMBLE_FAST_DELTA    18
#define LORA_FAST_THRESHOLD_BPS     30000
#define LORA_LIMIT_THRESHOLD_BPS    60000
#define LORA_GUARD_THRESHOLD_BPS    14000

#define AIRTIME_LONGTERM            3600
#define AIRTIME_LONGTERM_MS         (AIRTIME_LONGTERM * 1000)

#define DCD_SAMPLES                 2500
#define UTIL_UPDATE_INTERVAL        (1000 / STATUS_INTERVAL_MS)
#define AIRTIME_BINLEN_MS           (STATUS_INTERVAL_MS * DCD_SAMPLES)
#define AIRTIME_BINS                ((AIRTIME_LONGTERM * 1000) / AIRTIME_BINLEN_MS)

#define NOISE_FLOOR_SAMPLES         128

static csma_channel_t   channel = {
    .airtime = 0.0,
    .longterm_airtime = 0.0,
    .total_channel_util = 0.0,
    .longterm_channel_util = 0.0,
    .current_rssi = -292,
    .noise_floor = -292,
};

static csma_cw_t        cw = {
    .band =  1,
    .min = 0,
    .max = CSMA_CW_PER_BAND_WINDOWS
};

static uint16_t         airtime_bins[AIRTIME_BINS];
static float            longterm_bins[AIRTIME_BINS];

static int32_t          csma_slot_ms = CSMA_SLOT_MIN_MS;
static int32_t          difs_ms = CSMA_SIFS_MS + 2 * CSMA_SLOT_MIN_MS;

// PHY parameters (updated by csma_update_radio_params)
static float            lora_symbol_time_ms = 0.0;
static float            lora_symbol_rate = 0.0;
static long             lora_preamble_symbols = LORA_PREAMBLE_SYMBOLS_MIN;
static long             lora_preamble_time_ms = 0;

// Flush mode: at very low bitrates, send all queued packets after one DIFS+CW
static bool             lora_should_flush = false;

// DCD tracking
static bool             dcd = false;
static bool             util_samples[DCD_SAMPLES];
static int              dcd_sample = 0;
static float            local_channel_util = 0.0;

// Noise floor tracking
static int32_t          noise_floor_buffer[NOISE_FLOOR_SAMPLES];
static uint16_t         noise_floor_sample = 0;
static bool             noise_floor_sampled = false;

// Interference detection
static bool             interference_detected = false;
static bool             avoid_interference = true;
static uint64_t         interference_start = 0;
static bool             interference_persists = false;

static long map(long x, long in_min, long in_max, long out_min, long out_max) {
    const long run = in_max - in_min;
    const long rise = out_max - out_min;
    const long delta = x - in_min;

    return (delta * rise) / run + out_min;
}

static void update_csma_parameters() {
    int32_t airtime_pct = channel.airtime * 100;
    int32_t new_cw_band = cw.band;

    if (airtime_pct <= CSMA_BAND_1_MAX_AIRTIME) {
        new_cw_band = 1;
    } else {
        int32_t at = airtime_pct + CSMA_BAND_1_MAX_AIRTIME;

        new_cw_band = map(at, CSMA_BAND_1_MAX_AIRTIME, CSMA_BAND_N_MIN_AIRTIME, 2, CSMA_CW_BANDS);
    }

    if (new_cw_band > CSMA_CW_BANDS) {
        new_cw_band = CSMA_CW_BANDS;
    }

    if (new_cw_band != cw.band) {
        cw.band = (uint8_t)(new_cw_band);
        cw.min  = (cw.band - 1) * CSMA_CW_PER_BAND_WINDOWS;
        cw.max  = (cw.band) * CSMA_CW_PER_BAND_WINDOWS - 1;

        rnode_send_stat_csma(&cw);
    }
}

static uint16_t current_airtime_bin() {
    return (get_time() % AIRTIME_LONGTERM_MS) / AIRTIME_BINLEN_MS;
}

void csma_update_airtime() {
    uint16_t cb = current_airtime_bin();
    uint16_t pb = cb - 1;

    if (cb - 1 < 0) {
        pb = AIRTIME_BINS - 1;
    }

    uint16_t nb = cb+1;

    if (nb == AIRTIME_BINS) {
        nb = 0;
    }

    airtime_bins[nb] = 0; 
    channel.airtime = (float) (airtime_bins[cb] + airtime_bins[pb]) / (2.0*AIRTIME_BINLEN_MS);

    uint32_t longterm_airtime_sum = 0;

    for (uint16_t bin = 0; bin < AIRTIME_BINS; bin++) {
        longterm_airtime_sum += airtime_bins[bin];
    }

    channel.longterm_airtime = (float)longterm_airtime_sum / (float)AIRTIME_LONGTERM_MS;

    float longterm_channel_util_sum = 0.0;

    for (uint16_t bin = 0; bin < AIRTIME_BINS; bin++) {
        longterm_channel_util_sum += longterm_bins[bin];
    }

    channel.longterm_channel_util = (float)longterm_channel_util_sum / (float)AIRTIME_BINS;

    update_csma_parameters();
    rnode_send_stat_channel(&channel);
}

void csma_add_airtime(uint32_t ms) {
    uint16_t    cb = current_airtime_bin();
    uint16_t    nb = cb + 1;

    if (nb == AIRTIME_BINS) {
        nb = 0;
    }

    airtime_bins[cb] += ms;
    airtime_bins[nb] = 0;
}

uint32_t csma_get_cw() {
    return (random() % (cw.max - cw.min + 1) + cw.min) * csma_slot_ms;
}

void csma_get_channel(csma_channel_t *out) {
    *out = channel;
}

void csma_update_current_rssi() {
    // Only read RSSI if radio is in RX mode
    state_t radio_state = sx126x_get_state();
    if (radio_state != SX126X_RX_SINGLE && radio_state != SX126X_RX_CONTINUOUS) {
        return;
    }

    channel.current_rssi = sx126x_current_rssi();

    // Interference detection: high RSSI without carrier detection
    // Only check after noise floor is calibrated to avoid false positives
    if (noise_floor_sampled) {
        interference_detected = !dcd && (channel.current_rssi > (channel.noise_floor + CSMA_INFR_THRESHOLD_DB));
    } else {
        interference_detected = false;
    }

    // Handle potential false interference detection
    if (interference_detected && channel.current_rssi < CSMA_RFENV_RECAL_LIMIT_DB) {
        if (!interference_persists) {
            interference_persists = true;
            interference_start = get_time();
        } else {
            if (get_time() - interference_start >= CSMA_RFENV_RECAL_MS) {
                noise_floor_sampled = false;
                interference_persists = false;
            }
        }
    } else {
        interference_persists = false;
    }

    // Update noise floor (average of samples, only when no carrier detected)
    if (!dcd) {
        if (!noise_floor_sampled || channel.current_rssi < channel.noise_floor + CSMA_INFR_THRESHOLD_DB) {
            noise_floor_buffer[noise_floor_sample] = channel.current_rssi;
            noise_floor_sample = (noise_floor_sample + 1) % NOISE_FLOOR_SAMPLES;

            if (noise_floor_sample == 0) {
                noise_floor_sampled = true;
            }

            if (noise_floor_sampled) {
                int32_t sum = 0;
                for (int i = 0; i < NOISE_FLOOR_SAMPLES; i++) {
                    sum += noise_floor_buffer[i];
                }
                channel.noise_floor = sum / NOISE_FLOOR_SAMPLES;
            }
        }
    }

    // DCD-based channel utilization sampling
    util_samples[dcd_sample] = dcd;
    dcd_sample = (dcd_sample + 1) % DCD_SAMPLES;

    if (dcd_sample % UTIL_UPDATE_INTERVAL == 0) {
        int util_count = 0;
        for (int i = 0; i < DCD_SAMPLES; i++) {
            if (util_samples[i]) util_count++;
        }
        local_channel_util = (float)util_count / (float)DCD_SAMPLES;
        channel.total_channel_util = local_channel_util + channel.airtime;
        if (channel.total_channel_util > 1.0) channel.total_channel_util = 1.0;

        uint16_t cb = current_airtime_bin();
        uint16_t nb = cb + 1;
        if (nb == AIRTIME_BINS) nb = 0;

        if (channel.total_channel_util > longterm_bins[cb]) {
            longterm_bins[cb] = channel.total_channel_util;
        }
        longterm_bins[nb] = 0.0;

        csma_update_airtime();
    }
}

void csma_set_dcd(bool carrier_detected) {
    dcd = carrier_detected;
}

bool csma_should_flush() {
    return lora_should_flush;
}

bool csma_medium_free() {
    if (avoid_interference && interference_detected) return false;
    return !dcd;
}

bool csma_get_interference() {
    return interference_detected;
}

int32_t csma_get_difs_ms() {
    return difs_ms;
}

void csma_get_phy_params(float *sym_time, float *sym_rate,
                         long *preamble_syms, long *preamble_ms,
                         int32_t *slot_ms, int32_t *difs) {
    *sym_time = lora_symbol_time_ms;
    *sym_rate = lora_symbol_rate;
    *preamble_syms = lora_preamble_symbols;
    *preamble_ms = lora_preamble_time_ms;
    *slot_ms = csma_slot_ms;
    *difs = difs_ms;
}

void csma_update_radio_params(uint8_t sf, uint32_t bw) {
    if (bw == 0 || sf == 0) return;

    lora_symbol_rate = (float)bw / (float)(1 << sf);
    lora_symbol_time_ms = (1.0 / lora_symbol_rate) * 1000.0;

    uint32_t lora_bitrate = (uint32_t)(sf * (4.0 / 5.0) / ((float)(1 << sf) / ((float)bw / 1000.0)) * 1000.0);
    bool fast_rate = lora_bitrate > LORA_FAST_THRESHOLD_BPS;

    bool lora_limit_rate = lora_bitrate > LORA_LIMIT_THRESHOLD_BPS;
    bool lora_guard_rate = !lora_limit_rate && lora_bitrate > LORA_GUARD_THRESHOLD_BPS;
    lora_should_flush = !lora_limit_rate && !lora_guard_rate;

    int slot_min = CSMA_SLOT_MIN_MS;
    float preamble_target = LORA_PREAMBLE_TARGET_MS;
    if (fast_rate) {
        slot_min -= CSMA_SLOT_MIN_FAST_DELTA;
        preamble_target -= LORA_PREAMBLE_FAST_DELTA;
    }

    csma_slot_ms = (int32_t)(lora_symbol_time_ms * CSMA_SLOT_SYMBOLS);
    if (csma_slot_ms > CSMA_SLOT_MAX_MS) csma_slot_ms = CSMA_SLOT_MAX_MS;
    if (csma_slot_ms < slot_min) csma_slot_ms = slot_min;

    difs_ms = CSMA_SIFS_MS + 2 * csma_slot_ms;

    float target_preamble_symbols = preamble_target / lora_symbol_time_ms;
    if (target_preamble_symbols < LORA_PREAMBLE_SYMBOLS_MIN) {
        target_preamble_symbols = LORA_PREAMBLE_SYMBOLS_MIN;
    } else {
        target_preamble_symbols = ceilf(target_preamble_symbols);
    }

    lora_preamble_symbols = (long)target_preamble_symbols;
    lora_preamble_time_ms = (long)ceilf(lora_preamble_symbols * lora_symbol_time_ms);
}
