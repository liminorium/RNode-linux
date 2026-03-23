/*
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  RNode Linux
 *
 *  Copyright (c) 2025 Belousov Oleg aka R1CBU
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define STATUS_INTERVAL_MS  3

typedef struct {
    uint8_t band;
    uint8_t min;
    uint8_t max;
} csma_cw_t;

typedef struct {
    float   airtime;
    float   longterm_airtime;
    float   total_channel_util;
    float   longterm_channel_util;
    int32_t current_rssi;
    int32_t noise_floor;
} csma_channel_t;

void csma_update_airtime();
void csma_update_current_rssi();
void csma_add_airtime(uint32_t ms);
uint32_t csma_get_cw();
void csma_get_channel(csma_channel_t *out);

void csma_set_dcd(bool carrier_detected);
bool csma_medium_free();
bool csma_get_interference();
int32_t csma_get_difs_ms();

void csma_get_phy_params(float *sym_time, float *sym_rate,
                         long *preamble_syms, long *preamble_ms,
                         int32_t *slot_ms, int32_t *difs);
void csma_update_radio_params(uint8_t sf, uint32_t bw);
