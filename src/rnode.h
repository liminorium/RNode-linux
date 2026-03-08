/*
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  RNode Linux
 *
 *  Copyright (c) 2025 Belousov Oleg aka R1CBU
 */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include "csma.h"

#define MTU 1024

#define CMD_ERROR             0x90
#define ERROR_NONE            0x00
#define ERROR_INITRADIO       0x01
#define ERROR_TXFAILED        0x02
#define ERROR_EEPROM_LOCKED   0x03
#define ERROR_QUEUE_FULL      0x04
#define ERROR_MEMORY_LOW      0x05
#define ERROR_MODEM_TIMEOUT   0x06
#define ERROR_INVALID_COMMAND 0x07

void rnode_start();

void rnode_from_channel(const uint8_t *buf, size_t len);

void rnode_signal_stat(uint8_t rssi, int8_t snr, uint8_t signal_rssi);
void rnode_from_air(const uint8_t *buf, size_t len);
uint32_t rnode_to_air(const uint8_t *buf, size_t len);

void rnode_tx_done();
void rnode_rx_done(uint16_t len);

void rnode_send_stat_csma(csma_cw_t *cw);
void rnode_send_stat_channel(csma_channel_t *channel);
void rnode_send_stat_queue(uint16_t depth);

void rnode_report_error(uint8_t error_code);

bool rnode_check_airtime_lock(uint32_t packet_airtime);
void rnode_update_airtime_usage(uint32_t packet_airtime);
