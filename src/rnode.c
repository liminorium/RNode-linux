/*
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  RNode Linux
 *
 *  Copyright (c) 2025 Belousov Oleg aka R1CBU
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <syslog.h>
#include <time.h>
#include <sys/random.h>

#include "rnode.h"
#include "kiss.h"
#include "util.h"
#include "queue.h"
#include "sx126x.h"
#include "csma.h"

#define SINGLE_MTU          255
#define HEADER_L            1
#define DATA_MTU            (SINGLE_MTU - HEADER_L)
#define CONFIG_QUEUE_MAX_LENGTH 16

#define RSSI_OFFSET         157

#define CMD_UNKNOWN         0xFE
#define CMD_DATA            0x00
#define CMD_FREQUENCY       0x01
#define CMD_BANDWIDTH       0x02
#define CMD_TXPOWER         0x03
#define CMD_SF              0x04
#define CMD_CR              0x05
#define CMD_RADIO_STATE     0x06
#define CMD_RADIO_LOCK      0x07
#define CMD_DETECT          0x08
#define CMD_IMPLICIT        0x09
#define CMD_LEAVE           0x0A
#define CMD_ST_ALOCK        0x0B
#define CMD_LT_ALOCK        0x0C
#define CMD_PROMISC         0x0E
#define CMD_READY           0x0F

#define CMD_STAT_RX         0x21
#define CMD_STAT_TX         0x22
#define CMD_STAT_RSSI       0x23
#define CMD_STAT_SNR        0x24
#define CMD_STAT_CHTM       0x25
#define CMD_STAT_PHYPRM     0x26
#define CMD_STAT_BAT        0x27
#define CMD_STAT_CSMA       0x28
#define CMD_STAT_TEMP       0x29
#define CMD_STAT_QUEUE      0x2A
#define CMD_BLINK           0x30
#define CMD_RANDOM          0x40
#define CMD_RSSI_OFFSET     0x4A
#define CMD_PREAMBLE        0x4B

#define CMD_FB_EXT          0x41
#define CMD_FB_READ         0x42
#define CMD_FB_WRITE        0x43
#define CMD_FB_READL        0x44
#define CMD_DISP_READ       0x66
#define CMD_DISP_INT        0x45
#define CMD_DISP_ADDR       0x63
#define CMD_DISP_BLNK       0x64
#define CMD_DISP_ROT        0x67
#define CMD_DISP_RCND       0x68
#define CMD_NP_INT          0x65
#define CMD_BT_CTRL         0x46
#define CMD_BT_UNPAIR       0x70
#define CMD_BT_PIN          0x62
#define CMD_DIS_IA          0x69
#define CMD_WIFI_MODE       0x6A
#define CMD_WIFI_SSID       0x6B
#define CMD_WIFI_PSK        0x6C
#define CMD_WIFI_CHN        0x6E
#define CMD_WIFI_IP         0x84
#define CMD_WIFI_NM         0x85

#define CMD_BOARD           0x47
#define CMD_PLATFORM        0x48
#define CMD_MCU             0x49
#define CMD_FW_VERSION      0x50
#define CMD_CFG_READ        0x6D
#define CMD_ROM_READ        0x51
#define CMD_ROM_WRITE       0x52
#define CMD_CONF_SAVE       0x53
#define CMD_CONF_DELETE     0x54
#define CMD_DEV_HASH        0x56
#define CMD_DEV_SIG         0x57
#define CMD_FW_HASH         0x58
#define CMD_HASHES          0x60
#define CMD_FW_UPD          0x61
#define CMD_UNLOCK_ROM      0x59
#define ROM_UNLOCK_BYTE     0xF8
#define CMD_RESET           0x55
#define CMD_RESET_BYTE      0xF8

#define CMD_LOG             0x80
#define CMD_TIME            0x81
#define CMD_MUX_CHAIN       0x82
#define CMD_MUX_DSCVR       0x83

#define DETECT_REQ          0x73
#define DETECT_RESP         0x46

#define RADIO_STATE_OFF     0x00
#define RADIO_STATE_ON      0x01

#define NIBBLE_SEQ          0xF0
#define NIBBLE_FLAGS        0x0F
#define FLAG_SPLIT          0x01
#define SEQ_UNSET           0xFF

static uint8_t  seq = SEQ_UNSET;
static uint8_t  buf_in[MTU];
static size_t   len_in = 0;

static uint8_t  seq_tx = SEQ_UNSET;
static uint8_t  buf_tx[SINGLE_MTU];
static size_t   len_tx = 0;

static uint32_t current_freq;
static bw_t     current_bw;
static cr_t     current_cr;
static uint8_t  current_tx_power;
static uint8_t  current_sf;
static int8_t   rssi_offset = 0;
static uint16_t preamble_length = 18;
static bool     header_implicit = false;
static bool     radio_online = false;

// Packet statistics
static uint32_t stat_rx = 0;
static uint32_t stat_tx = 0;

// Airtime lock variables (float percentages like firmware)
static float    st_airtime_limit = 0.0;
static float    lt_airtime_limit = 0.0;

/* * */

static uint32_t bw_to_hz(bw_t bw) {
    switch (bw) {
        case BW_7800:   return 7800;
        case BW_10400:  return 10400;
        case BW_15600:  return 15600;
        case BW_20800:  return 20800;
        case BW_31250:  return 31250;
        case BW_41700:  return 41700;
        case BW_62500:  return 62500;
        case BW_125000: return 125000;
        case BW_250000: return 250000;
        case BW_500000: return 500000;
        default:        return 0;
    }
}

void rnode_report_error(uint8_t error_code) {
    uint8_t ans[] = { CMD_ERROR, error_code };
    kiss_encode(ans, sizeof(ans));
    syslog(LOG_ERR, "RNode error: 0x%02X", error_code);
}

/* * */

static void ans_detect(const uint8_t *param) {
    uint8_t ans[] = { CMD_DETECT, DETECT_RESP };

    kiss_encode(ans, sizeof(ans));
}

static void ans_fw_version(const uint8_t *param) {
    uint8_t ans[] = { CMD_FW_VERSION, 1, 52 };

    kiss_encode(ans, sizeof(ans));
}

static void ans_board(const uint8_t *param) {
    uint8_t ans[] = { CMD_BOARD, 0x60 };  // 0x60 = Generic board type for Linux implementation

    kiss_encode(ans, sizeof(ans));
}

static void ans_platform(const uint8_t *param) {
    uint8_t ans[] = { CMD_PLATFORM, 0xFF };  // Custom platform for Linux

    kiss_encode(ans, sizeof(ans));
}

static void ans_mcu(const uint8_t *param) {
    uint8_t ans[] = { CMD_MCU, 0xFF };  // Not applicable for Linux

    kiss_encode(ans, sizeof(ans));
}

static void ans_frequency(const uint8_t *param) {
    uint32_t freq = (param[0] << 24) | (param[1] << 16) | (param[2] << 8) | param[3];

    if (freq != 0) {
        current_freq = freq;
        if (radio_online) {
            sx126x_set_freq(current_freq);
        }
    }

    uint8_t ans[] = { CMD_FREQUENCY, current_freq >> 24, current_freq >> 16, current_freq >> 8, current_freq };

    kiss_encode(ans, sizeof(ans));
}

static void ans_bandwidth(const uint8_t *param) {
    uint32_t bw = (param[0] << 24) | (param[1] << 16) | (param[2] << 8) | param[3];

    switch (bw) {
        case 7800:      current_bw = BW_7800;   break;
        case 10400:     current_bw = BW_10400;  break;
        case 15600:     current_bw = BW_15600;  break;
        case 20800:     current_bw = BW_20800;  break;
        case 31250:     current_bw = BW_31250;  break;
        case 41700:     current_bw = BW_41700;  break;
        case 62500:     current_bw = BW_62500;  break;
        case 125000:    current_bw = BW_125000; break;
        case 250000:    current_bw = BW_250000; break;
        case 500000:    current_bw = BW_500000; break;

        default:
            break;
    }

    switch (current_bw) {
        case BW_7800:   bw = 7800;      break;
        case BW_10400:  bw = 10400;     break;
        case BW_15600:  bw = 15600;     break;
        case BW_20800:  bw = 20800;     break;
        case BW_31250:  bw = 31250;     break;
        case BW_41700:  bw = 41700;     break;
        case BW_62500:  bw = 62500;     break;
        case BW_125000: bw = 125000;    break;
        case BW_250000: bw = 250000;    break;
        case BW_500000: bw = 500000;    break;
    }

    uint8_t ans[] = { CMD_BANDWIDTH, bw >> 24, bw >> 16, bw >> 8, bw };

    kiss_encode(ans, sizeof(ans));
    csma_update_radio_params(current_sf, bw_to_hz(current_bw));
    if (radio_online) {
        sx126x_set_lora_modulation(current_sf, current_bw, current_cr, LDRO_OFF);
    }
}

static void ans_txpower(const uint8_t *param) {
    uint8_t db = param[0];

    if (db) {
        current_tx_power = db;
        if (radio_online) {
            sx126x_set_tx_power(current_tx_power);
        }
    }

    uint8_t ans[] = { CMD_TXPOWER, current_tx_power };

    kiss_encode(ans, sizeof(ans));
}

static void ans_sf(const uint8_t *param) {
    uint8_t sf = param[0];

    if (sf) {
        current_sf = sf;
    }

    uint8_t ans[] = { CMD_SF, current_sf };

    kiss_encode(ans, sizeof(ans));
    csma_update_radio_params(current_sf, bw_to_hz(current_bw));
    if (radio_online) {
        sx126x_set_lora_modulation(current_sf, current_bw, current_cr, LDRO_OFF);
    }
}

static void ans_cr(const uint8_t *param) {
    uint8_t cr = param[0];

    switch (cr) {
        case 4: current_cr = CR_4_4; break;
        case 5: current_cr = CR_4_5; break;
        case 6: current_cr = CR_4_6; break;
        case 7: current_cr = CR_4_7; break;
        case 8: current_cr = CR_4_8; break;

        default:
            break;
    }

    switch (current_cr) {
        case CR_4_4:    cr = 4; break;
        case CR_4_5:    cr = 5; break;
        case CR_4_6:    cr = 6; break;
        case CR_4_7:    cr = 7; break;
        case CR_4_8:    cr = 8; break;
    }

    uint8_t ans[] = { CMD_CR, cr };

    kiss_encode(ans, sizeof(ans));
    if (radio_online) {
        sx126x_set_lora_modulation(current_sf, current_bw, current_cr, LDRO_OFF);
    }
}

static void ans_radio_state(const uint8_t *param) {
    if (param[0] == 0xFF) {
        /* Query current state */
    } else if (param[0] == 0x01) {
        rnode_start();
        radio_online = true;
        syslog(LOG_INFO, "Radio on");
    } else if (param[0] == 0x00) {
        radio_online = false;
        syslog(LOG_INFO, "Radio off");
    }

    uint8_t ans[] = { CMD_RADIO_STATE, radio_online ? 0x01 : 0x00 };

    kiss_encode(ans, sizeof(ans));
}

static void ans_rssi_offset(const uint8_t *param) {
    if (param[0] != 0) {
        rssi_offset = (int8_t)param[0];
        syslog(LOG_INFO, "RSSI offset set to %d", rssi_offset);
    }

    uint8_t ans[] = { CMD_RSSI_OFFSET, (uint8_t)rssi_offset };

    kiss_encode(ans, sizeof(ans));
}

static void ans_preamble(const uint8_t *param) {
    uint16_t preamble = (param[0] << 8) | param[1];

    if (preamble != 0) {
        preamble_length = preamble;
        syslog(LOG_INFO, "Preamble length set to %d", preamble_length);
        if (radio_online) {
            sx126x_set_lora_packet(header_implicit ? HEADER_IMPLICIT : HEADER_EXPLICIT,
                                   preamble_length, 15, CRC_ON);
        }
    }

    uint8_t ans[] = { CMD_PREAMBLE, preamble_length >> 8, preamble_length & 0xFF };

    kiss_encode(ans, sizeof(ans));
}

static void ans_implicit(const uint8_t *param) {
    header_implicit = param[0] ? true : false;
    syslog(LOG_INFO, "Header mode set to %s", header_implicit ? "implicit" : "explicit");

    if (radio_online) {
        sx126x_set_lora_packet(header_implicit ? HEADER_IMPLICIT : HEADER_EXPLICIT,
                               preamble_length, 15, CRC_ON);
    }

    uint8_t ans[] = { CMD_IMPLICIT, header_implicit ? 0x01 : 0x00 };

    kiss_encode(ans, sizeof(ans));
}

static void ans_random(const uint8_t *param) {
    uint8_t ans[17];
    ans[0] = CMD_RANDOM;

    if (getrandom(&ans[1], 16, 0) != 16) {
        FILE *f = fopen("/dev/urandom", "rb");
        if (f) {
            fread(&ans[1], 1, 16, f);
            fclose(f);
        }
    }

    kiss_encode(ans, sizeof(ans));
}

static void ans_stat_queue(const uint8_t *param) {
    uint16_t depth = queue_get_depth();
    uint8_t ans[] = { CMD_STAT_QUEUE, depth >> 8, depth & 0xFF };
    kiss_encode(ans, sizeof(ans));
}

static void ans_stat_chtm(const uint8_t *param) {
    csma_channel_t ch;
    csma_get_channel(&ch);
    rnode_send_stat_channel(&ch);
}

static void ans_stat_phyprm(const uint8_t *param) {
    float sym_time, sym_rate;
    long preamble_syms, preamble_ms;
    int32_t slot_ms, difs;

    csma_get_phy_params(&sym_time, &sym_rate, &preamble_syms, &preamble_ms, &slot_ms, &difs);

    uint16_t st = (uint16_t)(sym_time * 1000);
    uint16_t sr = (uint16_t)(sym_rate);
    uint16_t ps = (uint16_t)(preamble_syms);
    uint16_t pt = (uint16_t)(preamble_ms);
    uint16_t sm = (uint16_t)(slot_ms);
    uint16_t df = (uint16_t)(difs);

    uint8_t ans[] = {
        CMD_STAT_PHYPRM,
        st >> 8, st,
        sr >> 8, sr,
        ps >> 8, ps,
        pt >> 8, pt,
        sm >> 8, sm,
        df >> 8, df
    };

    kiss_encode(ans, sizeof(ans));
}

static void ans_radio_lock(const uint8_t *param) {
    // Always unlocked on Linux (no EEPROM lock mechanism)
    uint8_t ans[] = { CMD_RADIO_LOCK, 0x00 };
    kiss_encode(ans, sizeof(ans));
}

static bool promisc = false;

static void ans_promisc(const uint8_t *param) {
    promisc = param[0] ? true : false;
    syslog(LOG_INFO, "Promiscuous mode %s", promisc ? "enabled" : "disabled");
    uint8_t ans[] = { CMD_PROMISC, promisc ? 0x01 : 0x00 };
    kiss_encode(ans, sizeof(ans));
}

static void ans_ready(const uint8_t *param) {
    bool ready = queue_get_depth() < CONFIG_QUEUE_MAX_LENGTH;
    uint8_t ans[] = { CMD_READY, ready ? 0x01 : 0x00 };
    kiss_encode(ans, sizeof(ans));
}

static void ans_reset(const uint8_t *param) {
    if (param[0] == CMD_RESET_BYTE) {
        syslog(LOG_INFO, "Radio reset requested");
        rnode_start();
        uint8_t ans[] = { CMD_RESET, 0x00 };
        kiss_encode(ans, sizeof(ans));
    }
}

static void ans_st_alock(const uint8_t *param) {
    uint16_t at = (param[0] << 8) | param[1];

    if (at == 0) {
        st_airtime_limit = 0.0;
    } else {
        st_airtime_limit = (float)at / (100.0 * 100.0);
        if (st_airtime_limit >= 1.0) { st_airtime_limit = 0.0; }
    }

    uint16_t at_out = (uint16_t)(st_airtime_limit * 100 * 100);
    uint8_t ans[] = { CMD_ST_ALOCK, at_out >> 8, at_out & 0xFF };
    kiss_encode(ans, sizeof(ans));

    syslog(LOG_INFO, "Short-term airtime lock set to %.2f%%", st_airtime_limit * 100.0);
}

static void ans_lt_alock(const uint8_t *param) {
    uint16_t at = (param[0] << 8) | param[1];

    if (at == 0) {
        lt_airtime_limit = 0.0;
    } else {
        lt_airtime_limit = (float)at / (100.0 * 100.0);
        if (lt_airtime_limit >= 1.0) { lt_airtime_limit = 0.0; }
    }

    uint16_t at_out = (uint16_t)(lt_airtime_limit * 100 * 100);
    uint8_t ans[] = { CMD_LT_ALOCK, at_out >> 8, at_out & 0xFF };
    kiss_encode(ans, sizeof(ans));

    syslog(LOG_INFO, "Long-term airtime lock set to %.2f%%", lt_airtime_limit * 100.0);
}

static void ans_stat_rx(const uint8_t *param) {
    uint8_t ans[] = { CMD_STAT_RX,
        (stat_rx >> 24) & 0xFF, (stat_rx >> 16) & 0xFF,
        (stat_rx >> 8) & 0xFF, stat_rx & 0xFF };
    kiss_encode(ans, sizeof(ans));
}

static void ans_stat_tx(const uint8_t *param) {
    uint8_t ans[] = { CMD_STAT_TX,
        (stat_tx >> 24) & 0xFF, (stat_tx >> 16) & 0xFF,
        (stat_tx >> 8) & 0xFF, stat_tx & 0xFF };
    kiss_encode(ans, sizeof(ans));
}

static void ans_stat_bat(const uint8_t *param) {
    /* Battery state: unknown on Linux */
    uint8_t ans[] = { CMD_STAT_BAT, 0x00, 0x00 };
    kiss_encode(ans, sizeof(ans));
}

/* ---- Minimal device hash (SHA-256 of /etc/machine-id) ---- */

static const uint32_t sha256_K[64] = {
    0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,
    0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,
    0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,
    0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,
    0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,
    0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,
    0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,
    0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2
};

#define SHA256_ROTR(x,n)  (((x) >> (n)) | ((x) << (32 - (n))))
#define SHA256_CH(x,y,z)  (((x) & (y)) ^ (~(x) & (z)))
#define SHA256_MAJ(x,y,z) (((x) & (y)) ^ ((x) & (z)) ^ ((y) & (z)))
#define SHA256_EP0(x)     (SHA256_ROTR(x,2) ^ SHA256_ROTR(x,13) ^ SHA256_ROTR(x,22))
#define SHA256_EP1(x)     (SHA256_ROTR(x,6) ^ SHA256_ROTR(x,11) ^ SHA256_ROTR(x,25))
#define SHA256_SIG0(x)    (SHA256_ROTR(x,7) ^ SHA256_ROTR(x,18) ^ ((x) >> 3))
#define SHA256_SIG1(x)    (SHA256_ROTR(x,17) ^ SHA256_ROTR(x,19) ^ ((x) >> 10))

static void sha256_compute(const uint8_t *data, size_t len, uint8_t out[32]) {
    uint32_t h[8] = {
        0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
        0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19
    };

    size_t padded_len = ((len + 8) / 64 + 1) * 64;
    uint8_t *msg = calloc(padded_len, 1);
    memcpy(msg, data, len);
    msg[len] = 0x80;

    uint64_t bit_len = (uint64_t)len * 8;
    for (int i = 0; i < 8; i++)
        msg[padded_len - 1 - i] = (uint8_t)(bit_len >> (i * 8));

    for (size_t offset = 0; offset < padded_len; offset += 64) {
        uint32_t w[64];
        for (int i = 0; i < 16; i++)
            w[i] = ((uint32_t)msg[offset + 4*i] << 24) |
                   ((uint32_t)msg[offset + 4*i+1] << 16) |
                   ((uint32_t)msg[offset + 4*i+2] << 8) |
                   ((uint32_t)msg[offset + 4*i+3]);
        for (int i = 16; i < 64; i++)
            w[i] = SHA256_SIG1(w[i-2]) + w[i-7] + SHA256_SIG0(w[i-15]) + w[i-16];

        uint32_t a=h[0], b=h[1], c=h[2], d=h[3], e=h[4], f=h[5], g=h[6], hh=h[7];

        for (int i = 0; i < 64; i++) {
            uint32_t t1 = hh + SHA256_EP1(e) + SHA256_CH(e,f,g) + sha256_K[i] + w[i];
            uint32_t t2 = SHA256_EP0(a) + SHA256_MAJ(a,b,c);
            hh=g; g=f; f=e; e=d+t1; d=c; c=b; b=a; a=t1+t2;
        }

        h[0]+=a; h[1]+=b; h[2]+=c; h[3]+=d; h[4]+=e; h[5]+=f; h[6]+=g; h[7]+=hh;
    }

    free(msg);
    for (int i = 0; i < 8; i++) {
        out[4*i]   = (h[i] >> 24) & 0xFF;
        out[4*i+1] = (h[i] >> 16) & 0xFF;
        out[4*i+2] = (h[i] >> 8) & 0xFF;
        out[4*i+3] = h[i] & 0xFF;
    }
}

#define DEV_HASH_LEN 32
static uint8_t dev_hash[DEV_HASH_LEN];
static bool dev_hash_ready = false;

static void compute_dev_hash(void) {
    uint8_t machine_id[16];
    memset(machine_id, 0, sizeof(machine_id));

    FILE *f = fopen("/etc/machine-id", "r");
    if (f) {
        char hex[33] = {0};
        if (fread(hex, 1, 32, f) == 32) {
            for (int i = 0; i < 16; i++) {
                unsigned int val;
                if (sscanf(hex + 2*i, "%2x", &val) == 1) {
                    machine_id[i] = (uint8_t)val;
                }
            }
        }
        fclose(f);
    } else {
        syslog(LOG_WARNING, "/etc/machine-id not found, device hash will be zeros");
    }

    sha256_compute(machine_id, sizeof(machine_id), dev_hash);
    dev_hash_ready = true;
}

static void ans_dev_hash(const uint8_t *param) {
    if (!dev_hash_ready) compute_dev_hash();
    uint8_t ans[1 + DEV_HASH_LEN];
    ans[0] = CMD_DEV_HASH;
    memcpy(&ans[1], dev_hash, DEV_HASH_LEN);
    kiss_encode(ans, sizeof(ans));
}

/* * */

/* Airtime lock check - uses CSMA channel airtime tracking like firmware */
bool rnode_check_airtime_lock() {
    if (st_airtime_limit == 0.0 && lt_airtime_limit == 0.0) {
        return false;  /* not locked */
    }

    csma_channel_t ch;
    csma_get_channel(&ch);

    if (st_airtime_limit != 0.0 && ch.airtime >= st_airtime_limit) {
        return true;  /* locked */
    }
    if (lt_airtime_limit != 0.0 && ch.longterm_airtime >= lt_airtime_limit) {
        return true;  /* locked */
    }

    return false;  /* not locked */
}

/* * */

void rnode_start() {
    if (!sx126x_begin()) {
        rnode_report_error(ERROR_INITRADIO);
        return;
    }

    sx126x_set_freq(current_freq);
    sx126x_set_tx_power(current_tx_power);

    sx126x_set_lora_modulation(current_sf, current_bw, current_cr, LDRO_OFF);
    sx126x_set_lora_packet(header_implicit ? HEADER_IMPLICIT : HEADER_EXPLICIT, preamble_length, 15, CRC_ON);
    sx126x_set_sync_word(0x1424);

    sx126x_request(RX_CONTINUOUS);

    uint32_t header_ms;
    uint32_t data_ms;

    sx126x_air_time(255, &header_ms, &data_ms);

    queue_set_busy_timeout(header_ms * 3 / 2, data_ms * 3 / 2);
}

void rnode_from_channel(const uint8_t *buf, size_t len) {
    uint8_t cmd = buf[0];

    buf++;
    len--;

    switch (cmd) {
        case CMD_DETECT:
            ans_detect(buf);
            break;

        case CMD_FW_VERSION:
            ans_fw_version(buf);
            break;

        case CMD_BOARD:
            ans_board(buf);
            break;

        case CMD_PLATFORM:
            ans_platform(buf);
            break;

        case CMD_MCU:
            ans_mcu(buf);
            break;

        case CMD_FREQUENCY:
            ans_frequency(buf);
            break;

        case CMD_BANDWIDTH:
            ans_bandwidth(buf);
            break;

        case CMD_TXPOWER:
            ans_txpower(buf);
            break;

        case CMD_SF:
            ans_sf(buf);
            break;

        case CMD_CR:
            ans_cr(buf);
            break;

        case CMD_RADIO_STATE:
            ans_radio_state(buf);
            break;

        case CMD_RSSI_OFFSET:
            ans_rssi_offset(buf);
            break;

        case CMD_PREAMBLE:
            ans_preamble(buf);
            break;

        case CMD_IMPLICIT:
            ans_implicit(buf);
            break;

        case CMD_ST_ALOCK:
            ans_st_alock(buf);
            break;

        case CMD_LT_ALOCK:
            ans_lt_alock(buf);
            break;

        case CMD_RANDOM:
            ans_random(buf);
            break;

        case CMD_STAT_QUEUE:
            ans_stat_queue(buf);
            break;

        case CMD_STAT_CHTM:
            ans_stat_chtm(buf);
            break;

        case CMD_STAT_PHYPRM:
            ans_stat_phyprm(buf);
            break;

        case CMD_RADIO_LOCK:
            ans_radio_lock(buf);
            break;

        case CMD_PROMISC:
            ans_promisc(buf);
            break;

        case CMD_READY:
            ans_ready(buf);
            break;

        case CMD_RESET:
            ans_reset(buf);
            break;

        case CMD_STAT_RX:
            ans_stat_rx(buf);
            break;

        case CMD_STAT_TX:
            ans_stat_tx(buf);
            break;

        case CMD_STAT_BAT:
            ans_stat_bat(buf);
            break;

        case CMD_DATA:
            queue_push(buf, len);
            break;

        case CMD_LEAVE:
            syslog(LOG_INFO, "CMD_LEAVE received");
            break;

        case CMD_BLINK:
        case CMD_FB_EXT:
        case CMD_FB_READ:
        case CMD_FB_WRITE:
        case CMD_FB_READL:
        case CMD_DISP_INT:
        case CMD_DISP_ADDR:
        case CMD_DISP_BLNK:
        case CMD_DISP_ROT:
        case CMD_DISP_RCND:
        case CMD_DISP_READ:
        case CMD_NP_INT:
        case CMD_BT_CTRL:
        case CMD_BT_UNPAIR:
        case CMD_BT_PIN:
        case CMD_DIS_IA:
        case CMD_WIFI_MODE:
        case CMD_WIFI_SSID:
        case CMD_WIFI_PSK:
        case CMD_WIFI_CHN:
        case CMD_WIFI_IP:
        case CMD_WIFI_NM:
        case CMD_DEV_HASH:
            ans_dev_hash(buf);
            break;

        case CMD_CONF_SAVE:
        case CMD_CONF_DELETE:
        case CMD_ROM_READ:
        case CMD_ROM_WRITE:
        case CMD_CFG_READ:
        case CMD_DEV_SIG:
        case CMD_FW_HASH:
        case CMD_HASHES:
        case CMD_FW_UPD:
        case CMD_UNLOCK_ROM:
        case CMD_LOG:
        case CMD_TIME:
        case CMD_MUX_CHAIN:
        case CMD_MUX_DSCVR:
            /* Not applicable on Linux — silently ignore */
            break;

        default:
            syslog(LOG_WARNING, "RNode unknown command: 0x%02X", cmd);
            rnode_report_error(ERROR_INVALID_COMMAND);
            break;
    }
}

void rnode_signal_stat(uint8_t rssi, int8_t snr, uint8_t signal_rssi) {
    // Apply RSSI offset calibration
    rssi = rssi + rssi_offset;
    signal_rssi = signal_rssi + rssi_offset;

    uint8_t ans_rssi[] = { CMD_STAT_RSSI, rssi };
    uint8_t ans_snr[] = { CMD_STAT_SNR, snr };

    kiss_encode(ans_rssi, sizeof(ans_rssi));
    kiss_encode(ans_snr, sizeof(ans_snr));
}

static void append_buf(const uint8_t *buf, size_t len) {
    memcpy(&buf_in[len_in], buf, len);
    len_in += len;
}

void rnode_from_air(const uint8_t *buf, size_t len) {
    if (promisc) {
        /* In promiscuous mode, pass raw data without header parsing */
        uint8_t buf_kiss[len + 1];
        buf_kiss[0] = CMD_DATA;
        memcpy(&buf_kiss[1], buf, len);
        kiss_encode(buf_kiss, len + 1);
        stat_rx++;
        return;
    }

    /* The standard operating mode allows large */
    /* packets with a payload up to 500 bytes,  */
    /* by combining two raw LoRa packets.       */
    /* We read the 1-byte header and extract    */
    /* packet sequence number and split flags   */

    uint8_t header = *buf;
    bool    split = header & FLAG_SPLIT;
    uint8_t sequence = header >> 4;
    bool    ready = false;

    buf++;
    len--;

    if (split) {
        if (seq == SEQ_UNSET) {
            /* This is the first part of a split    */
            /* packet, so we set the seq variable   */
            /* and add the data to the buffer       */

            len_in = 0;
            append_buf(buf, len);
            seq = sequence;
        } else if (seq == sequence) {
            /* This is the second part of a split   */
            /* packet, so we add it to the buffer   */
            /* and set ready flag                   */

            append_buf(buf, len);
            seq = SEQ_UNSET;
            ready = true;
        } else {
            /* This split packet does not carry the */
            /* same sequence id, so we must assume  */
            /* that we are seeing the first part of */
            /* a new split packet.                  */

            len_in = 0;
            append_buf(buf, len);
            seq = sequence;
        }
    } else {
        /* This is not a split packet, so we    */
        /* just read it and set the ready       */
        /* flag to true.                        */

        if (seq != SEQ_UNSET) {
            /* If we already had part of a split    */
            /* packet in the buffer, we clear it.   */

            len_in = 0;
            seq = SEQ_UNSET;
        }

        append_buf(buf, len);
        ready = true;
    }

    if (ready) {
        uint8_t buf_kiss[len_in + 1];

        buf_kiss[0] = CMD_DATA;
        memcpy(&buf_kiss[1], buf_in, len_in);
        kiss_encode(buf_kiss, len_in + 1);

        len_in = 0;
        stat_rx++;
    }
}

static void tx_buf(const uint8_t *buf, size_t len, uint8_t flag) {
    syslog(LOG_INFO, "TX buf (%i bytes)", len);

    uint8_t buf_air[len + HEADER_L];

    buf_air[0] = seq_tx | flag;
    memcpy(&buf_air[1], buf, len);

    sx126x_begin_packet();
    sx126x_write(buf_air, len + HEADER_L);
    sx126x_end_packet();

    syslog(LOG_INFO, "TX buf done");
}

uint32_t rnode_to_air(const uint8_t *buf, size_t len) {
    uint32_t air_time;

    if (promisc) {
        /* In promiscuous mode, send raw data without header/split */
        sx126x_begin_packet();
        sx126x_write(buf, len);
        sx126x_end_packet();

        air_time = sx126x_air_time(len, NULL, NULL);
        csma_add_airtime(air_time);
        stat_tx++;
        return air_time;
    }

    seq_tx = random() & 0xF0;

    if (len <= DATA_MTU) {
        /* Everything fit into one packet */

        tx_buf(buf, len, 0);
        len_tx = 0;

        air_time = sx126x_air_time(len, NULL, NULL);
    } else {
        /* It didn't fit. Save tail... */

        len_tx = len - DATA_MTU;
        memcpy(buf_tx, &buf[DATA_MTU], len_tx);
        air_time = sx126x_air_time(len_tx, NULL, NULL);

        /*  ...and sending the first part */

        tx_buf(buf, DATA_MTU, FLAG_SPLIT);
        air_time += sx126x_air_time(DATA_MTU, NULL, NULL);
    }

    csma_add_airtime(air_time);
    stat_tx++;

    return air_time;
}

void rnode_tx_done() {
    if (len_tx) {
        /* There is an unsent tail, sending it */

        tx_buf(buf_tx, len_tx, FLAG_SPLIT);
        len_tx = 0;
    } else {
        sx126x_request(RX_CONTINUOUS);
    }
}

void rnode_rx_done(uint16_t len) {
    uint8_t buf[len];

    if (len > 0) {
        uint8_t rssi, signal_rssi;
        int8_t snr;

        sx126x_packet_signal_raw(&rssi, &snr, &signal_rssi);
        sx126x_read(buf, len);

        rnode_signal_stat(rssi, snr, signal_rssi);
        rnode_from_air(buf, len);
    }
}

void rnode_send_stat_csma(csma_cw_t *cw) {
    uint8_t ans[] = { CMD_STAT_CSMA, cw->band, cw->min, cw->max };

    kiss_encode(ans, sizeof(ans));
}

void rnode_indicate_reset() {
    uint8_t ans[] = { CMD_RESET, CMD_RESET_BYTE };
    kiss_encode(ans, sizeof(ans));
}

void rnode_send_stat_queue(uint16_t depth) {
    uint8_t ans[] = { CMD_STAT_QUEUE, depth >> 8, depth & 0xFF };
    kiss_encode(ans, sizeof(ans));
}

void rnode_send_stat_channel(csma_channel_t *channel) {
    uint16_t ats = (uint16_t) (channel->airtime * 100 * 100);
    uint16_t atl = (uint16_t) (channel->longterm_airtime * 100 * 100);
    uint16_t cls = (uint16_t) (channel->total_channel_util * 100 * 100);
    uint16_t cll = (uint16_t) (channel->longterm_channel_util * 100 * 100);
    uint8_t  crs = (uint8_t) (channel->current_rssi);
    uint8_t  nfl = (uint8_t) (channel->noise_floor);
    uint8_t  ntf = 0xFF;

    if (csma_get_interference()) {
        ntf = (uint8_t) (channel->current_rssi + rssi_offset);
    }

    uint8_t ans[] = { CMD_STAT_CHTM, ats >> 8, ats, atl >> 8, atl, cls >> 8, cls, cll >> 8, cll, crs, nfl, ntf };

    kiss_encode(ans, sizeof(ans));
}
