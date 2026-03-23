/*
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  RNode Linux
 *
 *  Copyright (c) 2025 Belousov Oleg aka R1CBU
 */

#include <pthread.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <syslog.h>

#include "queue.h"
#include "util.h"
#include "rnode.h"
#include "csma.h"
#include "sx126x.h"

typedef struct item_t {
    uint8_t         *data;
    size_t          len;
    struct item_t   *next;
} item_t;

static struct item_t    *head = NULL;
static struct item_t    *tail = NULL;
static pthread_mutex_t  mux;
static uint16_t         queue_depth = 0;

static bool             tx_enable = false;
static uint32_t         tx_header_timeout;
static uint32_t         tx_data_timeout;
static uint64_t         tx_disabled = UINT64_MAX;
static uint64_t         tx_wait_timeout;
static uint64_t         rssi_delay;

// DIFS+CW state machine
static bool             difs_passed = false;
static uint64_t         difs_wait_start = 0;
static bool             cw_passed = false;
static uint64_t         cw_wait_start = 0;
static uint32_t         cw_wait_target = 0;

static bool send_packet() {
    bool res = false;

    if (head != NULL) {
        pthread_mutex_lock(&mux);

        item_t *item = head;

        pthread_mutex_unlock(&mux);

        // Check if airtime locks allow transmission
        if (rnode_check_airtime_lock()) {
            syslog(LOG_INFO, "Airtime lock: deferring transmission");
            return false;  // Don't remove from queue, try again later
        }

        syslog(LOG_INFO, "Queue: pop to air (%i)", item->len);
        uint32_t actual_airtime = rnode_to_air(item->data, item->len);
        tx_wait_timeout = get_time() + actual_airtime * 2;

        pthread_mutex_lock(&mux);

        if (head == tail) {
            head = tail  = NULL;
        } else {
            head = head->next;
        }

        free(item->data);
        free(item);

        queue_depth--;
        uint16_t depth = queue_depth;

        res = true;

        pthread_mutex_unlock(&mux);
        rnode_send_stat_queue(depth);
        csma_update_airtime();
    }

    return res;
}

static void csma_reset_state() {
    difs_passed = false;
    difs_wait_start = 0;
    cw_passed = false;
    cw_wait_start = 0;
    cw_wait_target = 0;
}

static void * queue_worker(void *p) {
    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);

    while (true) {
        uint64_t now = get_time();

        if (sx126x_get_state() != SX126X_TX) {
            if (tx_enable && head != NULL) {
                if (csma_medium_free()) {
                    // DIFS wait
                    if (!difs_passed) {
                        if (difs_wait_start == 0) {
                            difs_wait_start = now;
                        } else if (now - difs_wait_start >= (uint64_t)csma_get_difs_ms()) {
                            difs_passed = true;
                            difs_wait_start = 0;
                        }
                    }

                    // CW wait (only after DIFS passed)
                    if (difs_passed && !cw_passed) {
                        if (cw_wait_start == 0) {
                            cw_wait_target = csma_get_cw();
                            cw_wait_start = now;
                        } else if (now - cw_wait_start >= cw_wait_target) {
                            cw_passed = true;
                        }
                    }

                    // Transmit (after both DIFS and CW passed)
                    if (difs_passed && cw_passed) {
                        csma_reset_state();
                        send_packet();
                    }
                } else {
                    // Medium busy — reset CSMA state
                    csma_reset_state();
                }
            } else if (!tx_enable) {
                csma_reset_state();

                if (now >= tx_disabled) {
                    syslog(LOG_DEBUG, "False preamble timeout - re-enabling TX");
                    tx_enable = true;
                    csma_set_dcd(false);
                }
            }

            // Periodically update RSSI
            if (now > rssi_delay) {
                rssi_delay = now + 25;
                csma_update_current_rssi();
            }
        } else if (now > tx_wait_timeout) {
            syslog(LOG_WARNING, "TX wait timeout! Radio restart");
            rnode_start();
        }

        usleep(1000);
    }
}

void queue_init() {

    pthread_mutex_init(&mux, NULL);

    pthread_t thread;

    pthread_create(&thread, NULL, queue_worker, NULL);
    pthread_detach(thread);
}

void queue_set_busy_timeout(uint32_t header_ms, uint32_t data_ms) {
    tx_header_timeout = header_ms;
    tx_data_timeout = data_ms;

    syslog(LOG_INFO, "Maximum medium busy %i ms + %i ms", tx_header_timeout, tx_data_timeout);
}

void queue_push(const uint8_t *buf, size_t len) {
    syslog(LOG_INFO, "Queue: push (%i bytes)", len);

    item_t *item = malloc(sizeof(item_t));

    item->data = malloc(len);
    item->len = len;

    memcpy(item->data, buf, len);

    pthread_mutex_lock(&mux);

    if (head == NULL && tail == NULL) {
        head = tail = item;
    } else {
        tail->next = item;
        tail = item;
    }

    queue_depth++;
    uint16_t depth = queue_depth;

    pthread_mutex_unlock(&mux);
    rnode_send_stat_queue(depth);
}


void queue_medium_state(cause_medium_t cause) {
    uint64_t    now = get_time();

    switch(cause) {
        case CAUSE_INIT:
        case CAUSE_TX_DONE:
            csma_set_dcd(false);
            tx_enable = true;
            break;

        case CAUSE_RX_DONE:
        case CAUSE_HEADER_ERR:
            csma_set_dcd(false);
            tx_enable = true;
            break;

        case CAUSE_PREAMBLE_DETECTED:
            csma_set_dcd(true);
            tx_enable = false;
            tx_disabled = now + tx_header_timeout;
            break;

        case CAUSE_HEADER_VALID:
            csma_set_dcd(true);
            tx_enable = false;
            tx_disabled = now + tx_data_timeout;
            break;
    }
}

uint16_t queue_get_depth() {
    pthread_mutex_lock(&mux);
    uint16_t depth = queue_depth;
    pthread_mutex_unlock(&mux);
    return depth;
}
