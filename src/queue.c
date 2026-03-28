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

// DIFS + CW listen-before-talk: matches reference firmware tx_queue_handler()
// 1. DIFS wait: medium must stay free for difs_ms
// 2. CW wait: medium must stay free for random contention window
// If medium goes busy at any point, DIFS restarts but CW progress is preserved
static uint64_t         medium_free_since = 0;
static uint64_t         cw_wait_start = 0;
static uint64_t         cw_wait_passed = 0;
static uint64_t         cw_target = 0;
static bool             cw_picked = false;
static bool             flushing = false;

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

static void * queue_worker(void *p) {
    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);

    while (true) {
        uint64_t now = get_time();

        if (sx126x_get_state() != SX126X_TX) {
            if (tx_enable && head != NULL) {
                if (flushing) {
                    // Flush mode: send all queued packets back-to-back after one DIFS+CW
                    if (!send_packet()) {
                        flushing = false;
                    } else if (head == NULL) {
                        flushing = false;
                    }
                } else {
                    // Normal mode: DIFS+CW listen-before-talk
                    // Pick CW target once per CSMA cycle
                    if (!cw_picked) {
                        cw_target = csma_get_cw();
                        cw_picked = true;
                    }

                    if (csma_medium_free()) {
                        if (medium_free_since == 0) {
                            // Medium just became free, start DIFS timer
                            medium_free_since = now;
                        } else if (now - medium_free_since < (uint64_t)csma_get_difs_ms()) {
                            // Still in DIFS wait, keep waiting
                        } else {
                            // DIFS passed, now in CW wait
                            if (cw_wait_start == 0) {
                                cw_wait_start = now;
                            } else {
                                cw_wait_passed += now - cw_wait_start;
                                cw_wait_start = now;
                            }

                            if (cw_wait_passed >= cw_target) {
                                // CW passed, transmit
                                medium_free_since = 0;
                                cw_wait_start = 0;
                                cw_wait_passed = 0;
                                cw_picked = false;

                                if (csma_should_flush()) {
                                    flushing = true;
                                }
                                send_packet();
                            }
                        }
                    } else {
                        // Medium busy: restart DIFS, pause CW (but preserve cw_wait_passed)
                        medium_free_since = 0;
                        cw_wait_start = 0;
                    }
                }
            } else if (!tx_enable) {
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
            flushing = false;
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
