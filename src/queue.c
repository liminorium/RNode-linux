/*
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  RNode Linux
 *
 *  Copyright (c) 2025 Belousov Oleg aka R1CBU
 */

#include <pthread.h>
#include <stdlib.h>
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

static bool             tx_enable = false;
static uint64_t         tx_delay;
static uint32_t         tx_header_timeout;
static uint32_t         tx_data_timeout;
static uint64_t         tx_disabled;
static uint64_t         tx_wait_timeout;
static uint64_t         rssi_delay;

static bool send_packet() {
    bool res = false;

    if (head != NULL) {
        pthread_mutex_lock(&mux);

        item_t *item = head;

        pthread_mutex_unlock(&mux);

        // Calculate airtime for this packet
        uint32_t airtime = sx126x_air_time(item->len, NULL, NULL);
        
        // Check if airtime locks allow transmission
        if (!rnode_check_airtime_lock(airtime)) {
            syslog(LOG_INFO, "Airtime lock: deferring transmission");
            return false;  // Don't remove from queue, try again later
        }

        syslog(LOG_INFO, "Queue: pop to air (%i)", item->len);
        uint32_t actual_airtime = rnode_to_air(item->data, item->len);
        tx_wait_timeout = get_time() + actual_airtime * 2;
        
        // Update airtime usage after successful transmission
        rnode_update_airtime_usage(actual_airtime);

        pthread_mutex_lock(&mux);

        if (head == tail) {
            head = tail  = NULL;
        } else {
            head = head->next;
        }

        free(item->data);
        free(item);

        res = true;

        pthread_mutex_unlock(&mux);
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
            if (tx_enable) {
                if (now >= tx_delay) {
                    uint32_t delay = csma_get_cw();

                    send_packet();
                    tx_delay = now + delay;
                } else if (now > rssi_delay) {
                    rssi_delay = now + 1000;

                    csma_update_current_rssi();
                }
            } else {
                if (now >= tx_disabled) {
                    // TX was disabled (waiting for RX) but timed out
                    // This can happen with false preamble detections or interference
                    // Just re-enable TX instead of restarting the radio
                    syslog(LOG_DEBUG, "TX disabled timeout - re-enabling (likely false preamble detection)");
                    tx_enable = true;
                    tx_delay = now + csma_get_cw();
                }
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

    pthread_mutex_unlock(&mux);
}

void queue_medium_state(cause_medium_t cause) {
    uint64_t    now = get_time();

    switch(cause) {
        case CAUSE_INIT:
        case CAUSE_TX_DONE:
            tx_enable = true;
            tx_delay = now;
            break;

        case CAUSE_RX_DONE:
        case CAUSE_HEADER_ERR:
            tx_enable = true;
            tx_delay = now + csma_get_cw();
            break;

        case CAUSE_PREAMBLE_DETECTED:
            tx_enable = false;
            tx_disabled = now + tx_header_timeout;
            break;

        case CAUSE_HEADER_VALID:
            tx_enable = false;
            tx_disabled = now + tx_data_timeout;
            break;
    }
}
