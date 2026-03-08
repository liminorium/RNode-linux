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
#include <fcntl.h>
#include <errno.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <syslog.h>

#include "util.h"
#include "kiss.h"
#include "rnode.h"

static struct sockaddr_in   address;
static int                  server_fd;
static int                  client_fd = 0;
static uint8_t              buf_in[MTU] = {0};

void tcp_init(uint32_t port) {
    int addrlen = sizeof(address);

    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    int on = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port);

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        syslog(LOG_ERR, "TCP bind failed");
        exit(EXIT_FAILURE);
    }

    if (listen(server_fd, 1) < 0) {
        syslog(LOG_ERR, "TCP listen");
        exit(EXIT_FAILURE);
    }

    // Set server socket to non-blocking
    int flags = fcntl(server_fd, F_GETFL, 0);
    fcntl(server_fd, F_SETFL, flags | O_NONBLOCK);

    syslog(LOG_INFO, "TCP listening on port %d", port);
}

void tcp_read() {
    int addrlen = sizeof(address);
    fd_set readfds;
    struct timeval tv;
    int activity;

    while (true) {
        // Clear the file descriptor set
        FD_ZERO(&readfds);

        // Add server socket to set (to accept new connections)
        FD_SET(server_fd, &readfds);
        int max_fd = server_fd;

        // Add client socket to set if connected
        if (client_fd > 0) {
            FD_SET(client_fd, &readfds);
            if (client_fd > max_fd) {
                max_fd = client_fd;
            }
        }

        // Set timeout to 1ms - allows radio IRQ to process while we're not blocked
        tv.tv_sec = 0;
        tv.tv_usec = 1000;

        // Wait for activity on sockets
        activity = select(max_fd + 1, &readfds, NULL, NULL, &tv);

        if (activity < 0 && errno != EINTR) {
            syslog(LOG_ERR, "TCP select error: %s", strerror(errno));
            break;
        }

        // Check if there's a new incoming connection
        if (FD_ISSET(server_fd, &readfds)) {
            int new_fd = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen);
            
            if (new_fd < 0) {
                if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    syslog(LOG_ERR, "TCP accept error: %s", strerror(errno));
                }
            } else {
                // Close previous client if still connected
                if (client_fd > 0) {
                    close(client_fd);
                    syslog(LOG_INFO, "Previous client disconnected");
                }

                client_fd = new_fd;

                // Set client socket to non-blocking
                int flags = fcntl(client_fd, F_GETFL, 0);
                fcntl(client_fd, F_SETFL, flags | O_NONBLOCK);

                syslog(LOG_INFO, "Client connected from %s:%d",
                       inet_ntoa(address.sin_addr), ntohs(address.sin_port));
            }
        }

        // Check if client has data to read
        if (client_fd > 0 && FD_ISSET(client_fd, &readfds)) {
            int res = read(client_fd, buf_in, sizeof(buf_in));

            if (res <= 0) {
                if (res < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                    syslog(LOG_INFO, "Client disconnected: %s", strerror(errno));
                } else if (res == 0) {
                    syslog(LOG_INFO, "Client disconnected");
                }

                close(client_fd);
                client_fd = 0;
            } else {
                // Process incoming KISS data
                kiss_decode(buf_in, res);
            }
        }

        // Return to main loop to allow radio IRQ processing
        // This is the key difference: we DON'T stay in this loop
        // instead we return and let main.c call us again
        return;
    }
}

void tcp_send(char *buf, size_t len) {
    if (client_fd > 0) {
        ssize_t sent = send(client_fd, buf, len, MSG_DONTWAIT);
        if (sent < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                syslog(LOG_ERR, "TCP send error: %s", strerror(errno));
                close(client_fd);
                client_fd = 0;
            }
        }
    }
}