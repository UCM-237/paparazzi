/*
 * Copyright (C) 2025 Alejandro Rochas <alrochas@ucm.es>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** @file modules/lidar/sweep_lidar.c
 *  @brief driver for the Scanse Sweep lidar
 *
 */

// SIN PROBAR, LO MAS SEGURO ES QUE NO FUNCIONE

#include "mcu_periph/uart.h"
#include "modules/core/abi.h"
#include "state.h"
#include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"

// static uint32_t last_time = 0;

struct SweepLidar sweep_lidar;
static void sweep_parse(uint8_t byte);

void sweep_init(void) {
    sweep_lidar.device = &((LIDAR_UART_PORT).device);
    sweep_lidar.parse_status = 0;
    sweep_lidar.distance = 0;
    sweep_lidar.angle = 0;
    sweep_lidar.signal_strength = 0;
    uart_send_string(sweep_lidar.device, "DS\n"); // Start data acquisition
}

void sweep_event(void) {
    while (sweep_lidar.device->char_available(sweep_lidar.device->periph)) {
        sweep_parse(sweep_lidar.device->get_byte(sweep_lidar.device->periph));
    }
}

static void sweep_parse(uint8_t byte) {
    static uint8_t buffer[7];
    static uint8_t index = 0;
    
    buffer[index++] = byte;
    if (index == 7) {
        index = 0;
        sweep_lidar.angle = (buffer[1] << 8 | buffer[2]) / 16.0f;
        sweep_lidar.distance = buffer[3] << 8 | buffer[4];
        sweep_lidar.signal_strength = buffer[5];
        uint8_t checksum = (buffer[0] + buffer[1] + buffer[2] + buffer[3] + buffer[4] + buffer[5]) % 255;
        if (checksum == buffer[6]) {
            AbiSendMsgAGL(AGL_LIDAR_SWEEP_ID, get_sys_time_usec(), sweep_lidar.distance);
        }
    }
}
