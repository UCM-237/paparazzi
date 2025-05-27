/*
 * Copyright (C) 2025 UCM
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/*
@file "modules/com/serial_nps.c"
@authors Alejandro Rochas Fernandez
*/

// This file simulates the serial communication for the malacate missions.

#include "modules/com/serial_com.h"
#include "mcu_periph/sys_time.h"
#include "modules/nav/waypoints.h"

static uint32_t mock_start_time = 0;
static bool waiting_for_response = false;
static const uint32_t MOCK_RESPONSE_DELAY = 5000;

bool serial_msg_setting;
bool serial_msg_test;
bool serial_response;

struct serial_parse_t serial_msg;

void serial_ping(void)
{
  if (serial_msg_test && !waiting_for_response) {
    printf("[SIM] Enviando medida (de momento sin datos)\n");
    mock_start_time = get_sys_time_msec();
    waiting_for_response = true;
  }
  else{
    // printf("[SIM] Navigation. msg_test = %d, waiting = %d\n", serial_msg_test, waiting_for_response);
  }

  if (waiting_for_response) {
    if ((get_sys_time_msec() - mock_start_time) >= MOCK_RESPONSE_DELAY) {
      printf("[SIM] Respuesta recibida\n");
      serial_response = true;
      serial_msg_test = false;
      waiting_for_response = false;

      // Simula valores de respuesta
      // serial_msg.depth = probe_depth;
      // serial_msg.time = probe_time;
      serial_msg.error = 0;
    }
  }
}

void serial_init(void)
{
  serial_response = false;
  serial_msg_test = false;
  waiting_for_response = false;
}

// Same as AP but without sending the message
void send_measure_msg(uint8_t wp)
{
  printf("[SIM] Llegada al static ctrl \n");
  int probe_depth = WaypointX(wp);
  int probe_time = WaypointY(wp);

  serial_msg_test = true;
}


