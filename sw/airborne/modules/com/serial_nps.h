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
@file "modules/com/serial_nps.h"
@authors Alejandro Rochas Fernandez
*/


#ifndef SERIAL_NPS_H
#define SERIAL_NPS_H

// Variables globales
extern bool serial_msg_setting;
extern bool serial_msg_test;
extern bool serial_response;

// Parámetros de medida (rellenados desde el plan de vuelo)
extern int16_t probe_depth;
extern uint16_t probe_time;

// Estructura mínima de respuesta simulada
struct serial_parse_t {
  uint8_t error;
  uint16_t time;
  uint16_t depth;
};

extern struct serial_parse_t serial_msg;

// Funciones simuladas
extern void serial_init(void);
extern void serial_ping(void);
extern void send_measure_msg(uint8_t wp);

#endif // SERIAL_NPS_H
