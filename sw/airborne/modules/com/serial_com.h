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


  @file "modules/com/serial_com.h"
  @authors Jesús Bautista Villar
          Juan Francisco Jiménez Castellanos
          Lía García Pérez
          Hector Garcia de Marina
          Alejandro Rochas Fernandez
*/
 
#ifndef SERIAL_COM_H
#define SERIAL_COM_H

#include "std.h"

#include "pprzlink/pprzlink_device.h"
#include "mcu_periph/uart.h"

/* Parser variables */
// Variable to start/stop requesting stream
extern bool serial_msg_setting;
extern bool serial_msg_test;
extern bool serial_response;

extern const uint8_t headerLength;
extern const uint8_t checksumLength;


/* Parser msg struct */
#define SERIAL_MAX_PAYLOAD 26
#define SERIAL_MAX_MSG 17
#define BUTTONS 3 // Numero de botones usados del mando


/* Macros for bit manipulation */
#define SET_BIT(buffer, N)     ((buffer) |= (1 << (N)))
#define CLEAR_BIT(buffer, N)   ((buffer) &= ~(1 << (N)))
#define CHECK_BIT(buffer, N)   (((buffer) & (1 << (N))) != 0)
#define RESET_BUFFER(buffer)   ((buffer) = 0)
#define SET_BIT_IF(counter, interval, buffer, message) \
  if ((interval) > 0 && (counter) % (interval) == 0) { \
      SET_BIT(buffer, message); \
  }


// SEND
struct serial_send_t {

  uint8_t msg_length;
  uint8_t msg_id;

  uint8_t msgData[SERIAL_MAX_PAYLOAD];
  uint8_t error_cnt;
  uint8_t error;
  
  int32_t lon;
  int32_t lat;
  int32_t alt;
  
  uint32_t distance;
  uint8_t confidence;
  
  uint16_t ck;
  uint16_t time;
  uint16_t depth;

};

// RECEIVE
struct serial_parse_t {

  uint8_t msg_id;

  uint8_t msgData[SERIAL_MAX_MSG] __attribute__((aligned));
  uint8_t status;
  uint8_t count;
  
  uint8_t error_cnt;
  uint8_t error;
  
  uint8_t payload_len;
  
  uint16_t ck;
  bool msg_available;
  
  uint16_t time;
  uint16_t depth;

  int16_t button_state[BUTTONS];

};

extern struct serial_parse_t serial_msg;
extern struct serial_send_t serial_snd;


// Message functions
uint8_t set_header(uint8_t type);
void set_gps_message(uint8_t start_byte);
void set_imu_message(uint8_t start_byte);
void set_telemetry_message(uint8_t start_byte);
void set_probe_message(uint8_t start_byte, int16_t depth, uint16_t time);
void send_measure_msg(uint8_t wp);

  
/* External functions (called by the autopilot)*/
extern void serial_init(void);
extern void serial_ping(void);
extern void serial_event(void);
extern bool check_malacate(void);

#endif //SERIAL_COM_H