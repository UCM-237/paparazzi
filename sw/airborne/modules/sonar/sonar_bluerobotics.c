/*
 * Copyright (C) 2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file "modules/sensors/sonar/sonar_bluerobotics.c"
 * @authors Jesús Bautista Villar
 *          Juan Francisco Jiménez Castellanos
 *          Lía García Pérez
 *          Hector Garcia de Marina
 * 
 */

#include "modules/sonar/sonar_bluerobotics.h"

#include "generated/airframe.h"
#include "autopilot.h"
#include "navigation.h"
#include "state.h"

#include "std.h"
#include <stdio.h>

struct sonar_parse_t br_sonar;

bool sonar_stream_setting;

// Sonar msg header bytes (and checksum)
static uint8_t headerLength = 8;
static uint8_t checksumLength = 2;

static uint8_t SONAR_START1_BYTE = 0x42; // "B"
static uint8_t SONAR_START2_BYTE = 0x52; // "R"

// Sonar parse states
#define BR_INIT 0
#define BR_SYNC1 1
#define BR_SYNC2 2
#define BR_PAYLOAD_LEN1 3
#define BR_PAYLOAD_LEN2 4
#define BR_MSG_ID1 5
#define BR_MSG_ID2 6
#define BR_SRC_ID 7
#define BR_DEV_ID 8
#define BR_PAYLOAD 9
#define BR_CHECKSUM1 10

// Testing variable
uint8_t checksum_test;

/* TODO: Revisar que estos mensajes son correctos y que el sonar responde a ellos */
// Test request message: (general_request#6 --> common_protocol_version#5)
static uint8_t request_protocol_version[12] = { 
        0x42, //  0: "B"
        0x52, //  1: "R"
        0x02, //  2: 2_L payload length |
        0x00, //  3: 0_H                |
        0x06, //  4: 6_L message ID |
        0x00, //  5: 0_H            |
        0x00, //  6: source ID
        0x00, //  7: device ID
        0x05, //  8: 5_L requested message ID |
        0x00, //  9: 0_H                      |
        0xA1, // 10: 161_L message checksum (sum of all non-checksum bytes) |
        0x00  // 11: 0_H                                                    |
};

// Test request message: (general_request#6 --> distance_simple#1211)
static uint8_t request_simple_distance[12] = { 
        0x42, //  0: "B"
        0x52, //  1: "R"
        0x02, //  2: 2_L payload length |
        0x00, //  3: 0_H                |
        0x06, //  4: 6_L message ID |
        0x00, //  5: 0_H            |
        0x00, //  6: source ID
        0x01, //  7: device ID
        0xBB, //  8: BB_L requested message ID |
        0x04, //  9: 4_H                      |
        0x5C, // 10: 161_L message checksum (sum of all non-checksum bytes) |
        0x01  // 11: 0_H                                                    |
};

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

/* Telemetry functions (TESTING TELEMETRY) */
static void send_telemetry(struct transport_tx *trans, struct link_device *dev){
  pprz_msg_send_INFO_MSG(trans, dev, AC_ID, &checksum_test);
}
#endif


/* Initialize decoder */
void sonar_init(void)
{ 
  br_sonar.status = BR_INIT;
  br_sonar.msg_available = false;

  sonar_stream_setting = true;
  checksum_test = 0;
  
  #if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INFO_MSG, send_telemetry);
  #endif
}


/* Send message to serial port (byte by byte) */
static void sonar_send_msg(uint8_t len, uint8_t *bytes)
{
  struct link_device *dev = &((SONAR_DEV).device);

  int i = 0;
  for (i = 0; i < len; i++) {
    dev->put_byte(dev->periph, 0, bytes[i]);
  }

  br_sonar.msg_available = false;
}


/* Message parsing functions */
static uint32_t msgLength(void){
  return headerLength + br_sonar.payload_len + checksumLength;
};

static uint32_t calculateChecksum(void){
  uint32_t i = 0;
  uint32_t non_ck_len = msgLength() - checksumLength;
  br_sonar.ck = 0;

  for(i = 0; i < non_ck_len; i++) {
    br_sonar.ck += br_sonar.msgData[i];
  }

  return br_sonar.ck;
};

/* TODO: Diseñar el parser basado en GPS ublox y el código de Lia */
static void sonar_parse(uint8_t byte){checksum_test = checksum_test + 1;}; // ¡¡TESTING PARSER!!

/*
// To hexadecimal string for testing
static void dummy_sonar_parse(struct sonar_msg_t *BR_msg, uint8_t byte)
	{
	  BR_msg->len += sprintf(BR_msg->msg + BR_msg->len, "0x%02X ", byte);
	}
*/


// -- EXTERNAL FUNCTIONS -- // (callable from modules and/or autopilot) //
/* Look for data on serial port and send to parser */
/* TODO: (mandar también un mensaje si este está disponible) */
void sonar_event(void)
{
  struct link_device *dev = &((SONAR_DEV).device);
  // Por aquí pasa
  
  while (dev->char_available(dev->periph)) {
    sonar_parse(dev->get_byte(dev->periph));
    checksum_test = checksum_test + 1; // NO PASA
    /*if (sonar.msg_available) {
      sonar_send_msg();
    };*/
    }
}

// Send ping message
void sonar_ping(void)
{   
    sonar_send_msg(12, request_protocol_version);
    // Por aquí pasa
}
