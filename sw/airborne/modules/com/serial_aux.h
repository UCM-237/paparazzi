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
@file "modules/com/serial_aux.h"
@authors Jesús Bautista Villar
         Juan Francisco Jiménez Castellanos
				 Lía García Pérez
 	       Hector Garcia de Marina
				 Alejandro Rochas Fernandez
*/


#ifndef SERIAL_AUX_H
#define SERIAL_AUX_H

#include <stdint.h>

void serial_read_message(void);
void itoh(int value, unsigned char* str, int nbytes);
unsigned int serial_byteToint(uint8_t * bytes,int length);
void ito2h(int value, unsigned char* str);
void ftoh(float value, unsigned char* str, int nbytes) ;

uint32_t msgLength(void);
void serial_calculateChecksumMsg(uint8_t *msg, int msgLength);
void send_full_message(uint8_t msgLength);
void serial_send_msg(uint8_t len, uint8_t *bytes);
uint32_t serial_calculateChecksum(void);

#endif // SERIAL_AUX_H
