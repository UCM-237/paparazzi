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
@file "modules/com/serial_aux.c"
@authors Jesús Bautista Villar
         Juan Francisco Jiménez Castellanos
				 Lía García Pérez
 	       Hector Garcia de Marina
				 Alejandro Rochas Fernandez
*/


#include "serial_aux.h"
#include "serial_com.h"

#include "generated/modules.h"


// ------------------------------------------------------
// ------------------ FUNCIONES CONVERSION --------------
// ------------------------------------------------------

unsigned int serial_byteToint(uint8_t * bytes,int length){
  unsigned int num=0 ;
  for (int i=length-1;i>=0;i--){
      num=num|bytes[i]<<(8*(i));
 }
  return num;
}


void ito2h(int value, unsigned char* str) {
	
  str[1]=(value & 0xff00)>>8;
  str[0]= value & 0x0ff;	
}


void itoh(int value, unsigned char* str, int nbytes){
  double nmax;
  int nb;
  nmax=pow(2,(nbytes-1)*8)-1;
  nb=nbytes;
  if (abs(value) > nmax ) return;
  if (value <0) {
     str[0]=1;
     value=-value;
     }
  else str[0]=0;
  for (int i=1; i<nb; i++){
         str[i]=(value & (0xff << (nb-1-i)*8) ) >> (8*(nb-1-i));
      }

  return;

}


void ftoh(float value, unsigned char* str, int nbytes){
		
  memcpy(str, &value, sizeof(float));
  for (int i = sizeof(float); i < nbytes; i++) {
      str[i] = 0;
  }
}



// ------------------------------------------------------
// ------------------ FUNCIONES AUXILARES ---------------
// ------------------------------------------------------

uint32_t msgLength(void){
  return headerLength + serial_msg.payload_len + checksumLength;
};

uint32_t serial_calculateChecksum(void) {
  uint32_t i = 0;
  uint32_t non_ck_len = msgLength() - checksumLength;
  serial_msg.ck = 0;

  for(i = 0; i < non_ck_len; i++) {
    serial_msg.ck += serial_msg.msgData[i];
  }

  return serial_msg.ck;
}

void serial_calculateChecksumMsg(uint8_t *msg, int msgLength) {
  int i = 0;
  uint16_t suma=0;
  int non_ck_len = msgLength - checksumLength;
  
  for(i = 0; i < non_ck_len; i++) {
    suma += msg[i];
  }
  serial_snd.ck=suma;
  uint8_t chksBytes[2];
  ito2h(suma,chksBytes);
  msg[msgLength-2]=chksBytes[1];
  msg[msgLength-1]=chksBytes[0];
}

void serial_send_msg(uint8_t len, uint8_t *bytes) {
  struct link_device *dev = &((SERIAL_DEV).device);
	
  int i = 0;

  for (i = 0; i < len; i++) {
    dev->put_byte(dev->periph, 0, bytes[i]);

  }
	dev->put_byte(dev->periph, 0, '\n'); 
}

void send_full_message(uint8_t msgLength) {
  serial_calculateChecksumMsg(serial_snd.msgData, (int)msgLength);
  serial_send_msg(msgLength, serial_snd.msgData);
}
