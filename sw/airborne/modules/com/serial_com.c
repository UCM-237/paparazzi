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
@file "modules/com/serial_com.c"
@authors Jesús Bautista Villar
         Juan Francisco Jiménez Castellanos
				 Lía García Pérez
 	       Hector Garcia de Marina
				 Alejandro Rochas Fernandez
*/


#include "modules/com/serial_com.h"
#include "modules/lidar/tfmini.h"
#include "mcu_periph/sys_time.h"
#include "generated/airframe.h"
#include "generated/modules.h"
#include "autopilot.h"
#include "navigation.h"
#include "state.h"
//#include "modules/sonar/sonar_bluerobotics.h"		// No funciona en los rover
#include "modules/radio_control/radio_control.h"
#include "modules/nav/waypoints.h"

#include "firmwares/rover/navigation.h"
#include "pprzlink/messages.h"

#include "std.h"
#include <stdio.h>

#include "math/pprz_geodetic_int.h"
#include "modules/datalink/downlink.h"

#include "modules/ins/ins_int.h"
#include "modules/gps/gps.h"


// extern struct GpsState gps;
// extern struct InsInt ins_int;
struct serial_parse_t serial_msg;
struct serial_send_t serial_snd;

bool serial_msg_setting;

// Sonar msg header bytes (and checksum)
static uint8_t headerLength = 2;
static uint8_t checksumLength = 2;

static uint8_t PPZ_START_BYTE = 0x50; // "P"
static uint8_t COM_START_BYTE = 0x52; // "R"
static uint8_t PPZ_SONAR_BYTE = 0x53; // "S"
static uint8_t PPZ_TELEMETRY_BYTE = 0x54; // "T"
static uint8_t PPZ_HOME_BYTE = 0x48; // "H"
static uint8_t PPZ_IMU_BYTE = 0x49; // "I"
static uint8_t PPZ_GPS_BYTE = 0x47; // "G"
static uint8_t PPZ_LIDAR_BYTE = 0x4C; // "L"
static uint8_t PPZ_MEASURE_BYTE = 0x4D; // "M"
static uint8_t PPZ_SONDA_UP_BYTE = 0x55; // "U"
static uint8_t PPZ_SONDA_DOWN_BYTE = 0x44; // "D"
static uint8_t PPZ_SONDA_CENTER_BYTE = 0x43;	// "C"

static uint32_t last_s = 0;  // timestamp in usec when last message was send
uint16_t counter = 0;				 // for counting the number of messages sent
// uint8_t counter = 0;				 // for counting the number of messages sent
uint32_t msg_buffer = 0;
#define SEND_INTERVAL 500 // time between sending messages (ms)

// Sonar parse states
#define SR_INIT 0
#define SR_SYNC 1
#define SR_PAYLOAD1 2
#define SR_PAYLOAD2 3
#define SR_PAYLOAD3 4
#define SR_PAYLOAD4 5
#define SR_CHECKSUM1 6
#define SR_CHECKSUM2 7

// Sonar errors
/* last error type */
#define SERIAL_BR_ERR_NONE         0
#define SERIAL_BR_ERR_CHECKSUM    1
#define SERIAL_ERR_UNEXPECTED   2

// Messages sent (max id 31) ------------------------
#define END_MESSAGE	31
#define TELEMETRY_SN 0
#define SONDA_RQ 1
#define MEASURE_SN 2
#define SONDA_DOWN 3
#define SONDA_UP 4
#define SONDA_CENTER 9
#define HOME_RESPONSE 5
#define IMU_MESSAGE 6
#define GPS_MESSAGE 7
#define LIDAR_MESSAGE 8

// Delay of each message (0 for not periodic message, >= 1 for periodic)
#define TIME_TELEMETRY 12
#define TIME_HOME 0
#define TIME_IMU 10
#define TIME_GPS 20
#define TIME_LIDAR 0


//Messages received
#define SR_OK 79
#define SR_MEASURE 77
#define SR_FIN 70
#define SR_POS 80
#define SR_ERR_L 76
#define SR_ERR_D 78
#define SR_WAYPOINT 87			// 'W'
#define SR_HOME 72 					// 'H'

//Measuring mode
#define BR_SONAR_MS_1 1
#define BR_SONAR_MS_0 0

int modo_medida=BR_SONAR_MS_0;
int cont = 0;

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

/* Telemetry functions (TESTING TELEMETRY) */

static void send_telemetry(struct transport_tx *trans, struct link_device *dev){
  pprz_msg_send_SERIAL_COM(trans, dev, AC_ID,	&serial_snd.time,
  						&serial_snd.lat,
  						&serial_snd.lon,
  						&serial_snd.alt,
  						&serial_snd.distance,
  						&serial_snd.confidence,
  						&serial_snd.error_last,
  						&serial_snd.msg_length,
  						&serial_snd.ck,
  						//&serial_msg.msg_id,
  						//&serial_msg.status,
  						&serial_snd.msgData[0],
  						&serial_snd.msgData[1],
  						&serial_msg.payload_len,
  						&serial_msg.time,
  						&serial_msg.depth,
  						&serial_msg.error_last,
  						&serial_msg.ck
  						);
}
#endif


/* Initialize decoder */
void serial_init(void)
{ 
  serial_msg.status = SR_INIT;
  serial_msg.msg_available = true;
  serial_msg.error_last=0;
  serial_msg_setting = true;
  
  last_s=get_sys_time_msec();
  #if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_SERIAL_COM, send_telemetry);
  #endif

}


// ------------------------------------------------------
// ------------------ FUNCIONES AUXILARES ---------------
// ------------------------------------------------------


/* Message parsing functions */
static uint32_t msgLength(void){
  return headerLength + serial_msg.payload_len + checksumLength;
};

/* Send message to serial port (byte by byte) */
static void serial_send_msg(uint8_t len, uint8_t *bytes)
{
  struct link_device *dev = &((SERIAL_DEV).device);
	
  int i = 0;

  for (i = 0; i < len; i++) {
    dev->put_byte(dev->periph, 0, bytes[i]);

  }
	dev->put_byte(dev->periph, 0, '\n'); 
}


/* Calculate the checksum */
static uint32_t serial_calculateChecksum(void){
  uint32_t i = 0;
  uint32_t non_ck_len = msgLength() - checksumLength;
  serial_msg.ck = 0;

  for(i = 0; i < non_ck_len; i++) {
    serial_msg.ck += serial_msg.msgData[i];
  }

  return serial_msg.ck;
}

void serial_calculateChecksumMsg(uint8_t *msg, int msgLength){
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


};

void send_full_message(uint8_t msgLength) {
    serial_calculateChecksumMsg(serial_snd.msgData, (int)msgLength);
    serial_send_msg(msgLength, serial_snd.msgData);
}


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
// -------- FUNCIONES PARA LA RECEPCION DE MENSAJES -----
// ------------------------------------------------------


static void parse_MOVE_WP(void)
{
	// uint8_t ac_id = 2;		// Esto no nos hace falta
  // if (ac_id != AC_ID) { return; }
 
	uint8_t msg[serial_msg.payload_len + 2];	// Esto es por simplicidad
  memcpy(msg, serial_msg.msgData, serial_msg.payload_len + 2);
	
	uint8_t wp_id = msg[2];

  struct LlaCoor_i lla;
	lla.lat=(int32_t)(msg[3] | msg[4] << 8 | msg[5] << 16 | msg[6] << 24);
  // lla.lat = 40.4498915 * 1E+7;
	lla.lon=(int32_t)(msg[7] | msg[8] << 8 | msg[9] << 16 | msg[10] << 24);
  // lla.lon = -3.7250035 * 1E+7;
	lla.alt=(int32_t)(msg[11] | msg[12] << 8 | msg[13] << 16 | msg[14] << 24);
  lla.alt = lla.alt - state.ned_origin_i.hmsl + state.ned_origin_i.lla.alt;
  waypoint_move_lla(wp_id, &lla);

}


// Aqui se procesaban los mensajes (no se esta usando ahora mismo)
static void message_parse(void){
	uint8_t msgBytes[2]={serial_msg.msgData[3],serial_msg.msgData[2]};
	serial_msg.time = serial_byteToint(msgBytes,2);
	memset(msgBytes,0,2);
	msgBytes[0]=serial_msg.msgData[5];
	msgBytes[1]=serial_msg.msgData[4];
	serial_msg.depth=serial_byteToint(msgBytes,2);
}
  

// Aqui se procesaban los mensajes (no se esta usando ahora mismo)
static void message_OK_parse(void){
	uint8_t msgBytes[2]={serial_msg.msgData[3],serial_msg.msgData[2]};
	serial_msg.time = serial_byteToint(msgBytes,2);

}
  
  
/* Serial message parser */
// Aqui lee el mensaje
void serial_read_message(void){

// Checksum
	uint8_t chksBytes[2];
	if (serial_msg.msg_id == SR_OK){
		chksBytes[0]=serial_msg.msgData[5];
		chksBytes[1]=serial_msg.msgData[4];
	}
	else if (serial_msg.msg_id == SR_WAYPOINT){
		chksBytes[0]=serial_msg.msgData[2+serial_msg.payload_len+1];
		chksBytes[1]=serial_msg.msgData[2+serial_msg.payload_len];
	}
	else {
		chksBytes[0]=serial_msg.msgData[7];
		chksBytes[1]=serial_msg.msgData[6];

	}
	uint16_t chks;
	chks=serial_byteToint(chksBytes,2);
	serial_calculateChecksum();
	switch (serial_msg.msg_id){
		case SR_WAYPOINT:	// Este es para procesar los waypoints 
			parse_MOVE_WP();
			serial_msg.error_last = SERIAL_BR_ERR_NONE;
			// Ahora que puedo mandar multiples mensajes ¿poner un ACK?
 			break;

		case SR_HOME:	// Este es para procesar el home request
			// parse_HOME();		// REVISAR ESTO 
			SET_BIT(msg_buffer, HOME_RESPONSE);
			serial_msg.error_last = SERIAL_BR_ERR_NONE;
 			break;
 		
		case SR_OK:	 // Este es el mensaje de respuesta de medida (sin usar)
 			message_OK_parse();
 			serial_msg.error_last = SERIAL_BR_ERR_NONE;
 			// message_type = 10;
 			break;
 		
		case SR_FIN:
 			message_parse();
 			serial_msg.error_last = SERIAL_BR_ERR_NONE;
			// message_type = TELEMETRY_SN;
			modo_medida = BR_SONAR_MS_0;
 			break;
 		
		case SR_MEASURE:
 			message_parse();
 			serial_msg.error_last = SERIAL_BR_ERR_NONE;
			// message_type = 10;
 			break;
 		
		case SR_POS:
 			message_parse();
 			serial_msg.error_last = SERIAL_BR_ERR_NONE;
 			// message_type = 10;
 			break;
 		
		case SR_ERR_L:
 			serial_msg.error_last = SR_ERR_L;
 			break;
 		
		case SR_ERR_D:
 			serial_msg.error_last = SR_ERR_D;
 			break;

 		default:
 			serial_msg.error_last = SERIAL_ERR_UNEXPECTED;
 			break;
 		};

 	if(chks!=(unsigned int) serial_msg.ck){
		serial_msg.error_last=SERIAL_BR_ERR_CHECKSUM;
		serial_msg.error_cnt++;
		}
}


static void serial_parse(uint8_t byte){
	bool error=false;
	bool restart = false;

	switch (serial_msg.status){
		
		case SR_INIT: // First byte of a new message
			if (byte == COM_START_BYTE) { //Has to be an 'R'
				memset(serial_msg.msgData,0,17);
				serial_msg.status++;
				serial_msg.msgData[0]=byte;
				serial_msg.msg_available=false; 
				}
			break;

		case SR_SYNC: //Second byte
			serial_msg.status++;
			serial_msg.msg_id = byte;
			if(byte == SR_OK) serial_msg.payload_len=2;
			else if (byte == SR_WAYPOINT){
				serial_msg.payload_len= 13;
				serial_msg.count = 0;
				serial_msg.status = SR_WAYPOINT;
			}
			else if (byte == SR_HOME){
				serial_msg.payload_len= 1;
				serial_msg.count = 0;
				serial_msg.status = SR_WAYPOINT;
			} 
			else serial_msg.payload_len=4;			
			serial_msg.msgData[1]=byte;
			break;

		case SR_WAYPOINT:	// 3th and the rest for the messages (cambiar algun momento el nombre)
			serial_msg.msgData[serial_msg.count+2]=byte;
			serial_msg.count++;

			if (serial_msg.count >= serial_msg.payload_len+2){
				serial_msg.msg_available = true;
				serial_msg.status=SR_INIT;
			}
			break;

		// TODO ESTO YA NO SE ESTA USANDO 
		// case SR_PAYLOAD1: //3th byte 
		// 	serial_msg.status++;
		// 	serial_msg.msgData[2]=byte;		
		// 	break;

		// case SR_PAYLOAD2: //4th byte payload length byte 2
		// 	if(serial_msg.msg_id==SR_OK){
		// 		serial_msg.status +=2;
		// 		}
		// 	serial_msg.status++;
		// 	serial_msg.msgData[3]=byte;
		// 	break;
		// case SR_PAYLOAD3:
		// 	serial_msg.error_last = 0;
		// 	serial_msg.status++;
		// 	serial_msg.msgData[4]=byte;
		// 	break;
		// case SR_PAYLOAD4:
		// 	serial_msg.error_last = 0;
		// 	serial_msg.status++;
		// 	serial_msg.msgData[5]=byte;
		// 	break;
		// case SR_CHECKSUM1:
		// 	serial_msg.error_last = 0;
		// 	serial_msg.status++;
		// 	if (serial_msg.msg_id==SR_OK)
		// 		serial_msg.msgData[4]=byte;
		// 	else
		// 		serial_msg.msgData[6]=byte;
		// 	break;
		// case SR_CHECKSUM2:
		// 	serial_msg.error_last = 0;
		// 	serial_msg.status++;
		// 	if (serial_msg.msg_id==SR_OK)
		// 		serial_msg.msgData[5]=byte;
		// 	else 
		// 		serial_msg.msgData[7]=byte;
		// 	serial_msg.msg_available = true;
		// 	serial_msg.status=0;
		// 	restart=true;
		// 	break;

		default:
			serial_msg.error_last = SERIAL_ERR_UNEXPECTED;
			serial_msg.status= 0;
			error=true;
	};
	if(error){
		serial_msg.error_cnt++;
		serial_msg.status=0;
		serial_msg.msg_available = true;
		return;
		}
	if (restart){
		serial_msg.status =  SR_INIT;
		return;
		}
	return;

}; 


// Para recibir los mensajes
void serial_event(void)
{
 
 while(uart_char_available(&(SERIAL_DEV))){
 	uint8_t ch= uart_getch(&(SERIAL_DEV));
	
 	serial_parse(ch);		// Este lee del puerto serie
	
 	if (serial_msg.msg_available) {
      		serial_read_message();	// Este procesa el mensaje leido
         
       	}	
    }
}


// ------------------------------------------------------
// -----------FUNCIONES PARA EL ENVIO DE MENSAJES -------
// ------------------------------------------------------


uint8_t set_header(uint8_t type) {

		uint8_t msg_time[2]={0,0};
    memset(serial_snd.msgData, 0, serial_snd.msg_length);
    serial_snd.msgData[0] = PPZ_START_BYTE;
    serial_snd.msgData[1] = type;
    serial_snd.time=sys_time.nb_sec;
				
		ito2h(serial_snd.time, msg_time);
		serial_snd.msgData[2]=msg_time[0];
		serial_snd.msgData[3]=msg_time[1];

		return 4;
}


void set_gps_message(uint8_t start_byte){

	uint8_t j = start_byte;
	// struct LlaCoor_i *gps_coord;
	uint8_t msg_gps[5]={0,0,0,0,0};

	memset(&serial_snd.msgData[j], 0, 17);

	// Datos del GPS
	serial_snd.lat = gps.lla_pos.lat;
	serial_snd.lon = gps.lla_pos.lon;
	serial_snd.alt = 650000;

	itoh(serial_snd.lon, msg_gps, 5);
	for(int i=0;i<5;i++) serial_snd.msgData[i+j]=msg_gps[i];
	memset(msg_gps,0,5);
	itoh(serial_snd.lat, msg_gps, 5);
	for(int i=0;i<5;i++) serial_snd.msgData[i+j+5]=msg_gps[i];
	memset(msg_gps,0,5);
	itoh(serial_snd.alt,msg_gps,5);
	for(int i=0;i<4;i++) serial_snd.msgData[i+j+10]=msg_gps[i+1];

	// return (j+13);

}


void set_imu_message(uint8_t start_byte){

	uint8_t j = start_byte;
	uint8_t msg_imu[5]={0,0,0,0,0};
	// struct Int32Vect3 *accel_state;
	struct NedCoor_i *accel_state;
	

	memset(&serial_snd.msgData[j], 0, 18);

	// (Reutilizo los snd.lon ...)
	accel_state = stateGetAccelNed_i();
	serial_snd.lon=accel_state->x;
	serial_snd.lat=accel_state->y;
	serial_snd.alt=accel_state->z;

	itoh(accel_state->x,msg_imu,5);
	for(int i=0;i<5;i++) serial_snd.msgData[i+j]=msg_imu[i];
	memset(msg_imu,0,5);
	itoh(accel_state->y,msg_imu,5);
	for(int i=0;i<5;i++) serial_snd.msgData[i+j+5]=msg_imu[i];
	memset(msg_imu,0,5);
	itoh(serial_snd.alt,msg_imu,5);
	for(int i=0;i<5;i++) serial_snd.msgData[i+j+10]=msg_imu[i];

	// return (j+14);

}


void set_telemetry_message(uint8_t start_byte){

	uint8_t j = start_byte;
	struct LlaCoor_i *gps_coord;
	uint8_t msg_gps[5]={0,0,0,0,0};
	uint8_t msg_dist[5]={0,0,0,0,0};

	memset(&serial_snd.msgData[j], 0, 18);
	gps_coord = stateGetPositionLla_i();
	serial_snd.lon=gps_coord->lon;
	serial_snd.lat=gps_coord->lat;
	int32_t alt = (int)(ahrs_dcm.ltp_to_body_euler.psi*1e7);
	serial_snd.alt=alt;		// Changed to avoid issues with the rover
	itoh(gps_coord->lon,msg_gps,5);
	for(int i=0;i<5;i++) serial_snd.msgData[i+j]=msg_gps[i];
	memset(msg_gps,0,5);
	itoh(gps_coord->lat,msg_gps,5);
	for(int i=0;i<5;i++) serial_snd.msgData[i+j+5]=msg_gps[i];
	memset(msg_gps,0,5);
	/*NOTE: serial_snd.alt is an unsigned int. It is codified as 
	an signed (using one extra byte) int but sign byte is discarded
	*/
	itoh(serial_snd.alt,msg_gps,5);
	for(int i=0;i<5;i++) serial_snd.msgData[i+j+10]=msg_gps[i];
	
	// Get Sonar
	serial_snd.distance=0;
	serial_snd.confidence=0;
	/*NOTE: serial_snd.distance is an unsigned int. It is codified as 
	an signed (using one extra byte) int but sign byte is discarded
	*/
	
	itoh(serial_snd.distance,msg_dist,5);
	for(int i=0;i<4;i++) serial_snd.msgData[i+j+15]=msg_dist[i+1];
	serial_snd.msgData[j+19]=serial_snd.confidence;

	// return (j+19);

}


// ------------------------------------------------------

void serial_ping()
{   
	uint32_t now_s = get_sys_time_msec();

	uint8_t msg_byte = 0;
	struct sonar_parse_t *sonar_data;	// No funciona en el rover, solo en el barco
	uint8_t msg_gps[5]={0,0,0,0,0};
	uint8_t msg_dist[5]={0,0,0,0,0};

	// Aqui a lo mejor habria que comprobar que solo se active uno (depende de lo se necesite)
	if (radio_control_get(RADIO_GAIN2)>0){
		RESET_BUFFER(msg_buffer);
		SET_BIT(msg_buffer, SONDA_UP);
	}
	else if (radio_control_get(RADIO_GAIN2)<0){
		RESET_BUFFER(msg_buffer);
		SET_BIT(msg_buffer, SONDA_DOWN);
	}
	else{
		RESET_BUFFER(msg_buffer);
		SET_BIT(msg_buffer, SONDA_CENTER);
	}


	if (now_s > (last_s + SEND_INTERVAL)) {
		
		last_s = now_s;
		CLEAR_BIT(msg_buffer, END_MESSAGE);	// Por si acaso
		while (!CHECK_BIT(msg_buffer, END_MESSAGE)){
	
			if(CHECK_BIT(msg_buffer, TELEMETRY_SN)){
				serial_snd.msg_length=26;

				msg_byte = set_header(PPZ_TELEMETRY_BYTE);
				set_telemetry_message(msg_byte);

				send_full_message(serial_snd.msg_length);
				CLEAR_BIT(msg_buffer, TELEMETRY_SN);
			}

			else if(CHECK_BIT(msg_buffer, GPS_MESSAGE)){
				serial_snd.msg_length = 20;

				msg_byte = set_header(PPZ_GPS_BYTE);
				set_gps_message(msg_byte);

				send_full_message(serial_snd.msg_length);
				CLEAR_BIT(msg_buffer, GPS_MESSAGE); 
			}

			else if(CHECK_BIT(msg_buffer, IMU_MESSAGE)){
				serial_snd.msg_length=21;

				msg_byte = set_header(PPZ_IMU_BYTE);				
				set_imu_message(msg_byte);

				send_full_message(serial_snd.msg_length);

				CLEAR_BIT(msg_buffer, IMU_MESSAGE); 
			}

			else if(CHECK_BIT(msg_buffer, HOME_RESPONSE)){
				serial_snd.msg_length=20;

				msg_byte = set_header(PPZ_HOME_BYTE);

				// Coordenates from HOME
				serial_snd.lat=(int)(waypoint_get_lat_deg(1)*1E+07);
				serial_snd.lon=(int)(waypoint_get_lon_deg(1)*1E+07);
				serial_snd.alt=(int)(650*1E+03);

				itoh(serial_snd.lon, msg_gps, 5);
				for(int i=0;i<5;i++) serial_snd.msgData[i+4]=msg_gps[i];
				memset(msg_gps,0,5);
				itoh(serial_snd.lat, msg_gps, 5);
				for(int i=0;i<5;i++) serial_snd.msgData[i+9]=msg_gps[i];
				memset(msg_gps,0,5);
				itoh(serial_snd.alt,msg_gps,5);
				for(int i=0;i<4;i++) serial_snd.msgData[i+14]=msg_gps[i+1];
				
				send_full_message(serial_snd.msg_length);
				CLEAR_BIT(msg_buffer, HOME_RESPONSE); 

			}

      // TODO: Comprobar si existe el Lidar
			// else if(CHECK_BIT(msg_buffer, LIDAR_MESSAGE)){
			// 	serial_snd.msg_length=14;

			// 	msg_byte = set_header(PPZ_LIDAR_BYTE);

			// 	uint8_t lidar_hex[4]={0,0,0,0};
			// 	ftoh(tfmini.distance, lidar_hex, 4);
			// 	for(int i=0;i<4;i++) serial_snd.msgData[i+msg_byte]=lidar_hex[i];
			// 	msg_byte += 4;

			// 	memset(lidar_hex,0,4);
			// 	ftoh(tf_servo.ang, lidar_hex, 4);
			// 	for(int i=0;i<4;i++) serial_snd.msgData[i+msg_byte]=lidar_hex[i];

			// 	send_full_message(serial_snd.msg_length);
			// 	CLEAR_BIT(msg_buffer, LIDAR_MESSAGE);
			// }

			// ---- BEGIN UNUSED ---------
			else if(CHECK_BIT(msg_buffer, SONDA_RQ)){
				serial_snd.msg_length=6;
				msg_byte = set_header(PPZ_SONAR_BYTE);

				send_full_message(serial_snd.msg_length);
				CLEAR_BIT(msg_buffer, SONDA_RQ); 
			}

			else if(CHECK_BIT(msg_buffer, SONDA_UP)){
				// (tendria que implementar aqui algo de prueba para ver si funciona)
				serial_snd.msg_length=6;
				msg_byte = set_header(PPZ_SONDA_UP_BYTE);
				
				send_full_message(serial_snd.msg_length);
				CLEAR_BIT(msg_buffer, SONDA_UP); 
			}

			else if(CHECK_BIT(msg_buffer, SONDA_DOWN)){
				// (mismo caso que UP)
				serial_snd.msg_length=6;
				msg_byte = set_header(PPZ_SONDA_DOWN_BYTE);

				send_full_message(serial_snd.msg_length);
				CLEAR_BIT(msg_buffer, SONDA_DOWN); 
			}

			else if(CHECK_BIT(msg_buffer, SONDA_CENTER)){
				// (tendria que implementar aqui algo de prueba para ver si funciona)
				serial_snd.msg_length=6;
				msg_byte = set_header(PPZ_SONDA_CENTER_BYTE);
				
				send_full_message(serial_snd.msg_length);
				CLEAR_BIT(msg_buffer, SONDA_CENTER); 
			}
			
			else if(CHECK_BIT(msg_buffer, MEASURE_SN)){
			//No funciona en el rover
				serial_snd.msg_length=26;
				msg_byte = set_header(PPZ_MEASURE_BYTE);
				set_telemetry_message(msg_byte);
				// Reescribe esta parte
				// Get Sonar
				sonar_data= sonar_get();
				serial_snd.distance=sonar_data->distance;
				serial_snd.confidence=sonar_data->confidence;
				/*NOTE: serial_snd.distance is an unsigned int. It is codified as 
				an signed (using one extra byte) int but sign byte is discarded
				*/
				itoh(sonar_data->distance,msg_dist,5);
				for(int i=0;i<4;i++) serial_snd.msgData[i+18]=msg_dist[i+1];
				serial_snd.msgData[22]=sonar_data->confidence;
				serial_snd.msgData[23]=modo_medida;
				send_full_message(serial_snd.msg_length);
				CLEAR_BIT(msg_buffer, MEASURE_SN); 
			}

			// ---- END UNUSED ---------

			else{	 
				// Si llega a aqui es que no quedan mensajes que mandar
				RESET_BUFFER(msg_buffer);	// Por si acaso
				SET_BIT(msg_buffer, END_MESSAGE);
			}
		
		} // end WHILE

		RESET_BUFFER(msg_buffer);

			// Set the messages to sent in the next iteration
		// SET_BIT_IF(counter, TIME_TELEMETRY, msg_buffer, TELEMETRY_SN);
		// SET_BIT_IF(counter, TIME_IMU, msg_buffer, IMU_MESSAGE);
		// SET_BIT_IF(counter, TIME_GPS, msg_buffer, GPS_MESSAGE);
		// SET_BIT_IF(counter, TIME_LIDAR, msg_buffer, LIDAR_MESSAGE);	// Disable on the boat

		counter = (counter >= 255) ? 0 : counter + 1;

	}	// end IF
	
}	// VOID

