/*
 * Copyright (C) 2025 Alejandro Rochas Fernández <alrochas@ucm.es>
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
 */

/**
 * @file modules/ins/ins_int.c
 *
 * INS + SLAM Correction for the rovers using the TFMini Lidar.
 *
 */

// ------------------------------------

#include "autopilot.h"
#include "modules/ins/ins_int.h"
#include "modules/ins/ins_slam.h"
#include "modules/lidar/slam/lidar_correction.h"

#include "modules/nav/waypoints.h"

static struct FloatVect2 gps_offset = {0.0, 0.0};
static struct FloatVect2 offset = {0.0, 0.0};
static struct FloatVect2 nearest_point = {0.0, 0.0};
static struct FloatVect2 obstacle = {0.0, 0.0};

static struct FloatVect2 debug_point = {0.0, 0.0};  // BORRAR

// Parámetros de corrección
#define MIN_LIDAR_DISTANCE 0.1f
#define MAX_LIDAR_DISTANCE 10.0f
#define MAX_WALL_DISTANCE 5.0f   // No se corrige si el obstáculo está muy lejos
#define ALPHA 0.5f               // Factor de suavizado

struct InsSlam ins_slam = {
    TRUE,
    MIN_LIDAR_DISTANCE,
    MAX_LIDAR_DISTANCE,
    MAX_WALL_DISTANCE,
    ALPHA
};

struct InsInt ins_int;
struct FloatVector {
  float x;  ///< North
  float y;  ///< East
  float z;  ///< Down
};


#include "modules/core/abi.h"

static abi_event lidar_ev;
static void lidar_cb(uint8_t sender_id, uint32_t stamp, float distance, float angle);

uint8_t counter_test = 0;   // Esto es para pruebas


#include "modules/imu/imu.h"
#include "modules/gps/gps.h"

#include "generated/airframe.h"

#if USE_VFF_EXTENDED
#include "modules/ins/vf_extended_float.h"
#else
#include "modules/ins/vf_float.h"
#endif

#if defined SITL && USE_NPS
//#include "nps_fdm.h"
#include "nps_autopilot.h"
#include <stdio.h>
#endif

#include "math/pprz_geodetic_int.h"
#include "math/pprz_isa.h"
#include "math/pprz_stat.h"
#include <math.h>

#ifndef VFF_R_AGL
#define VFF_R_AGL 0.2
#endif

#if USE_GPS
  #ifndef INS_VFF_R_GPS
  #define INS_VFF_R_GPS 2.0
  #endif

  #ifndef INS_VFF_VZ_R_GPS
  #define INS_VFF_VZ_R_GPS 2.0
  #endif
#endif // USE_GPS

/** maximum number of propagation steps without any updates in between */
#ifndef INS_MAX_PROPAGATION_STEPS
  #define INS_MAX_PROPAGATION_STEPS 200
#endif

#ifndef USE_INS_NAV_INIT
  #define USE_INS_NAV_INIT TRUE
  PRINT_CONFIG_MSG("USE_INS_NAV_INIT defaulting to TRUE")
#endif

/** default barometer to use in INS */
#define INS_BARO_MAX_INIT_VAR 1.f  // variance threshold to set initial baro measurement
#ifndef INS_INT_BARO_ID
#if USE_BARO_BOARD
  #define INS_INT_BARO_ID BARO_BOARD_SENDER_ID
#else
  #define INS_INT_BARO_ID ABI_BROADCAST
#endif
#endif
PRINT_CONFIG_VAR(INS_INT_BARO_ID)
abi_event baro_ev;

/** ABI binding for IMU data.
 * Used accel ABI messages.
 */
#ifndef INS_INT_IMU_ID
#define INS_INT_IMU_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_INT_IMU_ID)
static abi_event accel_ev;
static void accel_cb(uint8_t sender_id, uint32_t stamp, struct Int32Vect3 *accel);

#ifndef INS_INT_GPS_ID
#define INS_INT_GPS_ID GPS_MULTI_ID
#endif
PRINT_CONFIG_VAR(INS_INT_GPS_ID)
static abi_event gps_ev;
static void gps_cb(uint8_t sender_id, uint32_t stamp, struct GpsState *gps_s);


#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void send_ins(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_INS(trans, dev, AC_ID,
                    &ins_int.ltp_pos.x, &ins_int.ltp_pos.y, &ins_int.ltp_pos.z,
                    &ins_int.ltp_speed.x, &ins_int.ltp_speed.y, &ins_int.ltp_speed.z,
                    &ins_int.ltp_accel.x, &ins_int.ltp_accel.y, &ins_int.ltp_accel.z);
}

static void send_ins_z(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_INS_Z(trans, dev, AC_ID,
                      &ins_int.baro_z, &ins_int.ltp_pos.z, &ins_int.ltp_speed.z, &ins_int.ltp_accel.z);
}

static void send_ins_ref(struct transport_tx *trans, struct link_device *dev)
{
  if (ins_int.ltp_initialized) {
    pprz_msg_send_INS_REF(trans, dev, AC_ID,
                          &ins_int.ltp_def.ecef.x, &ins_int.ltp_def.ecef.y, &ins_int.ltp_def.ecef.z,
                          &ins_int.ltp_def.lla.lat, &ins_int.ltp_def.lla.lon, &ins_int.ltp_def.lla.alt,
                          &ins_int.ltp_def.hmsl, &ins_int.qfe);
  }
}

static void send_slam(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t converted = (uint8_t)wall_system.converted_to_ltp;
  pprz_msg_send_SLAM(trans, dev, AC_ID,
                     (float[2]){offset.x, offset.y},
                     (float[2]){gps_offset.x, gps_offset.y},
                     (float[2]){nearest_point.x, nearest_point.y},
                     (float[2]){obstacle.x, obstacle.y},
                     &converted);
}

#endif    // PERIODIC_TELEMETRY

static void ins_ned_to_state(void);
static void ins_reset_local_origin(void);


void ins_int_init(void)
{

  #if USE_INS_NAV_INIT
    ins_init_origin_i_from_flightplan(MODULE_INS_SLAM_ID, &ins_int.ltp_def);
    ins_int.ltp_initialized = true;
  #else
    ins_int.ltp_initialized  = false;
  #endif

  init_walls();
  
  // DEBUG LAB
  ins_int.ltp_initialized  = true;

  /* we haven't had any measurement updates yet, so set the counter to max */
  ins_int.propagation_cnt = INS_MAX_PROPAGATION_STEPS;

  INT32_VECT3_ZERO(ins_int.ltp_pos);
  INT32_VECT3_ZERO(ins_int.ltp_speed);
  INT32_VECT3_ZERO(ins_int.ltp_accel);

  #if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS, send_ins);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS_Z, send_ins_z);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS_REF, send_ins_ref);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_SLAM, send_slam);
  #endif

  /*
   * Subscribe to scaled IMU and GPS measurements
   */
  AbiBindMsgIMU_ACCEL(INS_INT_IMU_ID, &accel_ev, accel_cb);
  AbiBindMsgGPS(INS_INT_GPS_ID, &gps_ev, gps_cb);
  AbiBindMsgLIDAR_SERVO(AGL_LIDAR_TFMINI_ID, &lidar_ev, lidar_cb);

}

static void ins_reset_local_origin(void)
{
  #if USE_GPS
    if (GpsFixValid()) {
      struct EcefCoor_i ecef_pos = ecef_int_from_gps(&gps);
      struct LlaCoor_i lla_pos = lla_int_from_gps(&gps);
      ltp_def_from_ecef_i(&ins_int.ltp_def, &ecef_pos);
      ins_int.ltp_def.lla.alt = lla_pos.alt;
      ins_int.ltp_def.hmsl = gps.hmsl;
      ins_int.ltp_initialized = true;
      stateSetLocalOrigin_i(MODULE_INS_SLAM_ID, &ins_int.ltp_def);
    } else {
      ins_int.ltp_initialized = false;
    }
  #else
    ins_int.ltp_initialized = false;
  #endif

  #if USE_HFF
    ins_int.hf_realign = true;
  #endif
    ins_int.vf_reset = true;
}


void ins_int_propagate(struct Int32Vect3 *accel, float dt)
{

  (void)dt; // I dont need this (avoid the warning)

  // Set body acceleration in the state
  stateSetAccelBody_i(MODULE_INS_SLAM_ID, accel);
  struct FloatVector body_accel;   // Aceleración en ejes cuerpo

  // Esta en int32, dividir entre 1024 para obtener el valor en m/s2
  body_accel.x = ACCEL_FLOAT_OF_BFP(accel->x);
  body_accel.y = ACCEL_FLOAT_OF_BFP(accel->y);
  body_accel.z = ACCEL_FLOAT_OF_BFP(accel->z) + 9.81;  // Aqui esta restando la gravedad

  // Velocidad angular (solo hace falta Z)
  struct FloatRates *ang_vel;
  ang_vel = stateGetBodyRates_f();

  // float U[3] = {body_accel.x, body_accel.y, ang_vel->r};
  // extended_kalman_filter_predict(&kalman_filter, U, dt);

  // Actualiza (y manda por telemetria), las aceleraciones en ejes cuerpo y la v. angular en Z
  ins_int.ltp_accel.x = ACCEL_BFP_OF_REAL(body_accel.x);
  ins_int.ltp_accel.y = ACCEL_BFP_OF_REAL(body_accel.y);
  ins_int.ltp_accel.z = ACCEL_BFP_OF_REAL(body_accel.z);
  ins_int.ltp_speed.z = SPEED_BFP_OF_REAL(ang_vel->r);  // Esta es para poder ver el mensaje

  // -------------------------------------------------------------
  // De aqui ...

  // if (counter_test > 5){

  //   // Paparazzi mueve los puntos en ENU, pero la posicion se mueve en NED
  //   //waypoint_move_xy_i(16, POS_BFP_OF_REAL(obstacle.x), POS_BFP_OF_REAL(obstacle.y)); // DEBUG
  //   //waypoint_move_xy_i(15, POS_BFP_OF_REAL(nearest_point.y), POS_BFP_OF_REAL(nearest_point.x)); // DEBUG

  //   struct NedCoor_i simulated_gps_pos = {100.0, 100.0, 0.0}; // cm
  //   struct NedCoor_i simulated_gps_vel = {0.0, 0.0, 0.0};

  //   // Aqui hace la correccion del offset
  //   gps_offset = offset;
  //   offset = (struct FloatVect2){0.0f, 0.0f};
  
  //   // TODO: Hay discrepancias usando ned y enu (de momento le doy la vuelta)
  //   if(ins_slam.enable){
  //     ins_int.ltp_pos.x = POS_BFP_OF_REAL(simulated_gps_pos.x/100.0f + gps_offset.x);
  //     ins_int.ltp_pos.y = POS_BFP_OF_REAL(simulated_gps_pos.y/100.0f + gps_offset.y);
  //   }
  //   else{
  //     INT32_VECT2_SCALE_2(ins_int.ltp_pos, simulated_gps_pos,
  //                           INT32_POS_OF_CM_NUM, INT32_POS_OF_CM_DEN);
  //   }
  
  //   INT32_VECT2_SCALE_2(ins_int.ltp_speed, simulated_gps_vel,
  //     INT32_SPEED_OF_CM_S_NUM, INT32_SPEED_OF_CM_S_DEN);

  // }
  // else{
  //   counter_test++;
  // }

  // ... a aqui va en el GPS (para depurar cuando no hay señal)
  // -------------------------------------------------------------

  ins_ned_to_state();

  /* increment the propagation counter, while making sure it doesn't overflow */
  if (ins_int.propagation_cnt < 100 * INS_MAX_PROPAGATION_STEPS) {
    ins_int.propagation_cnt++;
    dt = 0; // Para quitar el error
  }
}



#if USE_GPS
void ins_int_update_gps(struct GpsState *gps_s)
{
  if (gps_s->fix < GPS_FIX_3D) {
    return;
  }

  if (!wall_system.converted_to_ltp) {
    convert_walls_to_ltp();
  }

  if (!ins_int.ltp_initialized) {
    ins_reset_local_origin();
  }

  struct NedCoor_i gps_pos_cm_ned;
  struct EcefCoor_i ecef_pos_i = ecef_int_from_gps(gps_s);
  ned_of_ecef_point_i(&gps_pos_cm_ned, &ins_int.ltp_def, &ecef_pos_i);

  /* calculate body frame position taking BODY_TO_GPS translation (in cm) into account */
  #ifdef INS_BODY_TO_GPS_X
    /* body2gps translation in body frame */
    struct Int32Vect3 b2g_b = {
      .x = INS_BODY_TO_GPS_X,
      .y = INS_BODY_TO_GPS_Y,
      .z = INS_BODY_TO_GPS_Z
    };
    /* rotate offset given in body frame to navigation/ltp frame using current attitude */
    struct Int32Quat q_b2n = *stateGetNedToBodyQuat_i();
    QUAT_INVERT(q_b2n, q_b2n);
    struct Int32Vect3 b2g_n;
    int32_quat_vmult(&b2g_n, &q_b2n, &b2g_b);
    /* subtract body2gps translation in ltp from gps position */
    VECT3_SUB(gps_pos_cm_ned, b2g_n);
  #endif

  /// @todo maybe use gps_s->ned_vel directly??
  struct NedCoor_i gps_speed_cm_s_ned;
  struct EcefCoor_i ecef_vel_i = ecef_vel_int_from_gps(gps_s);
  ned_of_ecef_vect_i(&gps_speed_cm_s_ned, &ins_int.ltp_def, &ecef_vel_i);


  // Aqui hace la correccion del offset
  gps_offset = offset;
  offset = (struct FloatVect2){0.0f, 0.0f};

  if(ins_slam.enable){
    ins_int.ltp_pos.x = POS_BFP_OF_REAL(gps_pos_cm_ned.x/100.0f + gps_offset.x);
    ins_int.ltp_pos.y = POS_BFP_OF_REAL(gps_pos_cm_ned.y/100.0f + gps_offset.y);
  }
  else{
    INT32_VECT2_SCALE_2(ins_int.ltp_pos, gps_pos_cm_ned,
                          INT32_POS_OF_CM_NUM, INT32_POS_OF_CM_DEN);
  }

  INT32_VECT2_SCALE_2(ins_int.ltp_speed, gps_speed_cm_s_ned,
    INT32_SPEED_OF_CM_S_NUM, INT32_SPEED_OF_CM_S_DEN);

  ins_ned_to_state();
  waypoint_move_xy_i(16, POS_BFP_OF_REAL(obstacle.x), POS_BFP_OF_REAL(obstacle.y)); // DEBUG

  /* reset the counter to indicate we just had a measurement update */
  ins_int.propagation_cnt = 0;
}
#else
void ins_int_update_gps(struct GpsState *gps_s __attribute__((unused))) {}
#endif /* USE_GPS */


void ins_update_lidar(float distance, float angle){

  // struct FloatVect2 obstacle;

  if (distance < ins_slam.min_distance || distance > ins_slam.max_distance) {
    return;
  } 

  // // Desactivado hasta que termine de probar en el lab
  // if (!ins_int.ltp_initialized) {
  //     return; // No se corrige si la posición del rover no está inicializada
  // }

  // Asegura que los obstaculos esten inicializados
  if(!wall_system.converted_to_ltp){
    convert_walls_to_ltp();
  }


  // Obtener posición actual del rover en coordenadas locales
  float x_rover = POS_FLOAT_OF_BFP(ins_int.ltp_pos.x);
  float y_rover = POS_FLOAT_OF_BFP(ins_int.ltp_pos.y);
  float theta = ahrs_dcm.ltp_to_body_euler.psi;

  float corrected_angle = M_PI / 2 - angle*M_PI/180 - theta;

  obstacle.x = x_rover + (distance * cosf(corrected_angle));
  obstacle.y = y_rover + (distance * sinf(corrected_angle));

  // Obtener el punto más cercano en la pared conocida
  nearest_point = (struct FloatVect2){0.0f, 0.0f};
  float distance_wall = find_nearest_wall(&obstacle, &nearest_point);
  if(distance_wall > ins_slam.max_distance_wall){
    return;
  } 

  float delta_x = nearest_point.x - obstacle.x;
  float delta_y = nearest_point.y - obstacle.y;
  offset = (struct FloatVect2){delta_x, delta_y};

  debug_point.x = nearest_point.x;
  debug_point.y = nearest_point.y;

}


/** copy position and speed to state interface */
static void ins_ned_to_state(void)
{
  stateSetPositionNed_i(MODULE_INS_SLAM_ID, &ins_int.ltp_pos);
  stateSetSpeedNed_i(MODULE_INS_SLAM_ID, &ins_int.ltp_speed);
  stateSetAccelNed_i(MODULE_INS_SLAM_ID, &ins_int.ltp_accel);

  #if defined SITL && USE_NPS
    if (nps_bypass_ins) {
      sim_overwrite_ins();
    }
  #endif
}

static void accel_cb(uint8_t sender_id __attribute__((unused)),
                     uint32_t stamp, struct Int32Vect3 *accel)
{
  PRINT_CONFIG_MSG("Calculating dt for INS int propagation.")
  /* timestamp in usec when last callback was received */
  static uint32_t last_stamp = 0;

  if (last_stamp > 0) {
    float dt = (float)(stamp - last_stamp) * 1e-6;
    ins_int_propagate(accel, dt);
  }
  last_stamp = stamp;
}

static void gps_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp __attribute__((unused)),
                   struct GpsState *gps_s)
{
  ins_int_update_gps(gps_s);
}


static void lidar_cb(uint8_t __attribute__((unused)) sender_id,
                       uint32_t stamp __attribute__((unused)),
                       float distance, float angle)
{
  ins_update_lidar(distance, angle);
}


