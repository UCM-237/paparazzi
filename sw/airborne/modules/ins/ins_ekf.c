/*
 * Copyright (C) 2024 Alejandro Rochas Fernández <alrochas@ucm.es>
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
 * @file modules/ins/ins_ekf.c
 *
 * INS for the rovers using Extended Kalman Filter.
 *
 */

// ------------------------------------

#include "autopilot.h"
#include "modules/ins/ins_int.h"
#include "modules/ins/ins_ekf.h"

// KALMAN FILTER ----------------------
// #include "filters/jacobian.h"
#include "filters/extended_kalman_filter.h"

#define DELTA_T  0.008  // Tiempo entre medidas por defecto

bool enable_ekf_filter = true;
struct extended_kalman_filter kalman_filter;
struct KalmanVariance kalman_variance;
struct InsInt ins_int;
struct FloatVector {
  float x;  ///< North
  float y;  ///< East
  float z;  ///< Down
};

uint8_t counter_test = 0;   // Esto es para pruebas

#include "modules/core/abi.h"

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

static abi_event reset_ev;
static void reset_cb(uint8_t sender_id, uint8_t flag);

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

static void send_ins_ekf(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_INS_EKF(trans, dev, AC_ID,
                    kalman_filter.X, kalman_filter.K2[1], kalman_filter.K2[2], kalman_filter.K2[3], kalman_filter.K2[4]);
}


#endif    // PERIODIC_TELEMETRY

static void ins_ned_to_state(void);

void update_matrix(struct extended_kalman_filter *filter){
  
  filter->R[0][0] = kalman_variance.pos*1E-03;
  filter->R[1][1] = kalman_variance.pos*1E-03;
  filter->R[2][2] = kalman_variance.vel*1E-03;
  filter->R[3][3] = kalman_variance.vel*1E-03;
  filter->R[4][4] = kalman_variance.att*1E-03;

  filter->Q[0][0] = kalman_variance.imu*1E-05;
  filter->Q[1][1] = kalman_variance.imu*1E-05;
  filter->Q[2][2] = kalman_variance.imu*1E-05;
  filter->Q[3][3] = kalman_variance.imu*1E-05;
  filter->Q[4][4] = kalman_variance.imu*1E-05;

}


void init_filter(struct extended_kalman_filter *filter, float dt){

  dt = dt;    // Para evitar el warning

  uint8_t n = 5; // [px, py, vx, vy, theta]
  uint8_t c = 3; // [ax, ay, az]
  uint8_t m = 5; // Measurement Vector

  extended_kalman_filter_init(filter, n, c, m);

  // You need to define this funcions in sw/airborne/filters/jacobian.c
  filter->f = ekf_f;
  filter->compute_F = ekf_compute_F;

  filter->n = n;
  filter->c = c;
  filter->m = m;

  filter->H[0][0] = 1;
  filter->H[1][1] = 1;
  filter->H[2][2] = 1;
  filter->H[3][3] = 1;
  filter->H[4][4] = 1;

  kalman_variance.pos = RP_GPS;
  kalman_variance.vel = RV_GPS;
  kalman_variance.att = RT;
  kalman_variance.imu = R2_IMU;

  const float P_init = 0.5;   // Hay que dejarlo menor que 1 (sino se vuelve inestable)
  filter->P[0][0] = P_init;
  filter->P[1][1] = P_init;
  filter->P[2][2] = P_init;
  filter->P[3][3] = P_init;
  filter->P[4][4] = P_init;


  update_matrix(filter);

}


void ins_int_init(void)
{

  #if USE_INS_NAV_INIT
    ins_init_origin_i_from_flightplan(MODULE_INS_EKF_ID, &ins_int.ltp_def);
    ins_int.ltp_initialized = true;
  #else
    ins_int.ltp_initialized  = false;
  #endif

  init_filter(&kalman_filter, DELTA_T);   // DEFINICION DEL FILTRO DE KALMAN

  /* we haven't had any measurement updates yet, so set the counter to max */
  ins_int.propagation_cnt = INS_MAX_PROPAGATION_STEPS;

  INT32_VECT3_ZERO(ins_int.ltp_pos);
  INT32_VECT3_ZERO(ins_int.ltp_speed);
  INT32_VECT3_ZERO(ins_int.ltp_accel);

  #if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS, send_ins);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS_Z, send_ins_z);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS_REF, send_ins_ref);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS_EKF, send_ins_ekf);
  #endif

  /*
   * Subscribe to scaled IMU and GPS measurements
   */
  AbiBindMsgIMU_ACCEL(INS_INT_IMU_ID, &accel_ev, accel_cb);
  AbiBindMsgGPS(INS_INT_GPS_ID, &gps_ev, gps_cb);
  AbiBindMsgINS_RESET(ABI_BROADCAST, &reset_ev, reset_cb);

}

static void reset_ref(void)
{
  #if USE_GPS
    if (GpsFixValid()) {
      struct EcefCoor_i ecef_pos = ecef_int_from_gps(&gps);
      struct LlaCoor_i lla_pos = lla_int_from_gps(&gps);
      ltp_def_from_ecef_i(&ins_int.ltp_def, &ecef_pos);
      ins_int.ltp_def.lla.alt = lla_pos.alt;
      ins_int.ltp_def.hmsl = gps.hmsl;
      ins_int.ltp_initialized = true;
      stateSetLocalOrigin_i(MODULE_INS_EKF_ID, &ins_int.ltp_def);
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


static void reset_vertical_ref(void)
{
#if USE_GPS
  if (GpsFixValid()) {
    struct LlaCoor_i lla_pos = lla_int_from_gps(&gps);
    struct LlaCoor_i lla = {
      .lat = stateGetLlaOrigin_i().lat,
      .lon = stateGetLlaOrigin_i().lon,
      .alt = lla_pos.alt
    };
    ltp_def_from_lla_i(&ins_int.ltp_def, &lla);
    ins_int.ltp_def.hmsl = gps.hmsl;
    stateSetLocalOrigin_i(MODULE_INS_EKF_ID, &ins_int.ltp_def);
  }
#endif
  ins_int.vf_reset = true;
}


static void reset_vertical_pos(void)
{
  ins_int.vf_reset = true;
}


void ins_int_propagate(struct Int32Vect3 *accel, float dt)
{

  update_matrix(&kalman_filter);  // Actualiza las matrices Q y R con los param de la GCS

  // Set body acceleration in the state
  stateSetAccelBody_i(MODULE_INS_EKF_ID, accel);
  struct FloatVector body_accel;   // Aceleración en ejes cuerpo

  // Esta en int32, dividir entre 1024 para obtener el valor en m/s2
  body_accel.x = ACCEL_FLOAT_OF_BFP(accel->x);
  body_accel.y = ACCEL_FLOAT_OF_BFP(accel->y);
  body_accel.z = ACCEL_FLOAT_OF_BFP(accel->z) + 9.81;  // Aqui esta restando la gravedad

  // Velocidad angular (solo hace falta Z)
  struct FloatRates *ang_vel;
  ang_vel = stateGetBodyRates_f();

  float U[3] = {body_accel.x, body_accel.y, ang_vel->r};
  extended_kalman_filter_predict(&kalman_filter, U, dt);

  // Actualiza (y manda por telemetria), las aceleraciones en ejes cuerpo y la v. angular en Z
  ins_int.ltp_accel.x = ACCEL_BFP_OF_REAL(body_accel.x);
  ins_int.ltp_accel.y = ACCEL_BFP_OF_REAL(body_accel.y);
  ins_int.ltp_accel.z = ACCEL_BFP_OF_REAL(body_accel.z);
  ins_int.ltp_speed.z = SPEED_BFP_OF_REAL(ang_vel->r);  // Esta es para poder ver el mensaje

  ins_ned_to_state();

  /* increment the propagation counter, while making sure it doesn't overflow */
  if (ins_int.propagation_cnt < 100 * INS_MAX_PROPAGATION_STEPS) {
    ins_int.propagation_cnt++;
  }
}



#if USE_GPS
void ins_int_update_gps(struct GpsState *gps_s)
{
  if (gps_s->fix < GPS_FIX_3D) {
    return;
  }

  if (!ins_int.ltp_initialized) {
    reset_ref();
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


  // Vector de Medidas
  float Y[5];
  Y[0] = gps_pos_cm_ned.x/100.0f;
  Y[1] = gps_pos_cm_ned.y/100.0f;
  Y[2] = gps_speed_cm_s_ned.x/100.0f;
  Y[3] = gps_speed_cm_s_ned.y/100.0f;
  Y[4] = ahrs_dcm.ltp_to_body_euler.psi;

  extended_kalman_filter_update(&kalman_filter, Y);

  if(enable_ekf_filter){
    ins_int.ltp_pos.x = POS_BFP_OF_REAL(kalman_filter.X[0]);
    ins_int.ltp_pos.y = POS_BFP_OF_REAL(kalman_filter.X[1]);
    ins_int.ltp_speed.x = SPEED_BFP_OF_REAL(kalman_filter.X[2]);
    ins_int.ltp_speed.y = SPEED_BFP_OF_REAL(kalman_filter.X[3]);
    // kalman_filter.X[4] is updated in ahrs_float_dcm_wrapper.c
  }
  else{
    INT32_VECT2_SCALE_2(ins_int.ltp_pos, gps_pos_cm_ned,
                          INT32_POS_OF_CM_NUM, INT32_POS_OF_CM_DEN);
    INT32_VECT2_SCALE_2(ins_int.ltp_speed, gps_speed_cm_s_ned,
                          INT32_SPEED_OF_CM_S_NUM, INT32_SPEED_OF_CM_S_DEN);
  }

  ins_ned_to_state();
  /* reset the counter to indicate we just had a measurement update */
  ins_int.propagation_cnt = 0;
}
#else
void ins_int_update_gps(struct GpsState *gps_s __attribute__((unused))) {}
#endif /* USE_GPS */


/** copy position and speed to state interface */
static void ins_ned_to_state(void)
{
  stateSetPositionNed_i(MODULE_INS_EKF_ID, &ins_int.ltp_pos);
  stateSetSpeedNed_i(MODULE_INS_EKF_ID, &ins_int.ltp_speed);
  stateSetAccelNed_i(MODULE_INS_EKF_ID, &ins_int.ltp_accel);

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


static void reset_cb(uint8_t sender_id UNUSED, uint8_t flag)
{
  switch (flag) {
    case INS_RESET_REF:
      reset_ref();
      break;
    case INS_RESET_VERTICAL_REF:
      reset_vertical_ref();
      break;
    case INS_RESET_VERTICAL_POS:
      reset_vertical_pos();
      break;
    default:
      // unsupported cases
      break;
  }
}


