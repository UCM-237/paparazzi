/*
 * Copyright (C) 2008-2010 The Paparazzi Team
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
 * INS for rotorcrafts combining vertical and horizontal filters.
 *
 * @authors Alejandro Rochas Fernández
 *
 */



// KALMAN FILTER ----------------------
#include "filters/extended_kalman_filter.c"

#define R2_IMU 2.5E-4       // Varianza sobre la IMU
#define RP_GPS 0.005        // Varianza sobre la posición (REVISAR)
#define RV_GPS 1            // Varianza sobre la velocidad (REVISAR)
#define RT 0.1              // Varianza sobre la actitud

#define DELTA_T  0.5  // Tiempo entre medidas por defecto

struct extended_kalman_filter kalman_filter;
uint8_t time_calculated = 0;    // Esto es lo mejor que se me ha ocurrido por ahora

struct InsInt ins_int;
struct FloatVector {
  float x;  ///< North
  float y;  ///< East
  float z;  ///< Down
};
struct FloatVector world_accel;   // Aceleración en ejes mundo

// ------------------------------------

#include "modules/core/abi.h"

#include "modules/imu/imu.h"
#include "modules/gps/gps.h"

#include "generated/airframe.h"

#if USE_VFF_EXTENDED
#include "modules/ins/vf_extended_float.h"
#else
#include "modules/ins/vf_float.h"
#endif

// #if USE_HFF
// #include "modules/ins/hf_float.h"
// #endif

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

// #if USE_SONAR
// #if !USE_VFF_EXTENDED
// #error USE_SONAR needs USE_VFF_EXTENDED
// #endif

// #ifdef INS_SONAR_THROTTLE_THRESHOLD
// #include "firmwares/rotorcraft/stabilization.h"
// #endif

// #ifndef INS_SONAR_MIN_RANGE
// #define INS_SONAR_MIN_RANGE 0.001
// #endif
// #ifndef INS_SONAR_MAX_RANGE
// #define INS_SONAR_MAX_RANGE 4.0
// #endif
// #define VFF_R_SONAR_0 0.2
// #ifndef VFF_R_SONAR_OF_M
// #define VFF_R_SONAR_OF_M 0.2
// #endif

// #endif // USE_SONAR

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
// static void baro_cb(uint8_t sender_id, uint32_t stamp, float pressure);

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


/** ABI binding for AGL.
 * Usually this is comes from sonar or gps.
 */
// #ifndef INS_INT_AGL_ID
// #define INS_INT_AGL_ID ABI_BROADCAST
// #endif
// PRINT_CONFIG_VAR(INS_INT_AGL_ID)
// static abi_event agl_ev;                 ///< The agl ABI event
// static void agl_cb(uint8_t sender_id, uint32_t stamp, float distance);



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
#endif

static void ins_ned_to_state(void);
// static void ins_update_from_vff(void);
// #if USE_HFF
// static void ins_update_from_hff(void);
// #endif



void init_filter(struct extended_kalman_filter *filter, float dt){

  uint8_t n = 5; // [px, py, vx, vy, theta]
  uint8_t c = 3; // [ax, ay, az]
  uint8_t m = 5; // Measurement Vector

  extended_kalman_filter_init(filter, n, c, m);

  filter->n = n;
  filter->c = c;
  filter->m = m;


  // float A[4][4] = {
  //       {1, 0, dt, 0},
  //       {0, 1, 0, dt},
  //       {0, 0, 1, 0},
  //       {0, 0, 0, 1}
  // };

  // float B[4][2] = {
  //       {0,  0},
  //       {0,  0},
  //       {dt, 0},
  //       {0, dt}
  // };

  float H[5][5] = {
        {1, 0, 0, 0, 0},
        {0, 1, 0, 0, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 0, 1, 0},
        {0, 0, 0, 0, 1}
  };

  float Q[5][5] = {
        {R2_IMU*pow(dt, 4), 0, 0, 0, 0},
        {0, R2_IMU*pow(dt, 4), 0, 0, 0},
        {0, 0, R2_IMU*pow(dt, 4), 0, 0},
        {0, 0, 0, R2_IMU*pow(dt, 4), 0},
        {0, 0, 0, 0, R2_IMU*pow(dt, 4)}
  };

  float R[5][5] = {
        {RP_GPS, 0, 0, 0, 0},
        {0, RP_GPS, 0, 0, 0},
        {0, 0, RV_GPS, 0, 0},
        {0, 0, 0, RV_GPS, 0},
        {0, 0, 0, 0, RT}
  };

  const int P_init = 100;
  float P[5][5] = {
        {P_init, 0, 0, 0, 0},
        {0, P_init, 0, 0, 0},
        {0, 0, P_init, 0, 0},
        {0, 0, 0, P_init, 0},
        {0, 0, 0, 0, P_init}
  };

  float X[5] = {0, 0, 0, 0, 0};

  // memcpy(filter->A, A, sizeof(A));
  // memcpy(filter->B, B, sizeof(B));
  memcpy(filter->H, H, sizeof(H));
  memcpy(filter->Q, Q, sizeof(Q));
  memcpy(filter->R, R, sizeof(R));
  memcpy(filter->P, P, sizeof(P));
  memcpy(filter->X, X, sizeof(X));

}


void ins_int_init(void)
{

  #if USE_INS_NAV_INIT
    ins_init_origin_i_from_flightplan(&ins_int.ltp_def);
    ins_int.ltp_initialized = true;
  #else
    ins_int.ltp_initialized  = false;
  #endif

  init_filter(&kalman_filter, DELTA_T);   // DEFINICION DEL FILTRO DE KALMAN

  /* we haven't had any measurement updates yet, so set the counter to max */
  ins_int.propagation_cnt = INS_MAX_PROPAGATION_STEPS;

  // Bind to BARO_ABS message
  // AbiBindMsgBARO_ABS(INS_INT_BARO_ID, &baro_ev, baro_cb);
  // ins_int.baro_initialized = false;

  // ins_int.vf_reset = false;
  // ins_int.hf_realign = false;

  /* init vertical and horizontal filters */
  // vff_init_zero();
  // #if USE_HFF
  //   hff_init(0., 0., 0., 0.);
  // #endif

  INT32_VECT3_ZERO(ins_int.ltp_pos);
  INT32_VECT3_ZERO(ins_int.ltp_speed);
  INT32_VECT3_ZERO(ins_int.ltp_accel);

  #if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS, send_ins);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS_Z, send_ins_z);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS_REF, send_ins_ref);
  #endif

  /*
   * Subscribe to scaled IMU measurements and attach callbacks
   */
  AbiBindMsgIMU_ACCEL(INS_INT_IMU_ID, &accel_ev, accel_cb);
  AbiBindMsgGPS(INS_INT_GPS_ID, &gps_ev, gps_cb);
  // AbiBindMsgAGL(INS_INT_AGL_ID, &agl_ev, agl_cb); // ABI to the altitude above ground level
}

void ins_reset_local_origin(void)
{
  #if USE_GPS
    if (GpsFixValid()) {
      struct EcefCoor_i ecef_pos = ecef_int_from_gps(&gps);
      struct LlaCoor_i lla_pos = lla_int_from_gps(&gps);
      ltp_def_from_ecef_i(&ins_int.ltp_def, &ecef_pos);
      ins_int.ltp_def.lla.alt = lla_pos.alt;
      ins_int.ltp_def.hmsl = gps.hmsl;
      ins_int.ltp_initialized = true;
      stateSetLocalOrigin_i(&ins_int.ltp_def);
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

void ins_reset_altitude_ref(void)
{
  #if USE_GPS
    if (GpsFixValid()) {
      struct LlaCoor_i lla_pos = lla_int_from_gps(&gps);
      struct LlaCoor_i lla = {
        .lat = state.ned_origin_i.lla.lat,
        .lon = state.ned_origin_i.lla.lon,
        .alt = lla_pos.alt
      };
      ltp_def_from_lla_i(&ins_int.ltp_def, &lla);
      ins_int.ltp_def.hmsl = gps.hmsl;
      stateSetLocalOrigin_i(&ins_int.ltp_def);
    }
  #endif
  ins_int.vf_reset = true;
}

void ins_reset_vertical_pos(void)
{
  ins_int.vf_reset = true;
}


void ins_int_propagate(struct Int32Vect3 *accel, float dt)
{
  // Set body acceleration in the state
  stateSetAccelBody_i(accel);

  // Cambia de ejes cuerpo a ejes mundo --> Esta en int32, 
  // dividir entre 1024 para obtener el valor en m/s2
  world_accel.x = ACCEL_FLOAT_OF_BFP(accel->x);
  world_accel.y = ACCEL_FLOAT_OF_BFP(accel->y);
  world_accel.z = ACCEL_FLOAT_OF_BFP(accel->z) + 9.81;  // Aqui esta restando la gravedad

  float U[3] = {world_accel.x, world_accel.y, world_accel.z};
  extended_kalman_filter_predict(&kalman_filter, U, dt);

  ins_int.ltp_accel.x = ACCEL_BFP_OF_REAL(world_accel.x);
  ins_int.ltp_accel.y = ACCEL_BFP_OF_REAL(world_accel.y);
  ins_int.ltp_accel.z = ACCEL_BFP_OF_REAL(world_accel.z);
  

  // TEMPORAL: Para probar las unidades del estado
  // ins_int.ltp_pos.x = POS_BFP_OF_REAL(kalman_filter.X[0]);
  // ins_int.ltp_pos.y = POS_BFP_OF_REAL(kalman_filter.X[1]);
  // ins_int.ltp_speed.x = POS_BFP_OF_REAL(kalman_filter.X[2]);
  // ins_int.ltp_speed.y = POS_BFP_OF_REAL(kalman_filter.X[3]);;
  
  ins_ned_to_state();

  /* increment the propagation counter, while making sure it doesn't overflow */
  if (ins_int.propagation_cnt < 100 * INS_MAX_PROPAGATION_STEPS) {
    ins_int.propagation_cnt++;
  }
}

// static void baro_cb(uint8_t __attribute__((unused)) sender_id, __attribute__((unused)) uint32_t stamp, float pressure)
// {
//   if (pressure < 1.f)
//   {
//     // bad baro pressure, don't use
//     return;
//   }

//   if (!ins_int.baro_initialized) {
// #define press_hist_len 10
//     static float press_hist[press_hist_len];
//     static uint8_t idx = 0;

//     press_hist[idx] = pressure;
//     idx = (idx + 1) % press_hist_len;
//     float var = variance_f(press_hist, press_hist_len);
//     if (var < INS_BARO_MAX_INIT_VAR){
//       // wait for a first positive value
//       ins_int.vf_reset = true;
//       ins_int.baro_initialized = true;
//     }
//   }

//   if (ins_int.baro_initialized) {
//     float height_correction = 0.f;
//     if(ins_int.ltp_initialized){
//       // Calculate the distance to the origin
//       struct EnuCoor_f *enu = stateGetPositionEnu_f();
//       double dist2_to_origin = enu->x * enu->x + enu->y * enu->y;

//       // correction for the earth's curvature
//       const double earth_radius = 6378137.0;
//       height_correction = (float)(sqrt(earth_radius * earth_radius + dist2_to_origin) - earth_radius);
//     }

//     if (ins_int.vf_reset) {
//       ins_int.vf_reset = false;
//       ins_int.qfe = pressure;
//       vff_realign(height_correction);
//       ins_update_from_vff();
//     }

//     float baro_up = pprz_isa_height_of_pressure(pressure, ins_int.qfe);

//     // The VFF will update in the NED frame
//     ins_int.baro_z = -(baro_up - height_correction);

// #if USE_VFF_EXTENDED
//     vff_update_baro(ins_int.baro_z);
// #else
//     vff_update(ins_int.baro_z);
// #endif

//     /* reset the counter to indicate we just had a measurement update */
//     ins_int.propagation_cnt = 0;
//   }
// }

#if USE_GPS
void ins_int_update_gps(struct GpsState *gps_s)
{
  if (gps_s->fix < GPS_FIX_3D) {
    return;
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

  // struct NedCoor_i pos_est;
  // struct NedCoor_i vel_est;

  struct FloatEulers *att = stateGetNedToBodyEulers_f();
  float theta = att->psi;   // Este es el angulo

  // Vector de Medidas (ESTO ES LO UNICO QUE ME FALTA POR REVISAR, creo que esta bien)
  float Y[5];
  Y[0] = gps_pos_cm_ned.x/100.0f;
  Y[1] = gps_pos_cm_ned.y/100.0f;
  Y[2] = gps_speed_cm_s_ned.x/100.0f;
  Y[3] = gps_speed_cm_s_ned.y/100.0f;
  Y[4] = theta;

  extended_kalman_filter_update(&kalman_filter, Y);

  ins_int.ltp_pos.x = POS_BFP_OF_REAL(kalman_filter.X[0]);
  ins_int.ltp_pos.y = POS_BFP_OF_REAL(kalman_filter.X[1]);
  ins_int.ltp_speed.x = POS_BFP_OF_REAL(kalman_filter.X[2]);
  ins_int.ltp_speed.y = POS_BFP_OF_REAL(kalman_filter.X[3]);

  ins_ned_to_state();

  /* reset the counter to indicate we just had a measurement update */
  ins_int.propagation_cnt = 0;
}
#else
void ins_int_update_gps(struct GpsState *gps_s __attribute__((unused))) {}
#endif /* USE_GPS */

/** agl_cb
 * This callback handles all estimates of the height of the vehicle above the ground under it
 * This is only used with the extended version of the vertical float filter
 */
// #if USE_VFF_EXTENDED
// static void agl_cb(uint8_t __attribute__((unused)) sender_id, __attribute__((unused)) uint32_t stamp, float distance) {
//   if (distance <= 0 || !(ins_int.baro_initialized)) {
//     return;
//   }

// #if USE_SONAR
//   if (distance > INS_SONAR_MAX_RANGE || distance < INS_SONAR_MIN_RANGE){
//     return;
//   }
// #endif
// #ifdef INS_AGL_THROTTLE_THRESHOLD
//    if(stabilization.cmd[COMMAND_THRUST] < INS_AGL_THROTTLE_THRESHOLD){
//      return;
//    }
// #endif
// #ifdef INS_AGL_BARO_THRESHOLD
//   if(ins_int.baro_z < -INS_SONAR_BARO_THRESHOLD){ /* z down */
//     return;
//   }
// #endif


/** copy position and speed to state interface */
static void ins_ned_to_state(void)
{
  stateSetPositionNed_i(&ins_int.ltp_pos);
  stateSetSpeedNed_i(&ins_int.ltp_speed);
  stateSetAccelNed_i(&ins_int.ltp_accel);

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


