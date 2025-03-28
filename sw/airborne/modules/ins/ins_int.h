/*
 * Copyright (C) 2008-2012 The Paparazzi Team
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
 * @file modules/ins/ins_int.h
 *
 * INS for rotorcrafts combining vertical and horizontal filters.
 *
 */

#ifndef INS_INT_H
#define INS_INT_H

#include "modules/ins/ins.h"
#include "modules/gps/gps.h"
#include "std.h"
#include "math/pprz_geodetic_int.h"
#include "math/pprz_algebra_float.h"

/** Ins implementation state (fixed point) */
struct InsInt {
  struct LtpDef_i  ltp_def;
  bool           ltp_initialized;

  uint32_t propagation_cnt; ///< number of propagation steps since the last measurement update

  /** request to realign horizontal filter.
   * Sets to current position (local origin unchanged).
   */
  bool hf_realign;

  /** request to reset vertical filter.
   * Sets the z-position to zero and resets the the z-reference to current altitude.
   */
  bool vf_reset;

  /* output LTP NED */
  struct NedCoor_i ltp_pos;
  struct NedCoor_i ltp_speed;
  struct NedCoor_i ltp_accel;

  /* baro */
  float baro_z;  ///< z-position calculated from baro in meters (z-down)
  float qfe;
  bool baro_initialized;
};

/** global INS state */
extern struct InsInt ins_int;

extern void ins_int_init(void);
extern void ins_int_propagate(struct Int32Vect3 *accel, float dt);
extern void ins_int_update_gps(struct GpsState *gps_s);

#endif /* INS_INT_H */

#ifdef USE_KF_FILTER
  #include "filters/linear_kalman_filter.h"
  #ifndef KALMAN_FILTER_H
  #define KALMAN_FILTER_H
  extern struct linear_kalman_filter kalman_filter;
  #endif
#endif


#ifdef USE_EKF_FILTER
  #include "filters/extended_kalman_filter.h"
  #ifndef KALMAN_FILTER_H
    #define KALMAN_FILTER_H
    extern bool enable_ekf_filter;
    struct KalmanVariance {
      float imu;
      float pos;
      float vel;
      float att;
    };
    extern struct KalmanVariance kalman_variance;
    extern struct extended_kalman_filter kalman_filter;
    #ifndef R2_IMU
      #define R2_IMU 25
    #endif
    #ifndef RP_GPS
      #define RP_GPS 5 
    #endif
    #ifndef RV_GPS
      #define RV_GPS 10
    #endif
    #ifndef RT
      #define RT 10
    #endif
  #endif
#endif
// extern struct InsInt ins_int;








