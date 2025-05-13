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
 * @file modules/ins/ins_ekf.h
 *
 * INS for the rovers using Extended Kalman Filter.
 *
 */


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