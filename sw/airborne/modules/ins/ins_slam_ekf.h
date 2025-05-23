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
 * @file modules/ins/ins_int.h
 *
 * INS + SLAM Correction for the rovers using the TFMini Lidar.
 * This version use the Extended Kalman Filter.
 *
 */


#ifndef INS_SLAM_EKF_H
#define INS_SLAM_EKF_H

#include "filters/extended_kalman_filter.h"
#include "math/pprz_algebra_float.h"

#ifndef R2_IMU
  #define R2_IMU 25E-05
#endif
#ifndef RP_GPS
  #define RP_GPS 5E-03
#endif
#ifndef RV_GPS
  #define RV_GPS 10E-03
#endif
#ifndef RT
  #define RT 10E-03
#endif


struct KalmanVariance {
  float imu;
  float pos;
  float vel;
  float att;
  struct FloatVect2 delta;
  float psi;  // Angulo de la pared
};

extern struct KalmanVariance kalman_variance;
extern struct extended_kalman_filter kalman_filter;

struct InsSlam {
  bool enable;
  float min_distance;
  float max_distance;
  float max_distance_wall;
  float alpha;
  float beta;
  struct FloatVect2 gps_bias;
};

extern struct InsSlam ins_slam;

extern void ins_slam_init(void);
extern void ins_update_lidar(const float distance, const float angle);

#endif /* INS_SLAM_EKF_H */








