/*
 * Copyright (C) 2025 Alejandro Rochas Fernandez <alrochas@ucm.es>
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
/**
 * @file "filters/jacobian.c"
 *
 * Here you can define the functions used in the 
 * extended kalman filter for the jacobian
 */


#include "filters/jacobian.h"
#include "filters/extended_kalman_filter.h"



/*******************************************************************************
 *                                                                             *
 *  EKF for IMU+GPS fusion                                                     *
 *                                                                             *
 ******************************************************************************/

// Modelo no lineal f
void ekf_f(struct extended_kalman_filter *filter, float *U, float dt) {
    float ax = U[0], ay = U[1], wz = U[2];
    float theta = filter->X[4];

    filter->X_pred[0] = filter->X[0] + filter->X[2] * dt;
    filter->X_pred[1] = filter->X[1] + filter->X[3] * dt;
    filter->X_pred[2] = filter->X[2] + (cosf(theta) * ax - sinf(theta) * ay) * dt;
    filter->X_pred[3] = filter->X[3] + (sinf(theta) * ax + cosf(theta) * ay) * dt;
    filter->X_pred[4] = filter->X[4] + wz * dt;
}


// Jacobiano F (default F = I)
void ekf_compute_F(struct extended_kalman_filter *filter, float *U, float dt) {
    float ax = U[0], ay = U[1];
    float theta = filter->X[4];

    filter->F[0][2] = dt;
    filter->F[1][3] = dt;
    filter->F[2][4] = -(sinf(theta) * ax + cosf(theta) * ay) * dt;
    filter->F[3][4] = (cosf(theta) * ax - sinf(theta) * ay) * dt;
}



/*******************************************************************************
 *                                                                             *
 *  EKF for Lidar+GPS fusion                                                   *
 *                                                                             *
 ******************************************************************************/

// Right now the filter is the same as the one for IMU+GPS fusion
// The difference is the Y vector.

// Modelo no lineal f (same as ekf_f)
void ekf_slam_f(struct extended_kalman_filter *filter, float *U, float dt) {
    float ax = U[0], ay = U[1], wz = U[2];
    float theta = filter->X[4];

    filter->X_pred[0] = filter->X[0] + filter->X[2] * dt;
    filter->X_pred[1] = filter->X[1] + filter->X[3] * dt;
    filter->X_pred[2] = filter->X[2] + (cosf(theta) * ax - sinf(theta) * ay) * dt;
    filter->X_pred[3] = filter->X[3] + (sinf(theta) * ax + cosf(theta) * ay) * dt;
    filter->X_pred[4] = filter->X[4] + wz * dt;
    filter->X_pred[5] = filter->X[5];
    filter->X_pred[6] = filter->X[6];
}

// Jacobiano F (default F = I)
void ekf_slam_compute_F(struct extended_kalman_filter *filter, float *U, float dt) {
    float ax = U[0], ay = U[1];
    float theta = filter->X[4];

    filter->F[0][2] = dt;
    filter->F[1][3] = dt;
    filter->F[2][4] = -(sinf(theta) * ax + cosf(theta) * ay) * dt;
    filter->F[3][4] = (cosf(theta) * ax - sinf(theta) * ay) * dt;
}

