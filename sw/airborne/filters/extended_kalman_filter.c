/*
 * Copyright (C) 2024 Alejandro Rochas Fernández <alrochas@ucm.es>
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
 * @file "filters/extended_kalman_filter.c"
 *
 * Extended Kalman Filter for the Rovers
 */

#include "filters/extended_kalman_filter.h"
#include "math/pprz_algebra_float.h"


/** Init all matrix and vectors to zero
 *
 * @param filter pointer to a filter structure
 * @param n size of the state vector
 * @param c size of the command vector
 * @param m size of the measurement vector
 * @return false if n, c or m are larger than the maximum value
 */
bool extended_kalman_filter_init(struct extended_kalman_filter *filter, uint8_t n, uint8_t c, uint8_t m)
{
  if (n > KF_MAX_STATE_SIZE || c > KF_MAX_CMD_SIZE || m > KF_MAX_MEAS_SIZE) {
    filter->n = 0;
    filter->c = 0;
    filter->m = 0;
    return false; // invalide sizes;
  }
  filter->n = n;
  filter->c = c;
  filter->m = m;

  // Matrix
  MAKE_MATRIX_PTR(_F, filter->F, n);
  float_mat_identity(_F, n, n);
    MAKE_MATRIX_PTR(_H, filter->H, m);
  float_mat_zero(_H, m, n);
  MAKE_MATRIX_PTR(_P, filter->P, n);
  float_mat_zero(_P, n, n);
  MAKE_MATRIX_PTR(_Q, filter->Q, n);
  float_mat_zero(_Q, n, n);
  MAKE_MATRIX_PTR(_R, filter->R, m);
  float_mat_zero(_R, m, m);

  // Vector
  float_vect_zero(filter->X, n);
  float_vect_zero(filter->X_pred, n);

  return true;
}


// Modelo no lineal f (solo valido para el este caso concreto)
void ekf_f(struct extended_kalman_filter *filter, float *U, float dt) {
    float ax = U[0], ay = U[1], az = U[2];
    float theta = filter->X[4];

    filter->X_pred[0] = filter->X[0] + filter->X[2] * dt;
    filter->X_pred[1] = filter->X[1] + filter->X[3] * dt;
    filter->X_pred[2] = filter->X[2] + (cosf(theta) * ax - sinf(theta) * ay) * dt;
    filter->X_pred[3] = filter->X[3] + (sinf(theta) * ax + cosf(theta) * ay) * dt;
    filter->X_pred[4] = filter->X[4] + az * dt;
}


// Jacobiano F (solo valido para el este caso concreto)
void ekf_compute_F(struct extended_kalman_filter *filter, float *U, float dt) {
    float ax = U[0], ay = U[1];
    float theta = filter->X[4];

    filter->F[0][2] = dt;
    filter->F[1][3] = dt;
    filter->F[2][4] = -(sinf(theta) * ax + cosf(theta) * ay) * dt;
    filter->F[3][4] = (cosf(theta) * ax - sinf(theta) * ay) * dt;
}





/** Prediction step
 *
 * X = Ad * X + Bd * U
 * P = Ad * P * Ad' + Q
 *
 * @param filter pointer to the filter structure
 * @param U command vector
 */
void extended_kalman_filter_predict(struct extended_kalman_filter *filter, float *U, float dt) {
    
    float tmp[filter->n][filter->n];
    MAKE_MATRIX_PTR(_tmp, tmp, filter->n);
    MAKE_MATRIX_PTR(_P, filter->P, filter->n);
    MAKE_MATRIX_PTR(_F, filter->F, filter->n);
    MAKE_MATRIX_PTR(_Q, filter->Q, filter->n);

    // Predicción del estado no lineal
    ekf_f(filter, U, dt);
    float_vect_copy(filter->X, filter->X_pred, filter->n);

    // // Calcular Jacobiano F
    ekf_compute_F(filter, U, dt);

    // // P = F * P * F' + Q
    float_mat_mul(_tmp, _F, _P, filter->n, filter->n, filter->n);
    float_mat_mul_transpose(_P, _tmp, _F, filter->n, filter->n, filter->n);
    float_mat_sum(_P, _P, _Q, filter->n, filter->n);
}



/** Update step
 *
 * S = Cd * P * Cd' + R
 * K = P * Cd' / S
 * X = X + K * (Y - Cd * X)
 * P = P - K * Cd * P
 *
 * @param filter pointer to the filter structure
 * @param Y measurement vector
 */
void extended_kalman_filter_update(struct extended_kalman_filter *filter, float *Y) {

    float S[filter->m][filter->m];
    float K[filter->n][filter->m];
    float tmp1[filter->n][filter->m];
    float tmp2[filter->n][filter->n];

    MAKE_MATRIX_PTR(_P, filter->P, filter->n);
    MAKE_MATRIX_PTR(_H, filter->H, filter->m);
    MAKE_MATRIX_PTR(_R, filter->R, filter->m);
    MAKE_MATRIX_PTR(_S, S, filter->m);
    MAKE_MATRIX_PTR(_K, K, filter->n);
    MAKE_MATRIX_PTR(_tmp1, tmp1, filter->n);
    MAKE_MATRIX_PTR(_tmp2, tmp2, filter->n);

    // S = H * P * H' + R
    float_mat_mul_transpose(_tmp1, _P, _H, filter->n, filter->n, filter->m); // P * C'
    float_mat_mul(_S, _H, _tmp1, filter->m, filter->n, filter->m); // C *
    float_mat_sum(_S, _S, _R, filter->m, filter->m); // + R

    // K = P * H' * inv(S)
    float_mat_invert(_S, _S, filter->m); // inv(S) in place
    float_mat_mul(_K, _tmp1, _S, filter->n, filter->m, filter->m); // tmp1 {P*C'} * inv(S)

    // P = P - K * H * P
    float_mat_mul(_tmp2, _K, _H, filter->n, filter->m, filter->n); // K * H
    float_mat_mul_copy(_tmp2, _tmp2, _P, filter->n, filter->n, filter->n); // * P
    float_mat_diff(_P, _P, _tmp2, filter->n, filter->n); // P - K*H*P

    // X = X + K * err
    float err[filter->n];
    float dx_err[filter->n];

    // Modelo de observación h(X)
    float_mat_vect_mul(err, _H, filter->X, filter->m, filter->n);

    // Error de medición: err = Y - h(X)
    float_vect_diff(err, Y, err, filter->m);

    float_mat_vect_mul(dx_err, _K, err, filter->n, filter->m); // K * err
    float_vect_sum(filter->X, filter->X, dx_err, filter->n); // X + dx_err

}


