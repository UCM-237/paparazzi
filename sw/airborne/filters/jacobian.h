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
 * @file "filters/jacobian.h"
 *
 * Here you can define the functions used in the 
 * extended kalman filter for the jacobian
 */

#ifndef JACOBIAN_H
#define JACOBIAN_H

struct extended_kalman_filter;


/*******************************************************************************
 *                                                                             *
 *  EKF for IMU+GPS fusion                                                     *
 *                                                                             *
 ******************************************************************************/

extern void ekf_f(struct extended_kalman_filter *filter, float *U, float dt);
extern void ekf_compute_F(struct extended_kalman_filter *filter, float *U, float dt);



/*******************************************************************************
 *                                                                             *
 *  EKF for Lidar+GPS fusion                                                   *
 *                                                                             *
 ******************************************************************************/

extern void ekf_slam_f(struct extended_kalman_filter *filter, float *U, float dt);
extern void ekf_slam_compute_F(struct extended_kalman_filter *filter, float *U, float dt);



#endif