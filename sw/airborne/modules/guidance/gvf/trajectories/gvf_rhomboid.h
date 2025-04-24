/*
 * Copyright (C) 2024 Alfredo Gonzalez Calvin alfredgo@ucm.es
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
 *
 */

/** @file gvf_romboid.h
 *
 *  Guidance algorithm based on vector fields
 *  2D rhomboid trajectory
 */

#ifndef GVF_RHOMBOID_H
#define GVF_RHOMBOID_H

#include "modules/guidance/gvf/gvf.h"

/** @typedef gvf_romb_par
* @brief Parameters for the GVF rhomboid trajectory
* @param ke Gain defining how agressive is the vector field
* @param kn Gain for making converge the vehicle to the vector field
* @param r "Radius" of the rhomboid
*/
typedef struct {
  float ke;
  float kn;
  float r;
} gvf_romb_par;

extern gvf_romb_par gvf_rhomboid_par;

/** @function void gvf_romboid_info
 * @brief Function that computes the error to the desired path, its gradient and its
 * Hessian
 * @params:
 * phi [OUT] Error to the desired trajectory
 * grad [OUT] Gradient of the error to the desired trajectory
 * hess [OUT] Hessian of the error to the desired trajectory
 * @Returns None
 */
extern void gvf_romboid_info(float *phi, struct gvf_grad *, struct gvf_Hess *);

#endif // GVF_romboid_H
