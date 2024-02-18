/*
 * Copyright (C) 2016  Hector Garcia de Marina
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
 *  2D romboid trajectory
 */

#ifndef GVF_ROMBOID_H
#define GVF_ROMBOID_H

#include "modules/guidance/gvf/gvf.h"

/** @typedef gvf_ell_par
* @brief Parameters for the GVF line trajectory
* @param ke Gain defining how agressive is the vector field
* @param kn Gain for making converge the vehicle to the vector field
* @param a First axis of the romboid in meters
* @param b Second axis of the romboid in meters
* @param alpha Orientation of the romboid in rads
*/
typedef struct {
  float ke;
  float kn;
  float r;
} gvf_romb_par;

extern gvf_romb_par gvf_romboid_par;

extern void gvf_romboid_info(float *phi, struct gvf_grad *, struct gvf_Hess *);

#endif // GVF_romboid_H
