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

/** @file gvf_pnorm.h
 *
 *  Guidance algorithm based on vector fields
 *  2D pnorm trajectory
 */

#ifndef GVF_PNORM_H
#define GVF_PNORM_H

#include "modules/guidance/gvf/gvf.h"

/** @typedef gvf_squr_par
* @brief Parameters for the GVF rhomboid trajectory
* @param ke Gain defining how agressive is the vector field
* @param kn Gain for making converge the vehicle to the vector field
* @param r Maximum side length of the pnorm
* @param p The l-p norm
*/
typedef struct {
  float ke;
  float kn;
  float r;
  float p;
} gvf_lpnorm_par;

extern gvf_lpnorm_par gvf_pnorm_par;

extern void gvf_pnorm_info(float *phi, struct gvf_grad *, struct gvf_Hess *);

#endif // GVF_pnorm_H
