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

/** \file gvf_romboid.c
 *
 *  Guidance algorithm based on vector fields
 *  2D romboid trajectory
 */


#include "modules/nav/common_nav.h"
#include "gvf_rhomboid.h"
#include "generated/airframe.h"

/*! Default gain ke for the romboid trajectory */
#ifndef GVF_romboid_KE
#define GVF_romboid_KE 0.7
#endif

/*! Default gain kn for the romboid trajectory */
#ifndef GVF_romboid_KN
#define GVF_romboid_KN 2
#endif

/*! Default first axis for the romboid trajectory */
#ifndef GVF_romboid_R
#define GVF_romboid_R 6
#endif

gvf_romb_par gvf_rhomboid_par = {GVF_romboid_KE, GVF_romboid_KN, GVF_romboid_R};

void gvf_romboid_info(float *phi, struct gvf_grad *grad, struct gvf_Hess *hess)
{

  struct EnuCoor_f *p = stateGetPositionEnu_f();
  float px = p->x;
  float py = p->y;
  float wx = gvf_trajectory.p[0];
  float wy = gvf_trajectory.p[1];

  // Phi(x,y)
  *phi = fabs(px - wx) + fabs(py - wy) - gvf_rhomboid_par.r;

  // grad Phi
  float g1, g2;
  g1 = (px - wx >= 0) ? 1 : -1;
  g2 = (py - wy >= 0) ? 1 : -1;
  grad->nx = g1;
  grad->ny = g2;

  // Hessian Phi
  hess->H11 = 0;
  hess->H12 = 0;
  hess->H21 = 0;
  hess->H22 = 0;
}
