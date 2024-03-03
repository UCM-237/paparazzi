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

/** \file gvf_square.c
 *
 *  Guidance algorithm based on vector fields
 *  2D square trajectory
 */


#include "modules/nav/common_nav.h"
#include "gvf_square.h"
#include "generated/airframe.h"

/*! Default gain ke for the square trajectory */
#ifndef GVF_square_KE
#define GVF_square_KE 0.7
#endif

/*! Default gain kn for the square trajectory */
#ifndef GVF_square_KN
#define GVF_square_KN 2
#endif

/*! Default first axis for the square trajectory */
#ifndef GVF_square_R
#define GVF_square_R 6
#endif

gvf_squr_par gvf_square_par = {GVF_square_KE, GVF_square_KN,
                               GVF_square_R};

void gvf_square_info(float *phi, struct gvf_grad *grad,
                      struct gvf_Hess *hess)
{

  struct EnuCoor_f *p = stateGetPositionEnu_f();
  float px = p->x;
  float py = p->y;
  float wx = gvf_trajectory.p[0];
  float wy = gvf_trajectory.p[1];

  // Phi(x,y)
  float maxi = (fabs(px-wx) > fabs(py-wy)) ? fabs(px-wx) : fabs(py-wy);
  *phi = maxi - gvf_square_par.r;

  // grad Phi
  float g1, g2;
  g1 = (px - wx >= 0) ? 1 : -1;
  g2 = (py - wy >= 0) ? 1 : -1;
  if(abs(px-wx) > abs(py-wy)){
    grad->nx = g1;
    grad->ny = 0;
  }
  else if(abs(px-wx) < abs(py-wy)){
    grad->nx = 0;
    grad->ny = g2;
  }
  else{
    grad->nx = g1;
    grad->ny = g2;
  }

  // Hessian Phi
  hess->H11 = 0;
  hess->H12 = 0;
  hess->H21 = 0;
  hess->H22 = 0;
}
