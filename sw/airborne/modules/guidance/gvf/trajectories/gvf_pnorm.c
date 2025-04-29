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

/** \file gvf_pnorm.c
 *
 *  Guidance algorithm based on vector fields
 *  2D pnorm trajectory, for p \in [1,\infty)
 */


#include "modules/nav/common_nav.h"
#include "gvf_pnorm.h"
#include "generated/airframe.h"

/*! Default gain ke for the pnorm trajectory */
#ifndef GVF_pnorm_KE
#define GVF_pnorm_KE 0.7
#endif

/*! Default gain kn for the pnorm trajectory */
#ifndef GVF_pnorm_KN
#define GVF_pnorm_KN 2
#endif

/*! Default first axis for the pnorm trajectory */
#ifndef GVF_pnorm_R
#define GVF_pnorm_R 6
#endif

/*! Default p norm of pnorm trajectory */
#ifndef GVF_pnorm_P
#define GVF_pnorm_P 2
#endif


gvf_lpnorm_par gvf_pnorm_par = {GVF_pnorm_KE, GVF_pnorm_KN,
                                GVF_pnorm_R, GVF_pnorm_P};

void gvf_pnorm_info(float *phi, struct gvf_grad *grad, struct gvf_Hess *hess)
{
  // Position of the vehicle
  struct EnuCoor_f *p = stateGetPositionEnu_f();
  float px = p->x;
  float py = p->y;

  // Center of the pnorm
  float wx = gvf_trajectory.p[0];
  float wy = gvf_trajectory.p[1];

  // Componentes of the pnorm \phi(\xi) = ||\xi||_p
  float dx = powf(fabs(px-wx),gvf_pnorm_par.p);
  float dy = powf(fabs(py-wy),gvf_pnorm_par.p);
  float dr = powf(dx + dy, 1.0/gvf_pnorm_par.p);

  // Sign function
  //float sign1 = (px-wx >= 0) ? 1 : -1;
  //float sign2 = (py-wy >= 0) ? 1 : -1;
  float g1, g2;

  // \phi: Error to the desired curve
  *phi = powf(dr, gvf_pnorm_par.p) - powf(gvf_pnorm_par.r,gvf_pnorm_par.p);

  // grad \phi: Gradient of the error
  //g1 = powf( fabs(px-wx)/dr, gvf_pnorm_par.p - 1.0) * sign1;
  //g2 = powf( fabs(py-wy)/dr, gvf_pnorm_par.p - 1.0) * sign2;
  grad->nx = g1;
  grad->ny = g2;
  g1 = powf( fabs(px-wx), gvf_pnorm_par.p - 2.0) * (px - wx);
  g2 = powf( fabs(py-wy), gvf_pnorm_par.p - 2.0) * (py - wy);

  // Hessian \phi: TODO: Check Heassian for p >= 2:
  hess->H11 = gvf_pnorm_par.p * (gvf_pnorm_par.p - 1.0) * powf(fabs(px - wx), gvf_pnorm_par.p - 2.0);
  hess->H12 = 0;
  hess->H21 = 0;
  hess->H22 = gvf_pnorm_par.p * (gvf_pnorm_par.p - 1.0) * powf(fabs(py - wy), gvf_pnorm_par.p - 2.0);
}
