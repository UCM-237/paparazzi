/*
 * Copyright (C) 2023 Alfredo Gonzalez Calvin <alfredgo@ucm.es>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
 
#ifndef GVF_PARAMETRIC_BARE_2D_BEZIER_SPLINES_H
#define GVF_PARAMETRIC_BARE_2D_BEZIER_SPLINES_H



#ifndef GVF_PARAMETRIC_BARE_2D_BEZIER_N_SEG
#define GVF_PARAMETRIC_BARE_2D_BEZIER_N_SEG 1
#endif


typedef struct {
	float kx;
	float ky;
}gvf_bare_par_2d_bezier_par;


// Cubic or quintic bezier
typedef struct{
	float p0[2];
	float p1[2];
	float p2[2];
	float p3[2];
	// For quintic Bézier
	float p4[2];
	float p5[2];
}bare_bezier_t;




extern gvf_bare_par_2d_bezier_par gvf_parametric_bare_2d_bezier_par;

extern void bare_create_bezier_spline(bare_bezier_t *bezier, int order, float *px, float *py);
extern void gvf_parametric_bare_2d_bezier_splines_info(bare_bezier_t *bezier, int order, float *f1, float *f2, float *f1d, float *f2d, float *f1dd, float *f2dd);
extern float berstein_poly(float t, int k, int p);
extern float binom(float mu, float n);

#endif // bezier splines