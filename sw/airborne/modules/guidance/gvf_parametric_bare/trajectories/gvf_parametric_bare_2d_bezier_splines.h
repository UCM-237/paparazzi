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

// Define default number of segments of the spline
#ifndef GVF_PARAMETRIC_BARE_2D_BEZIER_N_SEG
#define GVF_PARAMETRIC_BARE_2D_BEZIER_N_SEG 1
#endif

/** @typdef struct gvf_bare_par_2d_bezier_par
 * @brief struct containing the perpendicular tuning gains for the guiding
 * vector field
 * @field kx Tuning gain for the x component of the vector field
 * @field ky Tuning gain for the y component of the vector field
 */
typedef struct {
  float kx;
  float ky;
}gvf_bare_par_2d_bezier_par;

/** @typdef struct bare_bezier_t
 * @brief Struct containing the control points of a Bézier curve, up to fifth order.
 * @field pi with i \in \{0,\dots,5\} Array containing the x and y coordinates
 * of the i'th control point.
 */
typedef struct{
  float p0[2];
  float p1[2];
  float p2[2];
  float p3[2];
  float p4[2];
  float p5[2];
}bare_bezier_t;

extern gvf_bare_par_2d_bezier_par gvf_parametric_bare_2d_bezier_par;

/** @function void bare_create_bezier_spline
 * @brief function used to create the bézier curve. Saves the points to the param bezier.
 * @params:
 * bezier [IN/OUT]: Pointer to struct containing Bézier points. Control points and
 * waypoints are saved in this struct.
 * order [IN]: Order of the Bézier Curves (up to fifth order)
 * px [IN]: Pointer to x points
 * py [IN]: Pointer to y points
 * @Returns: None
 */
extern void bare_create_bezier_spline(bare_bezier_t *bezier, int order, float *px, float *py);

/** @function void gvf_parametric_bare_2d_bezier_splines_info
 * @brief Function that evaluates the nth order Bézier curve at the virtual coordiante of
 * the guiding vector field.
 * @Params:
 * bezier [IN]: Pointer to Bézier struct holding all Bézier points
 * order [IN]: Order of the Bézier curve (up to fifth order)
 * f1, f2 [IN/OUT]: Pointer to floats to store the x and y information of the evaluated curve
 * f1d, f2d [IN/OUT]: Pointer to floats to store the x and y information of the derivative of the  curve
 * f1dd, f2dd [IN/OUT]: Pointer to floats to store the x and y information of the second derivative of the curve
 * @Returns: None
 */
extern void gvf_parametric_bare_2d_bezier_splines_info(bare_bezier_t *bezier, int order,
                                                       float *f1, float *f2,
                                                       float *f1d, float *f2d,
                                                       float *f1dd, float *f2dd);

/** @function float binom
 * @brief Function used to compute the binomial coefficient.
 * @Returns: Binomial coefficient (n mu) = n! / mu!(n-mu)!
 */
extern float binom(float mu, float n);

/** @function float berstein_poly
 * @brief Function used to compute a berstein polynomial
 * B_{n,k}(t) =  (n k)t^k(1-t)^{n-k}
 * @ Returns Berstein polynomial B_{n,k} at t
 */
extern float berstein_poly(float t, int k, int n);

#endif // bezier splines
