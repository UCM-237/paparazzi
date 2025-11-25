/*
 * Copyright (C) 2025 Alfredo Gonzalez Calvin <alfredgo@ucm.es>
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
 
#ifndef GVF_PARAMETRIC_BARE_2D_LINES_H
#define GVF_PARAMETRIC_BARE_2D_LINES_H

// Define default number of segments of the lines
#ifndef GVF_PARAMETRIC_BARE_2D_LINES_N_SEG
#define GVF_PARAMETRIC_BARE_2D_LINES_N_SEG 2
#endif

/** @typdef struct gvf_bare_par_2d_lines_par
 * @brief struct containing the perpendicular tuning gains for the guiding
 *        vector field as well as the epsilon for convolution
 * @field kx Tuning gain for the x component of the vector field
 * @field ky Tuning gain for the y component of the vector field
 * @field epsilon_x Constant used to control the support of the mollifier in x direction
 * @field epsilon_y Constant used to control the support of the mollifier in y direction
 */
typedef struct {
  float kx;
  float ky;
  float epsilon_x;
  float epsilon_y;
}gvf_bare_par_2d_lines_par;

extern gvf_bare_par_2d_lines_par gvf_parametric_bare_2d_lines_par;

/* @function gvf_parametric_bare_2d_lines_function
 * @brief Computes the desired function to be followed in one dimension
 * @param points IN: Arrays of points in one dimension
 * @param lambda IN: Value in which the function is evaluated
 * Returns: Evaluated function in one dimension
 */
float gvf_parametric_bare_2d_lines_function(float *points, float lambda);

/* @function gvf_parametric_bare_2d_lines_mollifier
 * @brief Computes the mollifier function \phi for smoothing
 * @param x IN: Value in which the mollifier is evaluated
 * @param epsilon IN: Half of the length of the support of the mollifier [-\epsilon, \epsilon]
 * Returns: Evaluated mollifier \frac{1}{\epsilon}\phi(x/\epsilon)
 */
float gvf_parametric_bare_2d_lines_mollifier(float x, float epsilon);

/* @function gvf_parametric_bare_2d_lines_mollifier_derivative
 * @brief Computes the derivative of the mollifier function \phi for smoothing
 * @param x IN: Value in which the mollifier is evaluated
 * @param epsilon IN: Half of the length of the support of the mollifier [-\epsilon, \epsilon]
 * Returns: Evaluated derivative of the mollifier
 */
float gvf_parametric_bare_2d_lines_mollifier_derivative(float x, float epsilon);

/* @function gvf_parametric_bare_2d_lines_simple_convolution
 * @brief Performs simple 1D convolution with mollifier or its derivative
 * @param lambda Parameter value
 * @param points Array of points to convolve
 * @param n_segments Number of segments (point count - 1)
 * @param epsilon Smoothing parameter
 * @param order 0 for function, 1 for derivative
 * Returns: Convolution of the function with the mollifier at lambda
 */
float gvf_parametric_bare_2d_lines_simple_convolution(float lambda, float *points,
                                                      int n_segments, float epsilon,
                                                      int order);

/* @function gvf_parametric_bare_2d_lines_info
 * @brief Computes smoothed function values and derivatives in both x and y directions
 *        Suppose the trajectory is expressed as g : [0,M] \to \R^2, where g =(g_1,g_2),
 *        and phi_i is the mollifier (i \in \{1,2\})
 * @param n_segments Number of segments
 * @param x_points Array of x coordinates
 * @param y_points Array of y coordinates
 * @param f1 Output: Convolution of the mollifier with g_1 (g_1 * phi_1)
 * @param f2 Output: Convolution of the mollifier with g_2 (g_2 * phi_2)
 * @param f2d Output: Derivative of convolution of the mollifier with g_1 (g_1 * phi_1')
 * @param f2d Output: Derivative of convolution of the mollifier with g_2 (g_2 * phi_2')
 * Returns: None
 */
void gvf_parametric_bare_2d_lines_info(int n_segments, float *x_points, float *y_points,
                                       float *f1, float *f2, float *f1d, float *f2d);


/* @function gvf_parametric_bare_2d_lines_restrict_curvature
 * @brief Computes the necessary epsilon to upper bound the curvature deppending
 * on kappa_max
 * @param n_segments Number of segments
 * @param x_points Array of x coordinates
 * @param y_points Array of y coordinates
 * @param kappa_max Maximum allowed curvature
 * Returns: None
 */
void gvf_parametric_bare_2d_lines_restrict_curvature(int n_segments, float *x_points,
                                                     float *y_points, float kappa_max);

#endif // 2D Lines Mollifier
