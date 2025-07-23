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

#include "modules/nav/common_nav.h"
#include "modules/guidance/gvf_parametric_bare/gvf_parametric_bare.h"
#include "modules/guidance/gvf_parametric_bare/trajectories/gvf_parametric_bare_2d_lines.h"

#ifndef GVF_PARAMETRIC_BARE_2D_LINES_KX
#define GVF_PARAMETRIC_BARE_2D_LINES_KX 0.5
#endif 

#ifndef GVF_PARAMETRIC_BARE_2D_LINES_KY
#define GVF_PARAMETRIC_BARE_2D_LINES_KY 0.5
#endif 

#ifndef GVF_PARAMETRIC_BARE_2D_LINES_EPSILON
#define GVF_PARAMETRIC_BARE_2D_LINES_EPSILON 0.5
#endif

/*
 * Reparametrization factor: adjusts the parameter range from [0, N_SEG]
 * to [0, FACTOR * N_SEG] to achieve finer control over the curve.
 *
 * Each segment between two points x, y ∈ \R^2 is linearly interpolated by:
 *
 *     λx + (1 - λ)y,   where λ ∈ [0, 1].
 *
 * The issue arises when ε = 0.5 — half of the final segment is excluded
 * due to the convolution's support. By increasing the parameter range
 * with a factor (e.g., FACTOR = 10), the effective resolution improves.
 * For example, with ε = 0.5 and FACTOR = 10, only 1/20 of the curve is cut.
 *
 * Do not forget to adjust the same FACTOR in the Ground Control Station!
 * TODO: Send this value via parameter so the GCS does not need to be changed
 */
#ifndef GVF_PARAMETRIC_BARE_2D_LINES_REPARAMETRIZATION_FACTOR
#define GVF_PARAMETRIC_BARE_2D_LINES_REPARAMETRIZATION_FACTOR 1.0
#endif

// Integration constant ensures that \int_{\R}\phi = 1, where \phi is the mollifier.
#define INTEGRATION_CONSTANT 0.44399

// Points of integration used to compute the convolution
#define NUM_POINTS_OF_INTEGRATION 100

gvf_bare_par_2d_lines_par gvf_parametric_bare_2d_lines_par = {GVF_PARAMETRIC_BARE_2D_LINES_KX,
                                                              GVF_PARAMETRIC_BARE_2D_LINES_KY,
                                                              GVF_PARAMETRIC_BARE_2D_LINES_EPSILON,
                                                              GVF_PARAMETRIC_BARE_2D_LINES_EPSILON};

// Just in one dimension
float gvf_parametric_bare_2d_lines_function(float *points, float lambda)
{
  float integer_part_float;
  int   integer_part;
  float fractional_part;
  float lambda_factor = lambda / GVF_PARAMETRIC_BARE_2D_LINES_REPARAMETRIZATION_FACTOR;
  fractional_part = modff(lambda_factor, &integer_part_float);
  integer_part = (int)(integer_part_float);

  /*
   * IT IS NECESSARY TO EXTEND THE DOMAIN OF THE FUNCTION.
   *
   * This extension is required to ensure the convolution of the trajectory
   * is properly carried out. Each component of the trajectory
   * is given by a function f : [0, N_SEG] → ℝ. When convolving with a mollifier
   * whose support is [-ε, ε], the convolution requires evaluating f outside its
   * original domain, specifically, over the extended interval [-ε, N_SEG + ε].
   *
   * For this reason, the trajectory must be defined beyond its original bounds.
   * Values of the curve parameter below 0 and above N_SEG are handled by extending
   * the line segment.
   */
  if(lambda <= 1)
  {
    // If the convolution parameter falls below the valid range,repeat the first segment.
    return (1 - fractional_part) * points[0] +  fractional_part * points[1];
  }
  else if(integer_part < GVF_PARAMETRIC_BARE_2D_LINES_N_SEG)
  {
    return (1 - fractional_part) * points[integer_part] + fractional_part * points[integer_part+1];
  }
  else
  {
    //If the convolution parameter is above the valid range,repeat the last segment to infinity
    return ( (1 - (lambda_factor - GVF_PARAMETRIC_BARE_2D_LINES_N_SEG + 1)) * points[GVF_PARAMETRIC_BARE_2D_LINES_N_SEG - 1] +
           (lambda_factor - GVF_PARAMETRIC_BARE_2D_LINES_N_SEG + 1) * points[GVF_PARAMETRIC_BARE_2D_LINES_N_SEG]);
  }
}

float gvf_parametric_bare_2d_lines_mollifier(float x, float epsilon)
{
  float y = x / epsilon;

  if(fabsf(y) < 1)
  {
    // Avoid divisions by zero
    if(fabsf(1-powf(y,2)) <= FLT_EPSILON)
    {
      return 0.0;
    }
    return 1 / (INTEGRATION_CONSTANT * epsilon) * expf(-1 / (1-powf(y,2)));
  }
  return 0.0;
}

float gvf_parametric_bare_2d_lines_mollifier_derivative(float x, float epsilon)
{
  // Avoid division by zero
  if(fabsf(powf(epsilon,2) - powf(x,2)) <= FLT_EPSILON)
  {
    return 0;
  }
  float fun_dot_f = -2 * powf(epsilon, 2) * x / (powf(epsilon,2) - powf(x,2));
  return gvf_parametric_bare_2d_lines_mollifier(x, epsilon) * fun_dot_f;
}

// Convolution in one dimension
float gvf_parametric_bare_2d_lines_simple_convolution(float lambda, float *points,
                                                      int n_segments, float epsilon,
                                                      int order)
{
  /*
   * NOTE: The subtraction of epsilon is due to the definition of the function.
   *
   * Suppose the function is defined as f : [0, n] → ℝ. In the convolution,
   * the integration limits for y should satisfy:
   *
   *     y ∈ [max(-n + x, -ε), min(x, ε)]
   *
   * This ensures the integration remains within the defined domain.
   *
   * Due to the boundary conditions specified in gvf_parametric_bare_2d_lines_function,
   * we can integrate over the support of the mollifier, since for any integration
   * value outside the support of the mollifier the first and last segments are
   * repeated, thus convolving with the first and lasts segments.
   * */
  float lower_integration_value = -epsilon;
  float upper_integration_value = epsilon;

  float step_of_integration = (upper_integration_value - lower_integration_value) / NUM_POINTS_OF_INTEGRATION;

  float convolution_at_lambda = 0;
  float step = 0;

  for(int k_iter = 0; k_iter < NUM_POINTS_OF_INTEGRATION; k_iter++)
  {
    step = k_iter * step_of_integration;

    // TODO: Replace with a more efficient approach so the "if" is not evaluated
    // in each iteration
    if(order == 0)
    {
      convolution_at_lambda +=
              gvf_parametric_bare_2d_lines_mollifier(lower_integration_value + step,
              epsilon) * gvf_parametric_bare_2d_lines_function(points, lambda -
              (lower_integration_value + step));
    }
    else if(order == 1)
    {
      convolution_at_lambda +=
              gvf_parametric_bare_2d_lines_mollifier_derivative(lower_integration_value
              + step, epsilon) * gvf_parametric_bare_2d_lines_function(points,
              lambda - (lower_integration_value + step));
    }
  }

  return convolution_at_lambda * step_of_integration;
}

void gvf_parametric_bare_2d_lines_info(int n_segments, float *x_points, float *y_points,
                                       float *f1, float *f2, float *f1d, float *f2d)
{
  float lambda = gvf_parametric_bare_control.w;
  lambda = (lambda <= GVF_PARAMETRIC_BARE_2D_LINES_N_SEG) ? lambda : 0;
  float epsilon_x = gvf_parametric_bare_2d_lines_par.epsilon_x;
  float epsilon_y = gvf_parametric_bare_2d_lines_par.epsilon_y;

  *f1 = gvf_parametric_bare_2d_lines_simple_convolution(lambda, x_points,
                                                        n_segments, epsilon_x,
                                                        0);

  *f2 = gvf_parametric_bare_2d_lines_simple_convolution(lambda, y_points,
                                                        n_segments, epsilon_y,
                                                        0);

  *f1d = gvf_parametric_bare_2d_lines_simple_convolution(lambda, x_points,
                                                        n_segments, epsilon_x,
                                                        1);

  *f2d = gvf_parametric_bare_2d_lines_simple_convolution(lambda, y_points,
                                                        n_segments, epsilon_y,
                                                        1);
}
