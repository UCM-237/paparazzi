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
 */
#ifndef GVF_PARAMETRIC_BARE_2D_LINES_REPARAMETRIZATION_FACTOR
#define GVF_PARAMETRIC_BARE_2D_LINES_REPARAMETRIZATION_FACTOR 1.0
#endif

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

  // If the convolution parameter falls below the valid range,repeat the first segment.
  if(lambda <= 1)
  {
    return (1 - lambda_factor) * points[0] +  lambda_factor * points[1];
  }
  else if(integer_part < GVF_PARAMETRIC_BARE_2D_LINES_N_SEG)
  {
    return (1 - fractional_part) * points[integer_part] + fractional_part * points[integer_part+1];
  }
  else
  {
    // If the convolution parameter is above the valid range,repeat the first segment.
    return (1 - fractional_part) * points[GVF_PARAMETRIC_BARE_2D_LINES_N_SEG - 1] + fractional_part * points[GVF_PARAMETRIC_BARE_2D_LINES_N_SEG];
  }
}

float gvf_parametric_bare_2d_lines_mollifier(float x, float epsilon)
{
  // TODO: Replace magic number
  float integration_constant = 0.44399;
  float y = x / epsilon;

  if(fabsf(y) < 1)
  {
    // Avoid divisions by zero
    if(fabsf(1-powf(y,2)) <= FLT_EPSILON)
    {
      return 0.0;
    }
    return 1 / (integration_constant * epsilon) * expf(-1 / (1-powf(y,2)));
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
  // TODO: Replace magic number
  int n_points_of_integration = 100;

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

  float step_of_integration = (upper_integration_value - lower_integration_value) / n_points_of_integration;

  float convolution_at_lambda = 0;
  float step = 0;

  for(int k_iter = 0; k_iter < n_points_of_integration; k_iter++)
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
