/*
 * Copyright (C) 2023 Alfredo González Calvin <alfredgo@ucm.es>
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

/**
 * @file modules/guidance/gvf_parametric_bare/gvf_parametric_bare.h
 *
 * Guiding vector field algorithm for 2D and 3D parametric trajectories.
 */

#ifndef GVF_PARAMETRIC_BARE_H
#define GVF_PARAMETRIC_BARE_H

#define GVF_PARAMETRIC_BARE_GRAVITY 9.806

/*! Default scale for the error signals */
#ifndef GVF_PARAMETRIC_BARE_CONTROL_L
#define GVF_PARAMETRIC_BARE_CONTROL_L 0.1
#endif

/*! Default scale for w  */
#ifndef GVF_PARAMETRIC_BARE_CONTROL_BETA
#define GVF_PARAMETRIC_BARE_CONTROL_BETA 0.01
#endif

/*! Default gain kpsi for tuning the alignment of the vehicle with the vector field */
#ifndef GVF_PARAMETRIC_BARE_CONTROL_KPSI
#define GVF_PARAMETRIC_BARE_CONTROL_KPSI 1
#endif

/*! Default GCS trajectory painter */
#ifndef GVF_OCAML_GCS
#define GVF_OCAML_GCS true
#endif

#include "modules/guidance/gvf_parametric_bare/trajectories/gvf_parametric_bare_2d_bezier_splines.h"
#include "std.h"

/** @typedef gvf_parametric_bare_con
* @brief Control parameters for the GVF_PARAMETRIC.
* @field w Virtual coordinate from the parametrization of the trajectory.
* @field delta_T Time between iterations needed for integrating w.
* @field s Defines the direction to be tracked. It takes the values -1 or 1.
* @field k_psi Gain for tuning the control signal gain.
* @field beta Gain for tuning the tangential component of the guiding vector field.
*/
typedef struct {
  float w;
  float delta_T;
  int8_t s;
  float k_psi;
  float L;
  float beta;
} gvf_parametric_bare_con;

// Struct containing all control parameters
extern gvf_parametric_bare_con gvf_parametric_bare_control;

// Enum relating the trajectories with their number
enum trajectories_parametric_bare {
  BEZIER_2D_BARE = 3,
  QUINTIC_BEZIER_2D_BARE = 4,
  NONE_PARAMETRIC_BARE = 255,
};

/** @typedef struct gvf_parametric_bare_tra
 * @brief Struct containing the parameters of the trajectory
 * @field type Enum indicating the trajectory type
 * @field p_parametric Array for the trajectory parameters
 * @field phi_errors Array containing the errors to the desired trajectory
 */
typedef struct {
  enum trajectories_parametric_bare type;
  float p_parametric[16];
  float phi_errors[3];
} gvf_parametric_bare_tra;

// Struct containing the trajectory parameters
extern gvf_parametric_bare_tra gvf_parametric_bare_trajectory;

// Struct of type bezier holding the Bézier points
extern bare_bezier_t gvf_bezier_2D_bare[GVF_PARAMETRIC_BARE_2D_BEZIER_N_SEG];

/** @function void gvf_parametric_bare_init
 *  @brief Function initializing parameters for gvf parametric bare
 *  @param None
 *  @return None
 */
extern void gvf_parametric_bare_init(void);

/** @function void gvf_parametric_bare_set_direction
 *  @brief Function to change the guiding vector field direction
 *  @param s: Parameter indicating the direction of the guiding vector field (+-1)
 *  @returns None
 */
extern void gvf_parametric_bare_set_direction(int8_t s);

/** @function void gvf_parametric_bare_control_2D
 *  @brief Function used to compute the guiding vector field and control signal
 *  @params:
 *  kx,ky Gains used for tuning the vertical component of the guiding vector field
 *  f1,f2 Components of the 2D trajectory evaluated at the virtual coordinate
 *  f1d, f2d Components of the derivative of the 2D trajectory evaluated at the virtual coordinate
 *  f1dd, f2dd Components of the second derivative of the 2D trajectory evaluated at the virtual coordinate
 *  @returns None
 */
extern void gvf_parametric_bare_control_2D(float kx, float ky,
                                           float f1, float f2,
                                           float f1d, float f2d,
                                           float f1dd, float f2dd);

/************ 2D THIRD ORDER BEZIER WITH C^2 CONTINUITY AT CONTROL POINTS ******/
/** @function bool gvf_parametric_bare_2D_bezier_wp
 *  @brief Function used to construct the third order Bézier curves and
 *  preparing the buffers to send them through telemetry.
 *  @param wp0: First point defined in the flight plan of the Bézier Curve
 *  @returns true if successful, false otherwise
 */
extern bool gvf_parametric_bare_2D_bezier_wp(uint8_t wp0);

/** @function bool gvf_parametric_bare_2D_bezier_XY
 *  @brief Function used to obtain the curve and its derivatives evaluated, and
 *  computes the control signal for the third order Bézier curves.
 *  @param None
 *  @returns true if successful, false otherwise
 */
extern bool gvf_parametric_bare_2D_bezier_XY(void);

/************ 2D FIFTH ORDER BEZIER WITH C^2 CONTINUITY AT CONTROL POINTS ******/

/** @function bool gvf_parametric_bare_2D_quintic_bezier_wp
 *  @brief Function used to construct the fifth order Bézier curves and
 *  preparing the buffers to send them through telemetry.
 *  @param wp0: First point defined in the flight plan of the Bézier Curve
 *  @returns true if successful, false otherwise
 */
extern bool gvf_parametric_bare_2D_quintic_bezier_wp(uint8_t wp0);

/** @function bool gvf_parametric_bare_2D_quintic_bezier_XY
 *  @brief Function used to obtain the curve and its derivatives evaluated, and
 *  computes the control signal for the fifth order Bézier curves with C^2
 *  continuity
 *  @param None
 *  @returns true if successful, false otherwise
 */
extern bool gvf_parametric_bare_2D_quintic_bezier_XY(void);

#endif // GVF_PARAMETRIC_H
