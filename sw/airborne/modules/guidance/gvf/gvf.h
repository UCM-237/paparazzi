/*
 * Copyright (C) 2016  Hector Garcia de Marina
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

/** @file gvf.h
 *
 *  Guidance algorithm based on vector fields
 */

#ifndef GVF_H
#define GVF_H

#define GVF_GRAVITY 9.806

/*! Default GCS trajectory painter */
#ifndef GVF_OCAML_GCS
#define GVF_OCAML_GCS true
#endif

#include "std.h"

/** @typedef gvf_con
* @brief Control parameters for the GVF
* @param ke Gain defining how agressive is the vector field
* @param kn Gain for making converge the vehile to the vector field
* @param error Error signal. It does not have any specific units. It depends on how the trajectory has been implemented. Check the specific wiki entry for each trajectory.
* @param omega Angular velocity of the vehicle
* @param speed Speed of the vehicle. It is only used in rotorcrafts for now.
* @param s Defines the direction to be tracked. Its meaning depends on the trajectory and its implementation. Check the wiki entry of the GVF. It takes the values -1 or 1.
* @param align Align the vehicle with the direction of the vector field. Can only used in rotorcrafts.
*/
typedef struct {
  float ke;
  float kn;
  float error;
  float omega;
  float speed;
  int8_t s;
  int8_t which_line;
  bool align;
} gvf_con;

extern gvf_con gvf_control;

typedef struct {
  float course;
  float px_dot;
  float py_dot;
} gvf_st;

extern gvf_st gvf_state;



enum trajectories {
  LINE = 0,
  ELLIPSE,
  SIN,
  LINE_ARRAY,
  RHOMBOID,
  SQUARE,
  PNORM,
  NONE = 255,
};

typedef struct {
  enum trajectories type;
  float p[16];
} gvf_tra;

/** @typedef gvf_seg
* @brief Struct employed by the LINE trajectory for the special case of trackinga segment, which is described by the coordinates x1, y1, x2, y2
* @param seg Tracking a segment or not
* @param x1 coordinate w.r.t. HOME
* @param y1 coordinate w.r.t. HOME
* @param x2 coordinate w.r.t. HOME
* @param y2 coordinate w.r.t. HOME
*/
typedef struct {
  int seg;
  float x1;
  float y1;
  float x2;
  float y2;
} gvf_seg;

typedef struct {
float bz0x;
float bz0y;
float bz4x;
float bz4y;
float bz7x;
float bz7y;
float bz11x;
float bz11y;
} bz_wp;

extern bz_wp bz_stop_wp;

extern gvf_tra gvf_trajectory;

struct gvf_grad {
  float nx;
  float ny;
  float nz;
};

struct gvf_Hess {
  float H11;
  float H12;
  float H13;
  float H21;
  float H22;
  float H23;
  float H31;
  float H32;
  float H33;
};

extern void gvf_init(void);
void gvf_control_2D(float ke, float kn, float e,
                    struct gvf_grad *, struct gvf_Hess *);
extern void gvf_set_speed(float speed); // Rotorcraft only (for now)
extern void gvf_set_align(bool align); // Rotorcraft only
extern void gvf_set_direction(int8_t s);

// Straight line
extern bool gvf_line_XY_heading(float x, float y, float heading);
extern bool gvf_line_XY1_XY2(float x1, float y1, float x2, float y2);
extern bool gvf_line_wp1_wp2(uint8_t wp1, uint8_t wp2);
extern bool gvf_segment_loop_XY1_XY2(float x1, float y1, float x2, float y2, float d1, float d2);
extern bool gvf_segment_loop_wp1_wp2(uint8_t wp1, uint8_t wp2, float d1, float d2);
extern bool gvf_segment_XY1_XY2(float x1, float y1, float x2, float y2);
extern bool gvf_segment_wp1_wp2(uint8_t wp1, uint8_t wp2);
extern bool gvf_line_wp_heading(uint8_t wp, float heading);

// Array of straight
extern bool gvf_lines_array_wp(uint8_t wp0, uint8_t wp1, uint8_t wp2, uint8_t wp3, uint8_t wp4, uint8_t wp5, uint8_t wp6);
extern bool gvf_lines_array_wp_v2(uint8_t wp0, float d1, float d2);
// Ellipse
extern bool gvf_ellipse_wp(uint8_t wp, float a, float b, float alpha);
extern bool gvf_ellipse_XY(float x, float y, float a, float b, float alpha);

/** @function gvf_rhomboid_wp
 * @brief Function used to update parameters regarding type of trajectory. This
 * function is called through the flight plan
 * @Params:
 * wp [IN]: Center of the rhomboid, obtained through the flight plan.
 * r [IN]: "Radius" of the rhomboid.
 * Returns: True if success/False otherwise
 */
extern bool gvf_rhomboid_wp(uint8_t wp, float r);

/** @function gvf_rhomboid_XY
 * @brief Function used to compute the control signal for the trajectory
 * @Params:
 * x [IN]: x component of the center of the rhomboid
 * y [IN]: y component of the center of the rhomboid
 * r [IN]: "Radius" of the rhomboid
 * Returns: True if success/False otherwise
 * TODO: The contents of this function could be called by gvf_rhomboid_wp. So
 * this function is actually not necessary. Put its code in gvf_rhomboid_wp
 */
extern bool gvf_rhomboid_XY(float x, float y, float r);

/** @function gvf_square_wp
 * @brief Function used to update parameters regarding type of trajectory. This
 * function is called through the flight plan
 * @Params:
 * wp [IN]: Center of the square, obtained through the flight plan.
 * r [IN]: "Radius" of the square.
 * Returns: True if success/False otherwise
 */
extern bool gvf_square_wp(uint8_t wp, float r);

/** @function gvf_square_XY
 * @brief Function used to compute the control signal for the trajectory
 * @Params:
 * x [IN]: x component of the center of the square
 * y [IN]: y component of the center of the square
 * r [IN]: "Radius" of the square
 * Returns: True if success/False otherwise
 * TODO: The contents of this function could be called by gvf_square_wp. So
 * this function is actually not necessary. Put its code in gvf_square_wp
 */
extern bool gvf_square_XY(float x, float y, float r);

/** @function gvf_pnorm_wp
 * @brief Function used to update parameters regarding type of trajectory. This
 * function is called through the flight plan
 * @Params:
 * wp [IN]: Center of the pnorm, obtained through the flight plan.
 * r [IN]: "Radius" of the pnorm.
 * Returns: True if success/False otherwise
 */
extern bool gvf_pnorm_wp(uint8_t wp, float r, float p);

/** @function gvf_pnorm_XY
 * @brief Function used to compute the control signal for the trajectory
 * @Params:
 * x [IN]: x component of the center of the pnorm
 * y [IN]: y component of the center of the pnorm
 * r [IN]: "Radius" of the pnorm
 * Returns: True if success/False otherwise
 * TODO: The contents of this function could be called by gvf_pnorm_wp. So
 * this function is actually not necessary. Put its code in gvf_pnorm_wp
 */
extern bool gvf_pnorm_XY(float x, float y, float r, float p);

// Sinusoidal
extern bool gvf_sin_XY_alpha(float x, float y, float alpha, float w, float off, float A);
extern bool gvf_sin_wp1_wp2(uint8_t wp1, uint8_t wp2, float w, float off, float A);
extern bool gvf_sin_wp_alpha(uint8_t wp, float alpha, float w, float off, float A);

/* TODO: The following functions shouldn't be in gvf.h since they are using Bézier
 * curves, which are specifically defined in gvf_parametric_bare, and only work with
 * that algorithms. They should be in gvf_parametric or in another folder
 * regarding the capability of stopping at waypoints.
 */

/** @function bool dist_bool
 * @brief function used to compute the distance to an stopping waypoint, and
 * returning true if its inside a certain circle, updating the next point to stop.
 * @Params
 * x_ [IN] x position of the vehicle
 * y_ [IN] y position of the vehicle .
 * wp0 [IN] First point of the Bézier curve. This argument is passed
 * through the flight plan
 * @ Returns true if the vehicle is near (inside a
 * circle of certain radius) the desired waypoint to stop. false otherwise
 */
extern bool dist_bool(float x_, float y_, uint8_t wp0);

/** @function float dist_bool
 * @brief function used to compute the distance to an stopping waypoint, and
 * returning the distance.
 * @Params
 * x_ [IN] x position of the vehicle
 * y_ [IN] y position of the vehicle .
 * wp0 [IN] First point of the Bézier curve. This argument is passed
 * through the flight plan
 * @ Returns true if the vehicle is near (inside a
 * circle of certain radius) the desired waypoint to stop. false otherwise
 */
extern float dist(float x_, float y_, uint8_t wp0);

/** @function bool increase_bz_pointer
 * @brief increases the pointer of a certain buffer that points to the next
 * waypoint to stop in a Bézier curve.
 * @Params None
 * @Returns None
 */
extern void increase_bz_pointer(void);

#endif // GVF_H

