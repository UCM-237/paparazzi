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
 * @file modules/guidance/gvf_parametric_bare/gvf_parametric_bare.cpp
 *
 * Guiding vector field algorithm for 2D and 3D complex trajectories.
 */

#include <math.h>
#include <stdbool.h>
#include "std.h"

#include "gvf_parametric_bare.h"
#include "./trajectories/gvf_parametric_bare_2d_bezier_splines.h"

#include "autopilot.h"


// Used for Bézier curves splines telemetru
uint32_t gvf_parametric_bare_splines_ctr = 0;

// GVF control parameters
gvf_parametric_bare_con gvf_parametric_bare_control;

// Trajectory parameters
gvf_parametric_bare_tra gvf_parametric_bare_trajectory;

// Parameters array length
int gvf_parametric_bare_plen = 1;
int gvf_parametric_bare_plen_wps = 0;

// Error signals array length
int gvf_parametric_bare_elen = 3;

// Bézier array segment
bare_bezier_t gvf_bezier_2D_bare[GVF_PARAMETRIC_BARE_2D_BEZIER_N_SEG];

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_gvf_parametric_bare(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t traj_type = (uint8_t)gvf_parametric_bare_trajectory.type;

  uint32_t now = get_sys_time_msec();
  uint32_t delta_T = now - gvf_c_t0;

  float wb = gvf_parametric_bare_control.w * gvf_parametric_bare_control.beta;

  if (delta_T < 200) {
    gvf_parametric_bare_splines_ctr = (gvf_parametric_bare_splines_ctr + 1) % 3;
    pprz_msg_send_GVF_PARAMETRIC(trans, dev, AC_ID, &traj_type, &gvf_parametric_bare_control.s, &wb, gvf_parametric_bare_plen,
                                 gvf_parametric_bare_trajectory.p_parametric, gvf_parametric_bare_elen, gvf_parametric_bare_trajectory.phi_errors);
  }
}

#if GVF_OCAML_GCS
static void send_circle_parametric(struct transport_tx *trans, struct link_device *dev)
{
  uint32_t now = get_sys_time_msec();
  uint32_t delta_T = now - gvf_c_t0;

  /*
  if (delta_T < 200)
    if (gvf_parametric_bare_trajectory.type == ELLIPSE_3D) {
      pprz_msg_send_CIRCLE(trans, dev, AC_ID, &gvf_parametric_bare_trajectory.p_parametric[0],
                           &gvf_parametric_bare_trajectory.p_parametric[1], &gvf_parametric_bare_trajectory.p_parametric[2]);
    }
    */
}
#endif // GVF_OCAML_GCS

#endif // PERIODIC TELEMETRY

void gvf_parametric_bare_init(void)
{
  gvf_parametric_bare_control.w = 0;
  gvf_parametric_bare_control.delta_T = 0;
  gvf_parametric_bare_control.s = 1;
  gvf_parametric_bare_control.k_psi = GVF_PARAMETRIC_BARE_CONTROL_KPSI;
  gvf_parametric_bare_control.L = GVF_PARAMETRIC_BARE_CONTROL_L;
  gvf_parametric_bare_control.beta = GVF_PARAMETRIC_BARE_CONTROL_BETA;
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GVF_PARAMETRIC, send_gvf_parametric_bare);
#if GVF_OCAML_GCS
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_CIRCLE, send_circle_parametric);
#endif // GVF_OCAML_GCS
#endif // PERIODIC_TELEMETRY

}

void gvf_parametric_bare_set_direction(int8_t s)
{
  gvf_parametric_bare_control.s = s;
}

void gvf_parametric_bare_control_2D(float kx, float ky, float f1, float f2, float f1d, float f2d, float f1dd, float f2dd)
{
// GVF Parameters
  float L = gvf_parametric_bare_control.L;
  float beta = gvf_parametric_bare_control.beta * gvf_parametric_bare_control.s;

  // Guiding vector field in 3D
  float X[3];

  // Proyected guiding vector field
  float Xp[2];

  // Normalized proyected guiding vector field
  float Xpn[2];

  // Norm of the guiding vector field
  float chipnorm;

  // Jacobian of guiding vector field
  float J[2][3];

  // Position of vehicle
  struct EnuCoor_f *pos_enu = stateGetPositionEnu_f();
  float x = pos_enu->x;
  float y = pos_enu->y;

  // Ground speed of vehicle
  float ground_speed = stateGetHorizontalSpeedNorm_f();

  // Scaled error to desired trajectory
  float phi1 = L * (x - f1);
  float phi2 = L * (y - f2);

  // Velocity of vehicle
  struct EnuCoor_f *vel_enu = stateGetSpeedEnu_f();

  // Course of vehicle
  float course = stateGetHorizontalSpeedDir_f();

  // Control signal
  float heading_rate;

  // Obtain time now. TODO: Incorporate the changes made in pull-request.
  uint32_t now = get_sys_time_msec();

  // Compute the time elapsed between two gvf calls
  gvf_parametric_bare_control.delta_T = now - gvf_c_t0;
  gvf_c_t0 = now;

  if (gvf_parametric_bare_control.delta_T > 300)
  {
    // Reset w since we assume the algorithm starts
    gvf_parametric_bare_control.w = 0;
    return;
  }

  // Carrot position
  #ifdef FIXEDWING_FIRMWARE
  desired_x = f1;
  desired_y = f2;
  #endif

  // Error signals for the telemetry
  gvf_parametric_bare_trajectory.phi_errors[0] = phi1;
  gvf_parametric_bare_trajectory.phi_errors[1] = phi2;
  gvf_parametric_bare_elen = 2;
  
  X[0] = L * ( L * beta * f1d - kx * phi1);
  X[1] = L * ( L * beta * f2d - ky * phi2);
  X[2] = L * ( L * L + beta * (kx * phi1 * f1d + ky * phi2 * f2d));

  Xp[0] = X[0];
  Xp[1] = X[1];

  chipnorm = sqrtf(Xp[0] * Xp[0] + Xp[1] * Xp[1]);

  Xpn[0] = Xp[0]/chipnorm;
  Xpn[1] = Xp[1]/chipnorm;

  J[0][0] = -kx * L * L;
  J[0][1] = 0;
  J[0][2] = L * (L * L * beta * beta * f1dd + kx * beta * L * f1d);
  J[1][0] = 0;
  J[1][1] = -ky * L * L;
  J[1][2] = L * (L * L * beta * beta * f2dd + kx * beta * L * f2d);
  

  // Compute virtual dimension derivative
  float w_dot = (ground_speed * X[2]) / sqrtf(X[0] * X[0] + X[1] * X[1]);
  
  // Velocities and virtual dimension
  float xi_dot[3] = {vel_enu->x, vel_enu->y, w_dot};

  // Heading direction
  float h[2] = {sinf(course), cosf(course)};

  // V = thetad - kh^TEchip
  // Desired angular speed
  float thetad;

  // Auxiliary variable
  float aux;

  thetad =  xi_dot[0]*(J[0][0]*Xpn[1] - J[1][0]*Xpn[0]);
  thetad += xi_dot[1]*(J[0][1]*Xpn[1] - J[1][1]*Xpn[0]);
  thetad += xi_dot[2]*(J[0][2]*Xpn[1] - J[1][2]*Xpn[0]);
  thetad = -thetad/chipnorm;
  aux = Xpn[0]*h[1] - Xpn[1]*h[0];

  heading_rate = thetad - gvf_parametric_bare_control.k_psi * aux;

  // From gvf_common.h TODO: implement d/dt of kppa and ori_err
  gvf_c_omega.omega   = heading_rate; 
  gvf_c_info.kappa    = (f1d*f2dd - f1dd*f2d)/powf(f1d*f1d + f2d*f2d, 1.5);
  gvf_c_info.ori_err  = 1 - (Xpn[0]*cosf(course) + Xpn[1]*sinf(course));
  
  // Virtual coordinate update, even if the vehicle is not in autonomous mode, the parameter w will get "closer" to
  // the vehicle. So it is not only okei but advisable to update it.
  gvf_parametric_bare_control.w += w_dot * gvf_parametric_bare_control.delta_T * 1e-3;
}

/** 2D Trajectories **/

bool gvf_parametric_bare_2D_bezier_XY(void)
{
  float fx, fy, fxd, fyd, fxdd, fydd;
  gvf_parametric_bare_trajectory.type = BEZIER_2D_BARE;

  // Obtain information from the curve. TODO: Replace magic number
  gvf_parametric_bare_2d_bezier_splines_info(gvf_bezier_2D_bare, 3,
                                             &fx, &fy, &fxd, &fyd, &fxdd, &fydd);

  // Compute control signal
  gvf_parametric_bare_control_2D(gvf_parametric_bare_2d_bezier_par.kx,
                                 gvf_parametric_bare_2d_bezier_par.ky,
                                 fx, fy, fxd, fyd, fxdd, fydd);
  return true;
}

bool gvf_parametric_bare_2D_bezier_wp(uint8_t wp0)
{

  // Obtain Bézier points via Paparazzi GCS
  float x[3*GVF_PARAMETRIC_BARE_2D_BEZIER_N_SEG+1];
  float y[3*GVF_PARAMETRIC_BARE_2D_BEZIER_N_SEG+1];
  for(int k = 0; k < 3 * GVF_PARAMETRIC_BARE_2D_BEZIER_N_SEG + 1; k++){
    x[k] = WaypointX(wp0+k);
    y[k] = WaypointY(wp0+k);
  }

  // Create Bézier splines
  bare_create_bezier_spline(gvf_bezier_2D_bare, 3, x, y);

  /* Send data piecewise. Some radio modules do not allow for a big data frame.*/

  // Send x points -> Indicate x with sign (+) in the first parameter
  if(gvf_parametric_bare_splines_ctr == 0){
    gvf_parametric_bare_trajectory.p_parametric[0] = -GVF_PARAMETRIC_BARE_2D_BEZIER_N_SEG; // send x (negative value)
    for(int k = 0; k < 3*GVF_PARAMETRIC_BARE_2D_BEZIER_N_SEG+1; k++)
      gvf_parametric_bare_trajectory.p_parametric[k+1] = x[k];
  }
  // Send y points -> Indicate y with sign (-) in the first parameter
  else if (gvf_parametric_bare_splines_ctr == 1){
    gvf_parametric_bare_trajectory.p_parametric[0]  = GVF_PARAMETRIC_BARE_2D_BEZIER_N_SEG; // send y (positive value)
    for(int k = 0; k < 3*GVF_PARAMETRIC_BARE_2D_BEZIER_N_SEG+1; k++)
      gvf_parametric_bare_trajectory.p_parametric[k+1] = y[k];
  }
  // send kx, ky, beta and anything else needed..
  else{
    gvf_parametric_bare_trajectory.p_parametric[0] = 0.0;
    gvf_parametric_bare_trajectory.p_parametric[1] = gvf_parametric_bare_2d_bezier_par.kx;
    gvf_parametric_bare_trajectory.p_parametric[2] = gvf_parametric_bare_2d_bezier_par.ky;
    gvf_parametric_bare_trajectory.p_parametric[3] = gvf_parametric_bare_control.beta;
  }
  gvf_parametric_bare_plen = 16;
  gvf_parametric_bare_plen_wps = 1;

  // restart the spline
  if(gvf_parametric_bare_control.w >= (float)GVF_PARAMETRIC_BARE_2D_BEZIER_N_SEG)
    gvf_parametric_bare_control.w = 0;
  else if(gvf_parametric_bare_control.w < 0)
    gvf_parametric_bare_control.w = 0;
  gvf_parametric_bare_2D_bezier_XY();
  return true;
}

bool gvf_parametric_bare_2D_quintic_bezier_XY(void)
{
  float fx, fy, fxd, fyd, fxdd, fydd;
  gvf_parametric_bare_trajectory.type = QUINTIC_BEZIER_2D_BARE;

  // Obtain information from the curve. TODO: Replace magic number
  gvf_parametric_bare_2d_bezier_splines_info(gvf_bezier_2D_bare, 5,
                                             &fx, &fy, &fxd, &fyd, &fxdd, &fydd);

  // Compute control signal
  gvf_parametric_bare_control_2D(gvf_parametric_bare_2d_bezier_par.kx, gvf_parametric_bare_2d_bezier_par.ky,
                                 fx, fy, fxd, fyd, fxdd, fydd);
  return true;
}

bool gvf_parametric_bare_2D_quintic_bezier_wp(uint8_t wp0)
{

  // Array contaning the x and y components of the Bézier curve points
  float x[3*(GVF_PARAMETRIC_BARE_2D_BEZIER_N_SEG+1)];
  float y[3*(GVF_PARAMETRIC_BARE_2D_BEZIER_N_SEG+1)];

  for(int k = 0; k < 3 * (GVF_PARAMETRIC_BARE_2D_BEZIER_N_SEG + 1); k++)
  {
    x[k] = WaypointX(wp0+k);
    y[k] = WaypointY(wp0+k);
  }

  // Create Bézier Spline
  bare_create_bezier_spline(gvf_bezier_2D_bare, 5, x, y);

  /* Send data piecewise. Some radio modules do not allow for a big data frame.*/
  // MAX 4 SEGMENTS FOR XBEE

  // Send x points -> Indicate x with sign (+) in the first parameter
  if(gvf_parametric_bare_splines_ctr == 0)
  {
    gvf_parametric_bare_trajectory.p_parametric[0] = -GVF_PARAMETRIC_BARE_2D_BEZIER_N_SEG; // send x (negative value)
    for(int k = 0; k < 3 *(GVF_PARAMETRIC_BARE_2D_BEZIER_N_SEG + 1); k++)
    {
      gvf_parametric_bare_trajectory.p_parametric[k+1] = x[k];
    }
  }
  // Send y points -> Indicate y with sign (-) in the first parameter
  else if (gvf_parametric_bare_splines_ctr == 1)
  {
    gvf_parametric_bare_trajectory.p_parametric[0]  = GVF_PARAMETRIC_BARE_2D_BEZIER_N_SEG; // send y (positive value)
    for(int k = 0; k < 3* (GVF_PARAMETRIC_BARE_2D_BEZIER_N_SEG + 1); k++)
    {
      gvf_parametric_bare_trajectory.p_parametric[k+1] = y[k];
    }
  }
  // send kx, ky, beta and anything else needed
  else
  {
    gvf_parametric_bare_trajectory.p_parametric[0] = 0.0;
    gvf_parametric_bare_trajectory.p_parametric[1] = gvf_parametric_bare_2d_bezier_par.kx;
    gvf_parametric_bare_trajectory.p_parametric[2] = gvf_parametric_bare_2d_bezier_par.ky;
    gvf_parametric_bare_trajectory.p_parametric[3] = gvf_parametric_bare_control.beta;
  }
  gvf_parametric_bare_plen = 16;
  gvf_parametric_bare_plen_wps = 1;

  // Restart the parameter of the Spline if we have reach the end of it.
  // TODO: Improve it
  if(gvf_parametric_bare_control.w >= (float)GVF_PARAMETRIC_BARE_2D_BEZIER_N_SEG)
  {
    gvf_parametric_bare_control.w = 0;
  }
  else if(gvf_parametric_bare_control.w < 0)
  {
    gvf_parametric_bare_control.w = 0;
  }
  gvf_parametric_bare_2D_quintic_bezier_XY();
  return true;
}

