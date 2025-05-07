/*
 * Copyright (C) 2016 Hector Garcia de Marina
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

#include <math.h>
#include "std.h"

#include "pprzlink/messages.h"
#include "pprzlink/telemetry/NUM_WP_MOVED.h"

#include "modules/guidance/gvf/gvf.h"
#include "modules/guidance/gvf/gvf_low_level_control.h"
#include "modules/guidance/gvf/trajectories/gvf_ellipse.h"
#include "modules/guidance/gvf/trajectories/gvf_line.h"
#include "modules/guidance/gvf/trajectories/gvf_sin.h"
#include "autopilot.h"
#include "../gvf_common.h"

#include "../../../firmwares/rover/guidance/boat_guidance.h"



uint8_t num_pnts;


// Control
gvf_con gvf_control;

// State
gvf_st gvf_state;

// Trajectory
gvf_tra gvf_trajectory;
gvf_seg gvf_segment;

// Time variables to check if GVF is active
uint32_t gvf_t0 = 0;

// Param array lenght
int gvf_plen = 1;
int gvf_plen_wps = 0;



bz_wp bz_stop_wp;

float dist_WP=0.0;
// Lines
gvf_li_line gvf_lines_array[GVF_N_LINES];


#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_gvf(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t traj_type = (uint8_t)gvf_trajectory.type;

  uint32_t now = get_sys_time_msec();
  uint32_t delta_T = now - gvf_t0;

  if (delta_T < 200) {
    pprz_msg_send_GVF(trans, dev, AC_ID, &gvf_control.error, &traj_type,
                      &gvf_control.s, &gvf_control.ke, gvf_plen, gvf_trajectory.p);

#if GVF_OCAML_GCS
    if (gvf_trajectory.type == ELLIPSE &&
        ((int)gvf_trajectory.p[2] == (int)gvf_trajectory.p[3])) {
      pprz_msg_send_CIRCLE(trans, dev, AC_ID,
                           &gvf_trajectory.p[0], &gvf_trajectory.p[1],
                           &gvf_trajectory.p[2]);
    }

    if (gvf_trajectory.type == LINE && gvf_segment.seg == 1) {
      pprz_msg_send_SEGMENT(trans, dev, AC_ID,
                            &gvf_segment.x1, &gvf_segment.y1,
                            &gvf_segment.x2, &gvf_segment.y2);
    }
    if (gvf_trajectory.type == LINE_ARRAY && gvf_segment.seg == 1) {
      pprz_msg_send_SEGMENT(trans, dev, AC_ID,
                            &gvf_segment.x1, &gvf_segment.y1,
                            &gvf_segment.x2, &gvf_segment.y2);
      // Draw approaching circle
      pprz_msg_send_CIRCLE(trans, dev, AC_ID,
      			    &gvf_segment.x2, &gvf_segment.y2,
      			    &gvf_c_stopwp.distance_stop);
      			    
    }
    
#endif // GVF_OCAML_GCS

  }
}
static void send_static_control(struct transport_tx *trans, struct link_device *dev){
    pprz_msg_send_STATIC_CONTROL(trans,dev,AC_ID,
    &gvf_c_stopwp.stay_still, &dist_WP,&gvf_c_stopwp.next_wp,&gvf_c_stopwp.pxd,&gvf_c_stopwp.pyd,
    &bz_stop_wp.bz0x, &bz_stop_wp.bz0y, &bz_stop_wp.bz4x,&bz_stop_wp.bz4y,
    &bz_stop_wp.bz7x, &bz_stop_wp.bz7y,&bz_stop_wp.bz11x,&bz_stop_wp.bz11y);

}

static void send_num_wp_moved(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_NUM_WP_MOVED(trans, dev, AC_ID,
                             &num_wp_moved);
  num_pnts = num_wp_moved;
}

#endif // PERIODIC_TELEMETRY

static int out_of_segment_area(float x1, float y1, float x2, float y2, float d1, float d2)
{
  struct EnuCoor_f *p = stateGetPositionEnu_f();
  float px = p->x - x1;
  float py = p->y - y1;

  float zx = x2 - x1;
  float zy = y2 - y1;
  float alpha = atan2f(zy, zx);

  float cosa = cosf(-alpha);
  float sina = sinf(-alpha);

  float pxr = px * cosa - py * sina;
  float zxr = zx * cosa - zy * sina;

  int s = 0;

  if (pxr < -d1) {
    s = 1;
  } else if (pxr > (zxr + d2)) {
    s = -1;
  }

  if (zy < 0) {
    s *= -1;
  }

  return s;
}

void gvf_init(void)
{
  gvf_control.ke = 1;
  gvf_control.kn = 1;
  gvf_control.s = 1;

  gvf_control.which_line = 0; // Start always with first line
  gvf_control.speed = 1.0; // Rotorcraft only (for now)
  gvf_control.align = false; // Rotorcraft only
  gvf_trajectory.type = NONE;

  // gvf_common.h
  gvf_c_stopwp.stay_still = 0;
  gvf_c_stopwp.stop_at_wp = 1;
  gvf_c_stopwp.distance_stop = 2;
  gvf_c_stopwp.wait_time = 15;
  
  gvf_c_stopwp.next_wp=0;
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GVF, send_gvf);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STATIC_CONTROL, send_static_control);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_NUM_WP_MOVED, send_num_wp_moved);
#endif
}

// GENERIC TRAJECTORY CONTROLLER
void gvf_control_2D(float ke, float kn __attribute__((unused)), float e,
                    struct gvf_grad *grad, struct gvf_Hess *hess)
{
  gvf_t0 = get_sys_time_msec();

  gvf_low_level_getState();
  float course __attribute__((unused)) = gvf_state.course;
  float px_dot = gvf_state.px_dot;
  float py_dot = gvf_state.py_dot;

  int s = gvf_control.s;

  // gradient Phi
  float nx = grad->nx;
  float ny = grad->ny;

  // tangent to Phi
  float tx = s * grad->ny;
  float ty = -s * grad->nx;

  // Hessian
  float H11 = hess->H11;
  float H12 = hess->H12;
  float H21 = hess->H21;
  float H22 = hess->H22;

  // Calculation of the desired angular velocity in the vector field
  float pdx_dot = tx - ke * e * nx;
  float pdy_dot = ty - ke * e * ny;

  float Apd_dot_dot_x = -ke * (nx * px_dot + ny * py_dot) * nx;
  float Apd_dot_dot_y = -ke * (nx * px_dot + ny * py_dot) * ny;

  float Bpd_dot_dot_x = ((-ke * e * H11) + s * H21) * px_dot
                        + ((-ke * e * H12) + s * H22) * py_dot;
  float Bpd_dot_dot_y = -(s * H11 + (ke * e * H21)) * px_dot
                        - (s * H12 + (ke * e * H22)) * py_dot;

  float pd_dot_dot_x = Apd_dot_dot_x + Bpd_dot_dot_x;
  float pd_dot_dot_y = Apd_dot_dot_y + Bpd_dot_dot_y;

  float norm_pd_dot = sqrtf(pdx_dot * pdx_dot + pdy_dot * pdy_dot);
  float md_x = pdx_dot / norm_pd_dot;
  float md_y = pdy_dot / norm_pd_dot;

  float md_dot_const = -(md_x * pd_dot_dot_y - md_y * pd_dot_dot_x)
                       / norm_pd_dot;

  float md_dot_x __attribute__((unused)) =  md_y * md_dot_const;
  float md_dot_y __attribute__((unused))= -md_x * md_dot_const;

  #if defined(ROTORCRAFT_FIRMWARE)

  // Use accel based control. Not recommended as of current implementation
  #if defined(GVF_ROTORCRAFT_USE_ACCEL)

  // Set nav for command
  // Use parameter kn as the speed command
  nav.speed.x = md_x * kn;
  nav.speed.y = md_y * kn;

  // Acceleration induced by the field with speed set to kn (!WIP!)
  #warning "Using GVF for rotorcraft is still experimental, proceed with caution"
  float n_norm = sqrtf(nx*nx+ny*ny);
  float hess_px_dot = px_dot * H11 + py_dot * H12;
  float hess_py_dot = px_dot * H21 + py_dot * H22;

  float hess_pdx_dot = pdx_dot * H11 + pdy_dot * H12;
  float hess_pdy_dot = pdx_dot * H21 + pdy_dot * H22;

  float curvature_correction = tx * hess_px_dot + ty * hess_py_dot / (n_norm * n_norm);
  float accel_correction_x = kn * hess_py_dot / n_norm;
  float accel_correction_y = - kn * hess_px_dot / n_norm;
  float accel_cmd_x = accel_correction_x + px_dot * curvature_correction;
  float accel_cmd_y = accel_correction_y + py_dot * curvature_correction;

  float speed_cmd_x = kn*tx / n_norm - ke * e * nx / (n_norm);
  float speed_cmd_y = kn*ty / n_norm - ke * e * ny / (n_norm);

  // TODO: don't change nav struct directly
  nav.accel.x = accel_cmd_x + (speed_cmd_x - px_dot);
  nav.accel.y = accel_cmd_y + (speed_cmd_y - py_dot);
  nav.heading = atan2f(md_x,md_y);

  #else // SPEED_BASED_GVF

  nav.setpoint_mode = NAV_SETPOINT_MODE_SPEED;

  // Speed-based control, acceleration based control not implemented yet
  nav.speed.x = gvf_control.speed * md_x;
  nav.speed.y = gvf_control.speed * md_y;

  // Optionally align heading with trajectory
  if (gvf_control.align) 
  {
    nav.heading = atan2f(md_x, md_y);
  }
  
  #endif

  #else // FIXEDWING / ROVER FIRMWARE

  float omega_d = -(md_dot_x * md_y - md_dot_y * md_x);

  float mr_x = sinf(course);
  float mr_y = cosf(course);

  float omega = omega_d + kn * (mr_x * md_y - mr_y * md_x);

  gvf_control.omega = omega;
  
  // From gvf_common.h
  gvf_c_omega.omega  = omega; 
  gvf_c_info.kappa   = (nx*(H12*ny - nx*H22) + ny*(H21*nx - H11*ny))/powf(nx*nx + ny*ny,1.5);
  gvf_c_info.ori_err = 1 - (md_x*cosf(course) + md_y*sinf(course));
  gvf_low_level_control_2D(omega);

  #endif
}

// BEGIN ROTORCRAFT

void gvf_set_speed(float speed)
{
  if (speed < 0.0) speed = 0.0;
  gvf_control.speed = speed;
}

void gvf_set_align(bool align)
{
  gvf_control.align = align;
}

// END ROTORCRAFT

void gvf_set_direction(int8_t s)
{
  gvf_control.s = s;
}

// STRAIGHT LINE

static void gvf_line(float a, float b, float heading)
{
  float e;
  struct gvf_grad grad_line;
  struct gvf_Hess Hess_line;

  gvf_trajectory.type = 0;
  gvf_trajectory.p[0] = a;
  gvf_trajectory.p[1] = b;
  gvf_trajectory.p[2] = heading;
  gvf_plen = 3 + gvf_plen_wps;
  gvf_plen_wps = 0;

  gvf_line_info(&e, &grad_line, &Hess_line);
  gvf_control.ke = gvf_line_par.ke;
  gvf_control_2D(gvf_line_par.ke, gvf_line_par.kn, e, &grad_line, &Hess_line); // Removed 1e-2

  gvf_control.error = e;

  gvf_setNavMode(GVF_MODE_WAYPOINT);

  gvf_segment.seg = 0;
}

bool gvf_line_XY_heading(float a, float b, float heading)
{
  gvf_set_direction(1);
  gvf_line(a, b, heading);
  return true;
}

bool gvf_line_XY1_XY2(float x1, float y1, float x2, float y2)
{
  if (gvf_plen_wps != 2) {
    gvf_trajectory.p[3] = x2;
    gvf_trajectory.p[4] = y2;
    gvf_trajectory.p[5] = 0;
    gvf_plen_wps = 3;
  }

  float zx = x2 - x1;
  float zy = y2 - y1;

  gvf_line_XY_heading(x1, y1, atan2f(zx, zy));

  gvf_setNavMode(GVF_MODE_ROUTE);
  gvf_segment.seg = 1;
  gvf_segment.x1 = x1;
  gvf_segment.y1 = y1;
  gvf_segment.x2 = x2;
  gvf_segment.y2 = y2;

  return true;
}

bool gvf_line_wp1_wp2(uint8_t wp1, uint8_t wp2)
{
  gvf_trajectory.p[3] = wp1;
  gvf_trajectory.p[4] = wp2;
  gvf_plen_wps = 2;

  float x1 = WaypointX(wp1);
  float y1 = WaypointY(wp1);
  float x2 = WaypointX(wp2);
  float y2 = WaypointY(wp2);

  return gvf_line_XY1_XY2(x1, y1, x2, y2);
}

bool gvf_segment_loop_XY1_XY2(float x1, float y1, float x2, float y2, float d1, float d2)
{
  int s = out_of_segment_area(x1, y1, x2, y2, d1, d2);
  if (s != 0) {
    gvf_set_direction(s);
  }

  float zx = x2 - x1;
  float zy = y2 - y1;
  float alpha = atanf(zx / zy);

  gvf_line(x1, y1, alpha);

  gvf_setNavMode(GVF_MODE_ROUTE);

  gvf_segment.seg = 1;
  gvf_segment.x1 = x1;
  gvf_segment.y1 = y1;
  gvf_segment.x2 = x2;
  gvf_segment.y2 = y2;

  return true;
}

bool gvf_segment_loop_wp1_wp2(uint8_t wp1, uint8_t wp2, float d1, float d2)
{
  gvf_trajectory.p[3] = wp1;
  gvf_trajectory.p[4] = wp2;
  gvf_trajectory.p[5] = d1;
  gvf_trajectory.p[6] = d2;
  gvf_plen_wps = 4;

  float x1 = WaypointX(wp1);
  float y1 = WaypointY(wp1);
  float x2 = WaypointX(wp2);
  float y2 = WaypointY(wp2);

  return gvf_segment_loop_XY1_XY2(x1, y1, x2, y2, d1, d2);
}

bool gvf_segment_XY1_XY2(float x1, float y1, float x2, float y2)
{
  struct EnuCoor_f *p = stateGetPositionEnu_f();
  float px = p->x - x1;
  float py = p->y - y1;

  float zx = x2 - x1;
  float zy = y2 - y1;

  float beta = atan2f(zy, zx);
  float cosb = cosf(-beta);
  float sinb = sinf(-beta);
  float zxr = zx * cosb - zy * sinb;
  float pxr = px * cosb - py * sinb;

  if ((zxr > 0 && pxr > zxr) || (zxr < 0 && pxr < zxr)) {
    return false;
  }

  return gvf_line_XY1_XY2(x1, y1, x2, y2);
}

bool gvf_segment_wp1_wp2(uint8_t wp1, uint8_t wp2)
{
  gvf_trajectory.p[3] = wp1;
  gvf_trajectory.p[4] = wp2;
  gvf_plen_wps = 2;

  float x1 = WaypointX(wp1);
  float y1 = WaypointY(wp1);
  float x2 = WaypointX(wp2);
  float y2 = WaypointY(wp2);

  return gvf_segment_XY1_XY2(x1, y1, x2, y2);
}

bool gvf_line_wp_heading(uint8_t wp, float heading)
{
  gvf_trajectory.p[3] = wp;
  gvf_plen_wps = 1;

  heading = RadOfDeg(heading);

  float a = WaypointX(wp);
  float b = WaypointY(wp);

  return gvf_line_XY_heading(a, b, heading);
}
// Array of Lines

bool gvf_lines_array_wp_v2(uint8_t wp0, uint8_t wp1, uint8_t wp2, uint8_t wp3, uint8_t wp4, uint8_t wp5, uint8_t wp6, float d1, float d2)
{

	// Create the points
	gvf_trajectory.type = LINE_ARRAY;
	//num_wp_moved = 16;
	
	if (num_wp_moved != 0 ){
	  num_pnts = num_wp_moved;
	  //printf("num_wp_moved= %d\n", num_pnts);
	}
	else{
	  num_pnts = GVF_N_LINES;
	  //printf("GVF_N_LINES= %d\n", num_pnts);
	}
	float x[GVF_N_LINES+1]; float y[GVF_N_LINES+1]; //CAMBIAR PARA DEFINIRLO BIEN
	x[0] = WaypointX(wp0); y[0] = WaypointY(wp0);
	x[1] = WaypointX(wp1); y[1] = WaypointY(wp1);
	x[2] = WaypointX(wp2); y[2] = WaypointY(wp2);
	x[3] = WaypointX(wp3); y[3] = WaypointY(wp3);
	x[4] = WaypointX(wp4); y[4] = WaypointY(wp4);
	x[5] = WaypointX(wp5); y[5] = WaypointY(wp5);
	x[6] = WaypointX(wp6); y[6] = WaypointY(wp6);
	
	float last_point_x = x[num_pnts-1];
	float last_point_y = y[num_pnts-1];
	//printf("num_pnts = %d", num_pnts);
	for(int k = 0; k < GVF_N_LINES; k++)
	{
		gvf_lines_array[k].p1x = x[k];
		gvf_lines_array[k].p1y = y[k];
		gvf_lines_array[k].p2x = x[k+1];
		gvf_lines_array[k].p2y = y[k+1];
	}
	struct EnuCoor_f *p = stateGetPositionEnu_f();
 	float px = p->x;
	float py = p->y;
	float dist = sqrtf( powf(px-gvf_lines_array[gvf_control.which_line].p2x,2) + powf(py-gvf_lines_array[gvf_control.which_line].p2y,2));
	
	if (gvf_lines_array[gvf_control.which_line].p1x == last_point_x && gvf_lines_array[gvf_control.which_line].p1y == last_point_y){
	  //printf("Static control");
	  guidance_control.cmd.speed = 0;
	  boat_guidance_bearing_static_ctrl();
	  
	}
	
	if((dist <= gvf_c_stopwp.distance_stop)){
		if(!gvf_c_stopwp.stop_at_wp){
			gvf_control.which_line = (gvf_control.which_line + 1) % GVF_N_LINES;
			}		
		if(gvf_c_stopwp.stop_at_wp && !gvf_c_stopwp.stay_still){
			gvf_control.which_line = (gvf_control.which_line + 1) % GVF_N_LINES;
			gvf_c_stopwp.stay_still = 1;
			}
		}
  	float x1 = gvf_lines_array[gvf_control.which_line].p1x;
   	float y1 = gvf_lines_array[gvf_control.which_line].p1y;
   	float x2 = gvf_lines_array[gvf_control.which_line].p2x;
    float y2 = gvf_lines_array[gvf_control.which_line].p2y;
    gvf_trajectory.p[3] = x2;
    gvf_trajectory.p[4] = y2;
    gvf_trajectory.p[5] = 0;
    gvf_plen_wps = 3;
    return gvf_segment_loop_XY1_XY2(x1, y1, x2, y2, d1, d2);
    
}



bool gvf_lines_array_wp_v3(uint8_t wp0, float d1, float d2)
{

	// Create the points
	gvf_trajectory.type = LINE_ARRAY;
	
	if (num_wp_moved != 0 ){
	  num_pnts = num_wp_moved;
	  //printf("num_wp_moved= %d\n", num_pnts);
	}
	else{
	  num_pnts = GVF_N_LINES;
	  //printf("GVF_N_LINES= %d\n", num_pnts);
	}

	float x[GVF_N_LINES];
	float y[GVF_N_LINES];
	
	for(int k = 0; k < num_pnts; k++){
	  x[k] = WaypointX(wp0+k);
	  y[k] = WaypointY(wp0+k);
	}
	//printf("num_pnts = %d", num_pnts);
	for(int k = 0; k < num_pnts-1; k++)
	{
		gvf_lines_array[k].p1x = x[k];
		gvf_lines_array[k].p1y = y[k];
		gvf_lines_array[k].p2x = x[k+1];
		gvf_lines_array[k].p2y = y[k+1];
	}

	struct EnuCoor_f *p = stateGetPositionEnu_f();
 	float px = p->x;
	float py = p->y;
	float dist = sqrtf( powf(px-gvf_lines_array[gvf_control.which_line].p2x,2) + powf(py-gvf_lines_array[gvf_control.which_line].p2y,2));
	
	if((dist <= gvf_c_stopwp.distance_stop)){
		if(!gvf_c_stopwp.stop_at_wp){
			gvf_control.which_line = (gvf_control.which_line + 1) % num_pnts;
			}		
		if(gvf_c_stopwp.stop_at_wp && !gvf_c_stopwp.stay_still){
			gvf_control.which_line = (gvf_control.which_line + 1) % num_pnts;
			gvf_c_stopwp.stay_still = 1;
			}
		}
  	float x1 = gvf_lines_array[gvf_control.which_line].p1x;
   	float y1 = gvf_lines_array[gvf_control.which_line].p1y;
   	float x2 = gvf_lines_array[gvf_control.which_line].p2x;
    float y2 = gvf_lines_array[gvf_control.which_line].p2y;
    gvf_trajectory.p[3] = x2;
    gvf_trajectory.p[4] = y2;
    gvf_trajectory.p[5] = 0;
    gvf_plen_wps = 3;
    return gvf_segment_loop_XY1_XY2(x1, y1, x2, y2, d1, d2);
    
}

// ELLIPSE

bool gvf_ellipse_XY(float x, float y, float a, float b, float alpha)
{
  float e;
  struct gvf_grad grad_ellipse;
  struct gvf_Hess Hess_ellipse;

  gvf_trajectory.type = 1;
  gvf_trajectory.p[0] = x;
  gvf_trajectory.p[1] = y;
  gvf_trajectory.p[2] = a;
  gvf_trajectory.p[3] = b;
  gvf_trajectory.p[4] = alpha;
  gvf_plen = 5 + gvf_plen_wps;
  gvf_plen_wps = 0;

  // SAFE MODE
  if (a < 1 || b < 1) {
    gvf_trajectory.p[2] = 60;
    gvf_trajectory.p[3] = 60;
  }

  if ((int)gvf_trajectory.p[2] == (int)gvf_trajectory.p[3]) {
    gvf_setNavMode(GVF_MODE_CIRCLE);

  } else {
    gvf_setNavMode(GVF_MODE_WAYPOINT);
  }

  gvf_ellipse_info(&e, &grad_ellipse, &Hess_ellipse);
  gvf_control.ke = gvf_ellipse_par.ke;
  gvf_control_2D(gvf_ellipse_par.ke, gvf_ellipse_par.kn,
                 e, &grad_ellipse, &Hess_ellipse);

  gvf_control.error = e;

  return true;
}


bool gvf_ellipse_wp(uint8_t wp, float a, float b, float alpha)
{
  gvf_trajectory.p[5] = wp;
  gvf_plen_wps = 1;

  gvf_ellipse_XY(WaypointX(wp),  WaypointY(wp), a, b, alpha);
  return true;
}

// SINUSOIDAL (if w = 0 and off = 0, then we just have the straight line case)

bool gvf_sin_XY_alpha(float a, float b, float alpha, float w, float off, float A)
{
  float e;
  struct gvf_grad grad_line;
  struct gvf_Hess Hess_line;

  gvf_trajectory.type = 2;
  gvf_trajectory.p[0] = a;
  gvf_trajectory.p[1] = b;
  gvf_trajectory.p[2] = alpha;
  gvf_trajectory.p[3] = w;
  gvf_trajectory.p[4] = off;
  gvf_trajectory.p[5] = A;
  gvf_plen = 6 + gvf_plen_wps;
  gvf_plen_wps = 0;

  gvf_sin_info(&e, &grad_line, &Hess_line);
  gvf_control.ke = gvf_sin_par.ke;
  gvf_control_2D(1e-2 * gvf_sin_par.ke, gvf_sin_par.kn, e, &grad_line, &Hess_line);

  gvf_control.error = e;

  return true;
}

bool gvf_sin_wp1_wp2(uint8_t wp1, uint8_t wp2, float w, float off, float A)
{
  w = 2 * M_PI * w;

  gvf_trajectory.p[6] = wp1;
  gvf_trajectory.p[7] = wp2;
  gvf_plen_wps = 2;

  float x1 = WaypointX(wp1);
  float y1 = WaypointY(wp1);
  float x2 = WaypointX(wp2);
  float y2 = WaypointY(wp2);

  float zx = x1 - x2;
  float zy = y1 - y2;

  float alpha = atanf(zy / zx);

  gvf_sin_XY_alpha(x1, y1, alpha, w, off, A);

  return true;
}

bool gvf_sin_wp_alpha(uint8_t wp, float alpha, float w, float off, float A)
{
  w = 2 * M_PI * w;
  alpha = RadOfDeg(alpha);

  gvf_trajectory.p[6] = wp;
  gvf_plen_wps = 1;

  float x = WaypointX(wp);
  float y = WaypointY(wp);

  gvf_sin_XY_alpha(x, y, alpha, w, off, A);

  return true;
}

bool dist_bool(float x_, float y_, uint8_t wp0){

	float x[3*(GVF_PARAMETRIC_BARE_2D_BEZIER_N_SEG+1)];
	float y[3*(GVF_PARAMETRIC_BARE_2D_BEZIER_N_SEG+1)];
	for(int k = 0; k < 3 * (GVF_PARAMETRIC_BARE_2D_BEZIER_N_SEG + 1); k++){
	  x[k] = WaypointX(wp0+k);
	  y[k] = WaypointY(wp0+k);
	}
	
	float x_bz[GVF_PARAMETRIC_BARE_2D_BEZIER_N_SEG+1];
	float y_bz[GVF_PARAMETRIC_BARE_2D_BEZIER_N_SEG+1];
	
	x_bz[0]=x[0]; y_bz[0]=y[0];
	x_bz[1]=x[4]; y_bz[1]=y[4];
	x_bz[2]=x[7]; y_bz[2]=y[7];
	x_bz[3]=x[11]; y_bz[3]=y[11];
	
  
  float px = x_;
  float py = y_;
  float dist = sqrtf( powf(px-x_bz[gvf_c_stopwp.next_wp],2) + powf(py-y_bz[gvf_c_stopwp.next_wp],2));
  if(dist <= gvf_c_stopwp.distance_stop){	
  	if(gvf_c_stopwp.stop_at_wp && !gvf_c_stopwp.stay_still){
  		gvf_c_stopwp.stay_still = 1;
  		gvf_c_stopwp.pxd = x_bz[gvf_c_stopwp.next_wp]; 
  		gvf_c_stopwp.pyd = y_bz[gvf_c_stopwp.next_wp];
  		return true;
  	}
  	
  } 
  return false;
}

float dist(float x_, float y_, uint8_t wp0){
	float x[3*(GVF_PARAMETRIC_BARE_2D_BEZIER_N_SEG+1)];
	float y[3*(GVF_PARAMETRIC_BARE_2D_BEZIER_N_SEG+1)];
	for(int k = 0; k < 3 * (GVF_PARAMETRIC_BARE_2D_BEZIER_N_SEG + 1); k++){
	  x[k] = WaypointX(wp0+k);
	  y[k] = WaypointY(wp0+k);
	}
	
	float x_bz[GVF_PARAMETRIC_BARE_2D_BEZIER_N_SEG+1];
	float y_bz[GVF_PARAMETRIC_BARE_2D_BEZIER_N_SEG+1];
	
	x_bz[0]=x[0]; y_bz[0]=y[0];
	x_bz[1]=x[4]; y_bz[1]=y[4];
	x_bz[2]=x[7]; y_bz[2]=y[7];
	x_bz[3]=x[11]; y_bz[3]=y[11];
	
	bz_stop_wp.bz0x=x_bz[0];
	bz_stop_wp.bz0y=y_bz[0];
	bz_stop_wp.bz4x=x_bz[1];
	bz_stop_wp.bz4y=y_bz[1];
	bz_stop_wp.bz7x=x_bz[2];
	bz_stop_wp.bz7y=y_bz[2];
	bz_stop_wp.bz11x=x_bz[3];
	bz_stop_wp.bz11y=y_bz[3];  
  float px = x_;
  float py = y_;
  float dist = sqrtf( powf(px-x_bz[gvf_c_stopwp.next_wp],2) + powf(py-y_bz[gvf_c_stopwp.next_wp],2));
  dist_WP=dist;

  gvf_c_stopwp.pxd = x_bz[gvf_c_stopwp.next_wp]; 
  gvf_c_stopwp.pyd = y_bz[gvf_c_stopwp.next_wp];
  if((dist <= gvf_c_stopwp.distance_stop)){	
  	if(gvf_c_stopwp.stop_at_wp && !gvf_c_stopwp.stay_still){
  		gvf_c_stopwp.stay_still = 1;
  		return dist;
  	}
  } 
  return dist;
  }
  
  bool increase_bz_pointer(void){
  gvf_c_stopwp.next_wp++;
  if (gvf_c_stopwp.next_wp>3) 
  	gvf_c_stopwp.next_wp=0;
  return false;
  }
