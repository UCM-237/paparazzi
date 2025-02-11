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
  gvf_trajectory.type = NONE;

  // gvf_common.h
  gvf_c_stopwp.stay_still = 0;
  gvf_c_stopwp.stop_at_wp = 0;
  gvf_c_stopwp.distance_stop = 0.1;
  gvf_c_stopwp.wait_time = 1;
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GVF, send_gvf);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_NUM_WP_MOVED, send_num_wp_moved);
#endif
}

// GENERIC TRAJECTORY CONTROLLER
void gvf_control_2D(float ke, float kn, float e,
                    struct gvf_grad *grad, struct gvf_Hess *hess)
{
  gvf_t0 = get_sys_time_msec();
  
  gvf_low_level_getState();
  float course = gvf_state.course;
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

  float norm_pd_dot = sqrtf(pdx_dot * pdx_dot + pdy_dot * pdy_dot);
  float md_x = pdx_dot / norm_pd_dot;
  float md_y = pdy_dot / norm_pd_dot;

  float Apd_dot_dot_x = -ke * (nx * px_dot + ny * py_dot) * nx;
  float Apd_dot_dot_y = -ke * (nx * px_dot + ny * py_dot) * ny;

  float Bpd_dot_dot_x = ((-ke * e * H11) + s * H21) * px_dot
                        + ((-ke * e * H12) + s * H22) * py_dot;
  float Bpd_dot_dot_y = -(s * H11 + (ke * e * H21)) * px_dot
                        - (s * H12 + (ke * e * H22)) * py_dot;

  float pd_dot_dot_x = Apd_dot_dot_x + Bpd_dot_dot_x;
  float pd_dot_dot_y = Apd_dot_dot_y + Bpd_dot_dot_y;

  float md_dot_const = -(md_x * pd_dot_dot_y - md_y * pd_dot_dot_x)
                       / norm_pd_dot;

  float md_dot_x =  md_y * md_dot_const;
  float md_dot_y = -md_x * md_dot_const;

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
}

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

bool gvf_lines_array_wp_v2(uint8_t wp0, uint8_t wp1, uint8_t wp2, uint8_t wp3, uint8_t wp4, uint8_t wp5, uint8_t wp6, uint8_t wp7, uint8_t wp8, uint8_t wp9, uint8_t wp10, uint8_t wp11, uint8_t wp12, uint8_t wp13, uint8_t wp14, uint8_t wp15, uint8_t wp16, uint8_t wp17, uint8_t wp18, uint8_t wp19, uint8_t wp20, uint8_t wp21, uint8_t wp22, uint8_t wp23, uint8_t wp24, uint8_t wp25, uint8_t wp26, uint8_t wp27, uint8_t wp28, uint8_t wp29, uint8_t wp30, uint8_t wp31, uint8_t wp32, uint8_t wp33, uint8_t wp34, uint8_t wp35, uint8_t wp36, uint8_t wp37, uint8_t wp38, uint8_t wp39, uint8_t wp40, uint8_t wp41, uint8_t wp42, uint8_t wp43, uint8_t wp44, uint8_t wp45, uint8_t wp46, uint8_t wp47, uint8_t wp48, uint8_t wp49, uint8_t wp50, float d1, float d2)
{
	// Create the points
	gvf_trajectory.type = 2;
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
	x[7] = WaypointX(wp7); y[7] = WaypointY(wp7);
  x[8] = WaypointX(wp8); y[8] = WaypointY(wp8);
  x[9] = WaypointX(wp9); y[9] = WaypointY(wp9);
  x[10] = WaypointX(wp10); y[10] = WaypointY(wp10);
  x[11] = WaypointX(wp11); y[11] = WaypointY(wp11);
  x[12] = WaypointX(wp12); y[12] = WaypointY(wp12);
  x[13] = WaypointX(wp13); y[13] = WaypointY(wp13);
  x[14] = WaypointX(wp14); y[14] = WaypointY(wp14);
  x[15] = WaypointX(wp15); y[15] = WaypointY(wp15);
  x[16] = WaypointX(wp16); y[16] = WaypointY(wp16);
  x[17] = WaypointX(wp17); y[17] = WaypointY(wp17);
  x[18] = WaypointX(wp18); y[18] = WaypointY(wp18);
  x[19] = WaypointX(wp19); y[19] = WaypointY(wp19);
  x[20] = WaypointX(wp20); y[20] = WaypointY(wp20);
  x[21] = WaypointX(wp21); y[21] = WaypointY(wp21);
  x[22] = WaypointX(wp22); y[22] = WaypointY(wp22);
  x[23] = WaypointX(wp23); y[23] = WaypointY(wp23);
  x[24] = WaypointX(wp24); y[24] = WaypointY(wp24);
  x[25] = WaypointX(wp25); y[25] = WaypointY(wp25);
  x[26] = WaypointX(wp26); y[26] = WaypointY(wp26);
  x[27] = WaypointX(wp27); y[27] = WaypointY(wp27);
  x[28] = WaypointX(wp28); y[28] = WaypointY(wp28);
  x[29] = WaypointX(wp29); y[29] = WaypointY(wp29);
  x[30] = WaypointX(wp30); y[30] = WaypointY(wp30);
  x[31] = WaypointX(wp31); y[31] = WaypointY(wp31);
  x[32] = WaypointX(wp32); y[32] = WaypointY(wp32);
  x[33] = WaypointX(wp33); y[33] = WaypointY(wp33);
  x[34] = WaypointX(wp34); y[34] = WaypointY(wp34);
  x[35] = WaypointX(wp35); y[35] = WaypointY(wp35);
  x[36] = WaypointX(wp36); y[36] = WaypointY(wp36);
  x[37] = WaypointX(wp37); y[37] = WaypointY(wp37);
  x[38] = WaypointX(wp38); y[38] = WaypointY(wp38);
  x[39] = WaypointX(wp39); y[39] = WaypointY(wp39);
  x[40] = WaypointX(wp40); y[40] = WaypointY(wp40);
  x[41] = WaypointX(wp41); y[41] = WaypointY(wp41);
  x[42] = WaypointX(wp42); y[42] = WaypointY(wp42);
  x[43] = WaypointX(wp43); y[43] = WaypointY(wp43);
  x[44] = WaypointX(wp44); y[44] = WaypointY(wp44);
  x[45] = WaypointX(wp45); y[45] = WaypointY(wp45);
  x[46] = WaypointX(wp46); y[46] = WaypointY(wp46);
  x[47] = WaypointX(wp47); y[47] = WaypointY(wp47);
  x[48] = WaypointX(wp48); y[48] = WaypointY(wp48);
  x[49] = WaypointX(wp49); y[49] = WaypointY(wp49);
  x[50] = WaypointX(wp50); y[50] = WaypointY(wp50);

  
  int j = 0;
	for(int k = 0; k < num_wp_moved-1; k++)
	{
	
		gvf_lines_array[k].p1x = x[k];
		gvf_lines_array[k].p1y = y[k];
		gvf_lines_array[k].p2x = x[k+1];
		gvf_lines_array[k].p2y = y[k+1];
		//printf("Valor de k = %d \n", k);
	}
	struct EnuCoor_f *p = stateGetPositionEnu_f();
 	float px = p->x;
	float py = p->y;
	float dist = sqrtf( powf(px-gvf_lines_array[gvf_control.which_line].p2x,2) + powf(py-gvf_lines_array[gvf_control.which_line].p2y,2));
	if((dist <= gvf_c_stopwp.distance_stop )){
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

