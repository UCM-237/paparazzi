/*
 * Copyright (C) 2025 Lia Garcia Pérez
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

/** @file gvf_cbf.h
 *
 *  Control barrier function modifiyng the GVF to avoid other neighbour robots
 */

#ifndef GVF_CBF_H
#define GVF_CBF_H

/*! Default GCS trajectory painter */
#ifndef GVF_OCAML_GCS
#define GVF_OCAML_GCS true
#endif

#include <math.h>
#include <std.h>
#include "state.h"
#include "firmwares/rover/navigation.h"

// Default number of neighbors per robot
#ifndef CBF_MAX_NEIGHBORS
#define CBF_MAX_NEIGHBORS 4

#endif

//
#ifndef CBF_NEI_AC_IDS
#error "You have to define CBF_NEI_AC_IDS in the ariframe file!"
#endif // CBF_NEI_AC_IDS

#define N CBF_MAX_NEIGHBORS

// CBF parameters

struct cbf_parameters {
  float r;
  float alpha;
  float timeout;
};
// CBF control
struct cbf_con {
  float omega_safe;
  uint16_t n_neighborns;
  uint32_t last_transmision;
};

// CBF telemetry
struct cbf_tel {
  uint16_t acs_id[CBF_MAX_NEIGHBORS];
  uint8_t acs_available[CBF_MAX_NEIGHBORS];
  uint16_t acs_timeslost[CBF_MAX_NEIGHBORS];

  float uref[CBF_MAX_NEIGHBORS];
  float prel_norm[CBF_MAX_NEIGHBORS];
  float px_rel[CBF_MAX_NEIGHBORS];
  float py_rel[CBF_MAX_NEIGHBORS];
  float vx_rel[CBF_MAX_NEIGHBORS];
  float vy_rel[CBF_MAX_NEIGHBORS];
  float h_ref[CBF_MAX_NEIGHBORS];
  float h_dot_ref[CBF_MAX_NEIGHBORS];
  float psi_ref[CBF_MAX_NEIGHBORS];
};

// CBF state
typedef struct{
  float xi_x;
  float xi_y;
  uint8_t nei;
  uint8_t active_conds;
  float xicbf_x;
  float xicbf_y;
  float r;
  float alpha;
  float x;
  float y;
  float speed;
  float course;
  float vx;
  float vy;
  float uref;
} cbf_state_t;

// CBF obstacle tables
typedef struct{
  uint16_t ac_id;
  cbf_state_t state;
  
  float omega_safe;
  bool available;
  uint32_t t_last_msg;
} cbf_tab_entrie_t;

/* Structure definitions ------------------------ */


extern struct cbf_con cbf_control;
extern struct cbf_tel cbf_telemetry;
extern cbf_state_t cbf_ac_state;
extern cbf_tab_entrie_t cbf_obs_tables[CBF_MAX_NEIGHBORS];

extern struct cbf_parameters cbf_param;
void ldltDecomposition(double A[N][N], double L[N][N], double D[N]);
void forwardSubstitution(double L[N][N], double b[N], double y[N]);
void diagonalSolve(double D[N], double y[N], double z[N]);
void backwardSubstitution(double L[N][N], double z[N], double x[N]);
void inverseUsingLDLT(double L[N][N], double D[N], double invA[N][N]);

/* External functions --------------------------- */
extern void gvf_cbf_init(void);
extern void cbf_init(void) ;
extern void gvf_cbf(void);
extern void parse_CBF_STATE(uint8_t *buf);
#endif // GVF_CBF_H

