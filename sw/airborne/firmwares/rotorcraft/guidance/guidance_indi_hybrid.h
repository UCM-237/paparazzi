/*
 * Copyright (C) 2015 Ewoud Smeur <ewoud.smeur@gmail.com>
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
 */

/**
 * @file firmwares/rotorcraft/guidance/guidance_indi_hybrid.h
 *
 * A guidance mode based on Incremental Nonlinear Dynamic Inversion
 * Come to ICRA2016 to learn more!
 *
 */

#ifndef GUIDANCE_INDI_HYBRID_H
#define GUIDANCE_INDI_HYBRID_H

#include "std.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"
#include "filters/high_pass_filter.h"
#include "firmwares/rotorcraft/guidance.h"
#include "firmwares/rotorcraft/stabilization.h"


#ifndef GUIDANCE_INDI_HYBRID_U
#error Please use guidance_indi_hybrid_tailsitter or guidance_indi_hybrid_quadplane in your airframe file.
#endif


// TODO change names for _indi_hybrid_

extern void guidance_indi_init(void);
extern void guidance_indi_enter(void);
extern float guidance_indi_get_liftd(float pitch, float theta);
extern void guidance_indi_calcg_wing(float Gmat[GUIDANCE_INDI_HYBRID_V][GUIDANCE_INDI_HYBRID_U], struct FloatVect3 a_diff, float v_body[GUIDANCE_INDI_HYBRID_V]);

#if GUIDANCE_INDI_HYBRID_USE_WLS
#include "math/wls/wls_alloc.h"
extern struct WLS_t wls_guid_p;
extern void guidance_indi_hybrid_set_wls_settings(float body_v[3], float roll_angle, float pitch_angle);
#endif

enum GuidanceIndiHybrid_HMode {
  GUIDANCE_INDI_HYBRID_H_POS,
  GUIDANCE_INDI_HYBRID_H_SPEED,
  GUIDANCE_INDI_HYBRID_H_ACCEL
};

enum GuidanceIndiHybrid_VMode {
  GUIDANCE_INDI_HYBRID_V_POS,
  GUIDANCE_INDI_HYBRID_V_SPEED,
  GUIDANCE_INDI_HYBRID_V_ACCEL
};

extern struct StabilizationSetpoint guidance_indi_run(struct FloatVect3 *accep_sp, float heading_sp);
extern struct StabilizationSetpoint guidance_indi_run_mode(bool in_flight, struct HorizontalGuidance *gh, struct VerticalGuidance *gv, enum GuidanceIndiHybrid_HMode h_mode, enum GuidanceIndiHybrid_VMode v_mode);
extern void guidance_set_min_max_airspeed(float min_airspeed, float max_airspeed);
extern void guidance_set_max_bank_angle(float max_bank);
extern void guidance_set_max_climb_speed(float max_climb_speed_quad, float max_climb_speed_fwd);
extern void guidance_set_max_descend_speed(float max_descend_speed_quad, float max_descend_speed_fwd);

struct guidance_indi_hybrid_params {
  float pos_gain;
  float pos_gainz;
  float speed_gain;
  float speed_gainz;
  float heading_bank_gain;
  float liftd_asq;
  float liftd_p80;
  float liftd_p50;
  float min_airspeed;
  float max_airspeed;
  float stall_protect_gain;
  float climb_vspeed_fwd;
  float descend_vspeed_fwd;
  float climb_vspeed_quad;
  float descend_vspeed_quad;
};

extern struct FloatVect3 sp_accel;
extern struct FloatVect3 gi_speed_sp;
extern struct guidance_indi_hybrid_params gih_params;

extern float guidance_indi_pitch_pref_deg;

extern float gi_unbounded_airspeed_sp;

extern float guidance_indi_thrust_z_eff;

extern float guidance_indi_specific_force_gain;
extern bool take_heading_control;
extern float guidance_indi_max_bank;
extern float guidance_indi_min_pitch;
extern bool force_forward;       ///< forward flight for hybrid nav
extern bool guidance_indi_airspeed_filtering;
extern bool coordinated_turn_use_accel;

#endif /* GUIDANCE_INDI_HYBRID_H */
