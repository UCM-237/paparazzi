/*
 * Copyright (C) 2021 Jesús Bautista <jesusbautistavillar@gmail.com> 
 *                    Hector García  <noeth3r@gmail.com>
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

#define AUTOPILOT_CORE_GUIDANCE_C

/** Mandatory dependencies header **/
#include "firmwares/rover/guidance/rover_guidance_steering.h"

#include "generated/airframe.h"

#include "modules/actuators/actuators_default.h"
#include "modules/radio_control/radio_control.h"
#include "autopilot.h"
#include "navigation.h"
#include "state.h"

#include "filters/pid.h" // Used for p+i speed controller
#include "modules/lidar/tfmini.h"

#include <math.h>
#include <stdio.h>

#include "modules/guidance/gvf_common.h"

// Guidance control main variables
rover_ctrl guidance_control;
// Protection against rollover
rover_rollover_protection rollover_protection;
// Obstacle avoidance
rover_obstacle_avoidance obstacle_avoidance;

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
#endif

// Moving Average filter. Number of samples
#ifndef MOV_AVG_M
#define MOV_AVG_M 10
#endif

PRINT_CONFIG_VAR(MOV_AVG_M)

// Speed moving average filter parameters
static int ptr_avg = 0;
static float speed_avg = 0;
static float mvg_avg[MOV_AVG_M] = {0};

// Distance moving average filter parameters
static int ptr_avg_dist = 0;
static float mvg_avg_dist[MOV_AVG_M] = {0};



#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
#endif

// Moving Average filter. Number of samples
#ifndef MOV_AVG_M
#define MOV_AVG_M 10
#endif

PRINT_CONFIG_VAR(MOV_AVG_M)

static int ptr_avg = 0;
static float speed_avg = 0;
static float mvg_avg[MOV_AVG_M] = {0};


static struct PID_f rover_pid;
static float time_step;
static float last_speed_cmd;
static uint8_t last_ap_mode;

// Integral, prop and derivative actions (telemetry)
static float i_action;
static float p_action;
static float d_action;

// Here the time
static uint32_t rover_time = 0;
static int reset_time = 0;

#if PERIODIC_TELEMETRY
static void send_rover_ctrl(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_ROVER_CTRL(trans, dev, AC_ID,
                    	    &guidance_control.cmd.speed,
                    	    &guidance_control.speed_error,
                    	    &guidance_control.throttle,
                    	    &guidance_control.cmd.delta,
                    	    &i_action,				// Integral action
                    	    &p_action,                        // Prop action 
                    	    &speed_avg);                      // Avg speed measured 
}
#endif


/** INIT function **/
void rover_guidance_steering_init(void)
{

  #if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ROVER_CTRL, send_rover_ctrl);
  #endif

  guidance_control.cmd.delta = 0.0;
  guidance_control.cmd.speed = 0.0;
  guidance_control.throttle  = 0.0;

	#if PERIODIC_TELEMETRY
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ROVER_CTRL, send_rover_ctrl);
	#endif

	guidance_control.cmd.delta = 0.0;
	guidance_control.cmd.speed = 0.0;
	guidance_control.throttle  = 0.0;

	last_speed_cmd = 0.0;
	last_ap_mode   = AP_MODE_KILL;

	guidance_control.speed_error = 0.0;
	guidance_control.kf = SR_MEASURED_KF;
	guidance_control.kp = 1400;
	guidance_control.ki = 600;
	guidance_control.kd = 1000;
	guidance_control.cmd_nonlinear.k = 9600;
	guidance_control.cmd.z_kappa = 0.0;
	guidance_control.cmd.z_ori = 0.0;

	init_pid_f(&rover_pid, guidance_control.kp, guidance_control.kd, guidance_control.ki, MAX_PPRZ*0.2);

  // Mov avg init
  float speed = stateGetHorizontalSpeedNorm_f();
  for(int k = 0; k < MOV_AVG_M; k++)
  	mvg_avg[k] = speed;
  speed_avg = speed;
  
  // Based on autopilot state machine frequency
  time_step = 1.f/PERIODIC_FREQUENCY;
}

/** CTRL functions **/
// Steering control (GVF)
void rover_guidance_steering_heading_ctrl(float omega) //GVF give us this omega
{
	float delta = 0.0;

  // Speed is bounded to avoid GPS noise while driving at small velocity
  //float speed = BoundSpeed(stateGetHorizontalSpeedNorm_f()); 
  float speed = speed_avg; // Use avg, avoid noise
  
  if (fabs(omega)>0.0) {
      delta = DegOfRad(-atanf(omega*DRIVE_SHAFT_DISTANCE/speed));
    }

  guidance_control.cmd.delta = BoundDelta(delta);
}

// Speed control (nonlinear or feedforward + pi)
void rover_guidance_steering_speed_ctrl(void) 
{
  // - Looking for setting update
  if (guidance_control.kp != rover_pid.g[0] || guidance_control.ki != rover_pid.g[2]) {
    set_gains_pid_f(&rover_pid, guidance_control.kp, 0.f, guidance_control.ki);
  }
  if (guidance_control.cmd.speed != last_speed_cmd) {
    last_speed_cmd = guidance_control.cmd.speed;
    //reset_pid_f(&rover_pid);
  }

  // Mov avg speed
  speed_avg = speed_avg - mvg_avg[ptr_avg]/MOV_AVG_M;
  mvg_avg[ptr_avg] = stateGetHorizontalSpeedNorm_f();
  speed_avg = speed_avg + mvg_avg[ptr_avg]/MOV_AVG_M;
  ptr_avg = (ptr_avg + 1) % MOV_AVG_M;

  // - Updating PID
  //guidance_control.speed_error = (guidance_control.cmd.speed - stateGetHorizontalSpeedNorm_f());
  guidance_control.speed_error = guidance_control.cmd.speed - speed_avg;
  update_pid_f(&rover_pid, guidance_control.speed_error, time_step);

  guidance_control.throttle = BoundThrottle(guidance_control.kf*guidance_control.cmd.speed + get_pid_f(&rover_pid));
  
  // Telemetry
  i_action = get_i_action(&rover_pid, guidance_control.speed_error, time_step);
  p_action = get_p_action(&rover_pid, guidance_control.speed_error);
}

// PI controller
void rover_guidance_steering_speed_ctrl_pid(void)
{
	// - Looking for setting update
	if (guidance_control.kp != rover_pid.g[0] || guidance_control.ki != rover_pid.g[2] || guidance_control.kd != rover_pid.g[1]) {
		set_gains_pid_f(&rover_pid, guidance_control.kp, guidance_control.kd, guidance_control.ki);
	}
	if (guidance_control.cmd.speed != last_speed_cmd) {
		last_speed_cmd = guidance_control.cmd.speed;
		//reset_pid_f(&rover_pid);
	}

	// - Updating PID
	guidance_control.speed_error = guidance_control.cmd.speed - speed_avg;
	update_pid_f(&rover_pid, guidance_control.speed_error, time_step);

  
	guidance_control.throttle = BoundThrottle(guidance_control.cmd.speed*guidance_control.kf + get_pid_f(&rover_pid));
	
	// Telemetry TODO: unify in one function
	i_action = get_i_action(&rover_pid, guidance_control.speed_error, time_step);
	p_action = get_p_action(&rover_pid, guidance_control.speed_error);
	d_action = get_d_action(&rover_pid, guidance_control.speed_error, time_step);
}

// Non linear controller 
void rover_guidance_steering_speed_ctrl_lyap(float dv_sp)
{
	guidance_control.speed_error = guidance_control.cmd.speed - speed_avg;
	float u = dv_sp + guidance_control.cmd_nonlinear.k * tanhf(speed_avg + 0.1) * guidance_control.speed_error;
	
	/* Since paparazzi uses a big throttle (between -9600 and 9600, compute the throttle with constant = 1)
	 * it is `clear' that if K = 1 doesnt hold for the CBF, neither does K = 9600
	*/
	float u_aux = dv_sp + tanhf(speed_avg + 0.1)*guidance_control.speed_error;
	
	if(rollover_protection.use_cbf){
		float amax = rollover_protection.max_lateral_accel;
		float w = stateGetBodyRates_f()->r;
		rollover_protection.h_cbf = 0.5*(amax*amax - speed_avg*speed_avg*w*w);
		float Lfh = -speed_avg*speed_avg*w*gvf_c_omega.omega;
		float Lgh = -speed_avg*w*w;
		if(Lfh + Lgh*u_aux + rollover_protection.beta*rollover_protection.h_cbf < 0)
			u = -guidance_control.cmd_nonlinear.k*(Lfh + rollover_protection.beta*rollover_protection.h_cbf)/Lgh;
				
	}
	guidance_control.throttle = BoundThrottle(u);
}


// Update dist measurement using moving average filter
void rover_guidance_steering_update_measurment(void)
{
	obstacle_avoidance.distance = obstacle_avoidance.distance - mvg_avg_dist[ptr_avg_dist]/MOV_AVG_M;
	tfmini_event();
	mvg_avg_dist[ptr_avg_dist] = tfmini.distance;
	obstacle_avoidance.distance = obstacle_avoidance.distance + mvg_avg_dist[ptr_avg_dist]/MOV_AVG_M;
	ptr_avg_dist = (ptr_avg_dist + 1) % MOV_AVG_M;
}

// Turn 90 degrees when obstacle is detected
float rover_guidance_steering_omega_obstacle_avoidance(void)
{

	float omega_ret = -1;
	if(obstacle_avoidance.use_speed_function)
		obstacle_avoidance.max_distance = speed_avg*2.0 + obstacle_avoidance.min_distance;
	
	if((obstacle_avoidance.distance <= obstacle_avoidance.max_distance) 
	   && (obstacle_avoidance.distance >= obstacle_avoidance.min_distance) 
	   && (obstacle_avoidance.turn_90_deg == 0)){
	   
		obstacle_avoidance.turn_90_deg = 1;
		obstacle_avoidance.old_psi = stateGetNedToBodyEulers_f()->psi;
	}
	if(obstacle_avoidance.turn_90_deg)
	{
		if(obstacle_avoidance.choose_direction == 1) // Turn right
			omega_ret = -1000;
		else // Turn left
			omega_ret = 1000;
		// 90 degrees turn
		if(DegOfRad(fabs(obstacle_avoidance.old_psi - stateGetNedToBodyEulers_f()->psi)) >= 90.0)
			obstacle_avoidance.turn_90_deg = 0;
	}
	return omega_ret;
}

// Turn when obstacle is detected
float rover_guidance_steering_omega_obstacle_avoidance_v2(void)
{

	float omega_ret = -1;
	
	if(obstacle_avoidance.use_speed_function)
		obstacle_avoidance.max_distance = speed_avg*2.0 + obstacle_avoidance.min_distance;
	
	
	if((obstacle_avoidance.distance <= obstacle_avoidance.max_distance) 
	   && (obstacle_avoidance.distance >= obstacle_avoidance.min_distance))
	{
		if(obstacle_avoidance.choose_direction == 1) // Turn right
			omega_ret = -1000;
		else // Turn left
			omega_ret = 1000;
	}
	return omega_ret;
}

/** PID RESET function**/
void rover_guidance_steering_pid_reset(void)
{
	// Reset speed PID
	if (rover_pid.sum != 0) {
		reset_pid_f(&rover_pid);
	}
}

void rover_guidance_steering_kill(void)
{
	guidance_control.cmd.delta = 0.0;
	guidance_control.cmd.speed = 0.0;
}
