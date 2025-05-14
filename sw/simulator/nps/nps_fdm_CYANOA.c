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

#include "nps_fdm.h"

#include <stdlib.h>
#include <stdio.h>

#include "math/pprz_geodetic.h"
#include "math/pprz_geodetic_double.h"
#include "math/pprz_geodetic_float.h"
#include "math/pprz_algebra.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_isa.h"

#include "generated/airframe.h"
#include "generated/flight_plan.h"

#include "firmwares/rover/guidance/boat_guidance.h"
#include "state.h"

// Check if rover firmware 
#ifndef ROVER_FIRMWARE
#error "The module nps_fdm_rover is designed for rovers and doesn't support other firmwares!!"
#endif

// NpsFdm structure
struct NpsFdm fdm;

// Reference point
static struct LtpDef_d ltpdef;

// Static functions declaration
static void init_ltp(void);
double bat = 8.4;
/** Physical model structures **/
static struct EnuCoor_d rover_pos;
static struct EnuCoor_d rover_vel;
static struct EnuCoor_d rover_acc;


/******************************************************/
// Cálculo de la fuerza de los motores //
double calculo_motor_izq (double throttle_izq)
{
  //printf("throttle izq = %f\n", throttle_izq);
  double fuerza_motor_izq = (2.4277*pow(throttle_izq,3) + 0.4041*pow(throttle_izq,2) + 0.9662 * throttle_izq - 0.0004)*9.8/physical_params.mass;
  //printf("fuerza_motor_izq = %f\n", fuerza_motor_izq);
  if (throttle_izq == 0) 
  {
    fuerza_motor_izq = 0;
  }
  return fuerza_motor_izq;
}

double calculo_motor_dcho (double throttle_derecho)
{
  //printf("throttle dcho = %f\n", throttle_derecho*9600);
  double fuerza_motor_dcho = (2.4277*pow(throttle_derecho,3) + 0.4041*pow(throttle_derecho,2) + 0.9662 * throttle_derecho - 0.0004)*9.8/physical_params.mass;
  //printf("fuerza_motor_dcho = %f\n", fuerza_motor_dcho);
  if (throttle_derecho == 0) 
  {
    fuerza_motor_dcho = 0;
  }
  return fuerza_motor_dcho;
}


/** NPS FDM rover init ***************************/
void nps_fdm_init(double dt)
{
  fdm.init_dt = dt; // (1 / simulation freq)
  fdm.curr_dt = dt; // ¿Configurable from GCS? Es posible, sería añadirlo en el xml donde se ajustan también los rozamientos

  fdm.on_ground = TRUE;

  fdm.nan_count = 0;
  fdm.pressure = -1;
  fdm.pressure_sl = PPRZ_ISA_SEA_LEVEL_PRESSURE;
  fdm.total_pressure = -1;
  fdm.dynamic_pressure = -1;
  fdm.temperature = -1;
  fdm.phi_d = 0;
  fdm.ltpprz_to_body_eulers.psi = 0.0;
  fdm.n_x = 0;
  fdm.n_y = 0;
  
  /** Physical model parameters **/
  init_ltp();
}


struct PhysicalParams physical_params = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };  // Valores predeterminados

// Función para inicializar los parámetros de perturbación
void physical_params_init(void) {
  // Valores predeterminados en caso de que el XML no lo configure
  physical_params.mu_x_sim = 1;
  physical_params.mu_y_sim = 1;
  physical_params.mu_w_sim = 1;
     
  // Variables para almacenar viento y marea base (configurables)
  physical_params.wind_north_base = 0.50;
  physical_params.wind_east_base = 0.50;

  // Puedes definir amplitudes para viento y marea por separado
  physical_params.current_ampl_x = 0.0;
  physical_params.current_ampl_y = 0.00;

  physical_params.wind_ampl_x = 0.0;
  physical_params.wind_ampl_y = 0.0;

  physical_params.wind_x = 0;
  physical_params.wind_y = 0;
  
  physical_params.current_x = 0;
  physical_params.current_y = 0;   
  
  physical_params.mass = 50; //Masa del barco = 50kg
  physical_params.motor_separation = 0.1; //Separación lateral en metros
  physical_params.motor_longitudinal_offset = 0.1; //Separación longitudinal en metros
}



void nps_fdm_run_step(bool launch __attribute__((unused)), double *commands, int commands_nb __attribute__((unused)))
{ 

  /****************************************************************************
  PHYSICAL MODEL
  -------------
  The physical model of your rover goes here. This physics takes place in
  the LTP plane (so we transfer every integration step to NED and ECEF at the end).
  */

  #ifdef COMMAND_MRIGHT // STEERING ROVER PHYSICS
  // Steering rover cmds: 
  //    COMMAND_MRIGHT -> acceleration in right motor
  //    COMMAND_MLEFT -> acceleration in letf motor
  
  //-----------IMPLEMENTACIÓN DEL MODELO FÍSICO-------------------//
  
  /** Physical model for car-like robots .................. **/
  // From previous step...
  enu_of_ecef_point_d(&rover_pos, &ltpdef, &fdm.ecef_pos);
  enu_of_ecef_vect_d(&rover_vel, &ltpdef, &fdm.ecef_ecef_vel);
  
  //Esta es la fuerza de los motores. La función está extraida de las especificaciones del motor
  double f_right = calculo_motor_dcho(commands[COMMAND_MRIGHT]);
  double f_left = calculo_motor_izq(commands[COMMAND_MLEFT]);

  // Posiciones respecto al CM
  double dx = physical_params.motor_separation / 2.0;
  double l  = physical_params.motor_longitudinal_offset;
  
  double phi = fdm.ltpprz_to_body_eulers.psi;
  //Vamos a calcular primero la fuerza de los motores
  double a = cos(phi);
  double b = sin(phi);

  // Torque de cada motor: r x F = x*Fy - y*Fx
  double tau_left  = (-dx) * f_left;
  double tau_right = (dx) * f_right;

  double speed = FLOAT_VECT2_NORM(rover_vel);
  double tau = (tau_right + tau_left);
  //double tau = (-f_right+f_left);
  //printf("tau = %f\n", tau);
  double fa = (f_right+f_left);
  update_environment_perturbations(fdm.curr_dt);
  

  //printf("psi ltp %f\n psi ltpprz %f\n", fdm.ltp_to_body_eulers.psi, fdm.ltpprz_to_body_eulers.psi);
  // Setting accelerations
  rover_acc.x = fa * a - (pow(a,2)*physical_params.mu_x_sim*(rover_vel.x-fdm.n_x) +  pow(b,2)*physical_params.mu_y_sim*(rover_vel.x-fdm.n_x) + a*b*(rover_vel.y-fdm.n_y)*(physical_params.mu_x_sim-physical_params.mu_y_sim));
  rover_acc.y = fa * b - (pow(b,2)*physical_params.mu_x_sim*(rover_vel.y-fdm.n_y) +  pow(a,2)*physical_params.mu_x_sim*(rover_vel.y-fdm.n_y) + a*b*(rover_vel.x-fdm.n_x)*(physical_params.mu_x_sim-physical_params.mu_y_sim));
  // No se ha añadido η, este término hace referencia a resistencias o correcciones del modelo. 
  //printf("phi_d %f\n", fdm.phi_d);
  double phi_dd = tau - physical_params.mu_w_sim*fdm.phi_d; //aceleracion angular
  //printf("Throttle izq = %f\n Throttle dch = %f\n",commands[COMMAND_MLEFT], commands[COMMAND_MRIGHT]);
  //printf("Tau = %f\n fa = %f\n", tau, fa);
  // Velocities (EULER INTEGRATION)
  rover_vel.x += rover_acc.x * fdm.curr_dt;
  rover_vel.y += rover_acc.y * fdm.curr_dt;
  fdm.phi_d  += phi_dd*fdm.curr_dt;
  //printf("Velocidad x = %f      Velocidad y = %f\n", rover_vel.x, rover_vel.y);
  // Positions
  rover_pos.x += rover_vel.x * fdm.curr_dt;
  rover_pos.y += rover_vel.y * fdm.curr_dt;
  phi += fdm.phi_d * fdm.curr_dt;
  
  // phi have to be contained in [-180º,180º). So:
  phi = (phi > M_PI)? - 2*M_PI + phi : (phi < -M_PI)? 2*M_PI + phi : phi; //NORMALIZACIÓN DE PHI ORIGINAL
  
  //phi = M_PI/2;
  //printf("phi = %f\n", phi);
  
  #else
  #warning "The physics of this rover are not yet implemented in nps_fdm_rover!!"
  #endif // STEERING ROVER PHYSICS


  /****************************************************************************/
  // Coordenates transformations |
  // ----------------------------|

  /* in ECEF */
  ecef_of_enu_point_d(&fdm.ecef_pos, &ltpdef, &rover_pos);
  ecef_of_enu_vect_d(&fdm.ecef_ecef_vel, &ltpdef, &rover_vel);
  ecef_of_enu_vect_d(&fdm.ecef_ecef_accel, &ltpdef, &rover_acc);

  /* in NED */
  ned_of_ecef_point_d(&fdm.ltpprz_pos, &ltpdef, &fdm.ecef_pos);
  ned_of_ecef_vect_d(&fdm.ltpprz_ecef_vel, &ltpdef, &fdm.ecef_ecef_vel);
  ned_of_ecef_vect_d(&fdm.ltpprz_ecef_accel, &ltpdef, &fdm.ecef_ecef_accel);

  /* Eulers */
  fdm.ltpprz_to_body_eulers.psi = phi;

  // ENU to NED and exporting heading (psi euler angle)
  fdm.ltp_to_body_eulers.psi = - phi + M_PI / 2;

  // Exporting Eulers to AHRS (in quaternions)
  double_quat_of_eulers(&fdm.ltp_to_body_quat, &fdm.ltp_to_body_eulers);
  
  // Angular vel & acc
  fdm.body_ecef_rotvel.r   = fdm.phi_d; 
  fdm.body_ecef_rotaccel.r = phi_dd;

}


/**************************
 ** Generating LTP plane **
 **************************/

static void init_ltp(void)
{

  struct LlaCoor_d llh_nav0; /* Height above the ellipsoid */
  llh_nav0.lat = RadOfDeg((double)NAV_LAT0 / 1e7);
  llh_nav0.lon = RadOfDeg((double)NAV_LON0 / 1e7);

  struct EcefCoor_d ecef_nav0;

  ecef_of_lla_d(&ecef_nav0, &llh_nav0);

  ltp_def_from_ecef_d(&ltpdef, &ecef_nav0);
  fdm.ecef_pos = ecef_nav0;

  fdm.ltp_g.x = 0.;
  fdm.ltp_g.y = 0.;
  fdm.ltp_g.z = 0.; // accel data are already with the correct format


#ifdef AHRS_H_X
  fdm.ltp_h.x = AHRS_H_X;
  fdm.ltp_h.y = AHRS_H_Y;
  fdm.ltp_h.z = AHRS_H_Z;
  PRINT_CONFIG_MSG("Using magnetic field as defined in airframe file.")
#else
  fdm.ltp_h.x = 0.4912;
  fdm.ltp_h.y = 0.1225;
  fdm.ltp_h.z = 0.8624;
#endif

}

/*****************************************************/
// Atmosphere function (we don't need that features) //

void update_environment_perturbations(double time) 
{
  //Calculamos los vientos
  set_wind(time);
  
  //Calculamos las corrientes
  set_current(time);

  // Perturbación total del medio (componente este y norte)
  fdm.n_x = physical_params.wind_x + physical_params.current_x;
  fdm.n_y = physical_params.wind_y + physical_params.current_y;
}

void set_wind(double time)
{
  physical_params.wind_x = physical_params.wind_ampl_x * sin(0.2 * time);
  physical_params.wind_y = physical_params.wind_ampl_y * cos(0.25 * time);
}

void set_current(double time)
{
  physical_params.current_x = physical_params.current_ampl_x * sin(2.0 * M_PI * time / (12.0 * 3600.0));
  physical_params.current_y = physical_params.current_ampl_y * cos(2.0 * M_PI * time / (12.0 * 3600.0));
}

void nps_fdm_set_wind(double speed __attribute__((unused)),
                      double dir __attribute__((unused)))
{
}

void nps_fdm_set_wind_ned(double wind_north __attribute__((unused)),
                          double wind_east __attribute__((unused)),
                          double wind_down __attribute__((unused)))
{
}

void nps_fdm_set_turbulence(double wind_speed __attribute__((unused)),
                            int turbulence_severity __attribute__((unused)))
{
}

void nps_fdm_set_temperature(double temp __attribute__((unused)),
                             double h __attribute__((unused)))
{
}

/**********************************************/

