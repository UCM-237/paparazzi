/*
 * Copyright (C) 2021 Jesús Bautista <jesusbautistavillar@gmail.com> 
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

#ifndef BOAT_GUIDANCE_H
#define BOAT_GUIDANCE_H

#include "std.h"
#include <math.h>

/** Generated airframe.h from airframe.xml
 * - fun: SetActuatorsFromCommands
 * - var: commands
 * - var: hardware and construction parameters
 **/
 
#include "generated/airframe.h"

// Check critical global definitiones
#ifndef SERVO_MOTOR_LEFT
#error "Boat guidance requires the servo MOTOR_LEFT"
#endif

#ifndef SERVO_MOTOR_RIGHT
#error "Boat guidance requires the servo MOTOR_RIGHT"
#endif

#ifndef COMMAND_MLEFT
#error "Boat guidance requires the command COMMAND_MLEFT"
#endif

#ifndef COMMAND_MRIGHT
#error "Boat guidance requires the command COMMAND_MRIGHT"
#endif


/** Global variables definitions **/
// MIN_SPEED, MAX_SPEED: Min and max state speed (m/s)
#ifndef MAX_SPEED //TODO: Si no se usa la velocidad en el bearing control, se puede eliminar
#define MAX_SPEED 999.0 //We don't really use that variable
#endif
#ifndef MIN_SPEED //TODO: Si no se usa la velocidad en el bearing control, se puede eliminar
#define MIN_SPEED 0.2 //But this one is mandatory because we have
#endif                //to deal with GPS noise (and 1/v in guidance control).

// SR_MEASURED_KF: Lineal feed forward control constant (have to be measured in new servos)
#ifndef BOAT_MEASURED_KF
#define BOAT_MEASURED_KF 10
#warning "Construction constant BOAT_MEASURED_KF for boat speed ctrl not defined"
#endif


/** Steering rover guidance STRUCTURES **/
// High level commands
typedef struct {
  float speed;
} guidance_cmd_t;

// Main structure
typedef struct {
  guidance_cmd_t cmd;
  float gvf_omega;
  float throttle;
  float bearing;

  float speed_error;
  float kf;
  float kp;
  float ki;
} ctrl_t;

extern ctrl_t guidance_control;

/** Boat guidance EXT FUNCTIONS **/
extern void boat_guidance_init(void);
extern void boat_guidance_bearing_ctrl(void);
extern void boat_guidance_speed_ctrl(void);
extern void boat_guidance_pid_reset(void);
extern void boat_guidance_kill(void);


/** MACROS **/
// Bound speed | TODO: Si no se usa la velocidad en el bearing control, se puede eliminar
#define BoundSpeed(speed) (speed <  MIN_SPEED ? MIN_SPEED : \
                          (speed >  MAX_SPEED ? MAX_SPEED : \
                           speed));
                           
// Bound commands
#define BoundCmd(cmd) TRIM_PPRZ((int)cmd);

// Set AP throttle value
#define SetAPThrottleFromCommands(void) { \
autopilot.throttle = (commands[COMMAND_MLEFT] + commands[COMMAND_MRIGHT])/2; }

#endif // BOAT_GUIDANCE_H
