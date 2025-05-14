/*
 * Copyright (C) 2013 Felix Ruess <felix.ruess@gmail.com>
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
 * @file nps_electrical.c
 * Electrical status (bat voltage) for NPS.
 */

#include "nps_electrical.h"
#include "nps_fdm.h"
#include "nps_main.h"

#include "autopilot.h"

#include "generated/airframe.h"
#include "modules/energy/electrical.h"

struct NpsElectrical nps_electrical;
struct NpsFdm fdm;

static struct EnuCoor_d rover_vel;
static struct LtpDef_d ltpdef;
double Ah_0 = 30;
double bat_status;
double consumo_acum;
double batery;
void nps_electrical_init(void)
{

#ifdef MAX_BAT_LEVEL
  nps_electrical.supply_voltage = MAX_BAT_LEVEL;
#else
  nps_electrical.supply_voltage = 11.1;
#endif
//batery = nps_electrical.supply_voltage;
batery = 0;
printf("Batería inicial = %f", batery);
}

void nps_electrical_run_step(double time __attribute__((unused)))
{
//--------IMPLEMENTACIÓN MODELO DE CONSUMO-----------------//
  // Calcular módulo de la velocidad
  double *commands = nps_autopilot.commands; //ESTO LO PASA A %
  // Simulación de consumo de batería
  double consumo_right = (16.242643*pow(commands[COMMAND_MRIGHT],2) + 0.085211*commands[COMMAND_MRIGHT] - 1.491132)*fdm.init_dt/3600; //Amperaje en el instante de tiempo dt
  double consumo_left = (16.242643*pow(commands[COMMAND_MLEFT],2) + 0.085211*commands[COMMAND_MLEFT] - 1.491132)*fdm.init_dt/3600; //Amperaje en el instante de tiempo dt
  
  if (consumo_right < 0){
    consumo_right = 0.000000001;
  }
  
  if (consumo_left < 0){
    consumo_left = 0.000000001;
  }
  
  double consumo = consumo_right + consumo_left;
  
  consumo_acum += consumo;
  bat_status = 100*(1-consumo_acum/Ah_0);
  
  electrical.vsupply = bat_status;

  batery += consumo;  // Restar el consumo de batería (asumiendo que es descarga)
  // Imprimir para depuración
  //printf("Bateria = %f\n", electrical.vsupply);
  //printf("Throttle_right = %f    Throttle_left = %F\n", commands[COMMAND_MRIGHT], commands[COMMAND_MLEFT]);
  //printf("Consumo_right = %f    consumo_left = %F\n", consumo_right, consumo_left);
  //printf("time = %f\n", fdm.init_dt);
  
}


