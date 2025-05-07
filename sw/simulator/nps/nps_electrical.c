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
#include "generated/airframe.h"
#include "modules/energy/electrical.h"

struct NpsElectrical nps_electrical;
struct NpsFdm fdm;

static struct EnuCoor_d rover_vel;
static struct LtpDef_d ltpdef;

double batery;
void nps_electrical_init(void)
{

#ifdef MAX_BAT_LEVEL
  nps_electrical.supply_voltage = MAX_BAT_LEVEL;
#else
  nps_electrical.supply_voltage = 11.1;
#endif
batery = nps_electrical.supply_voltage;
printf("Batería inicial = %f", batery);
}

void nps_electrical_run_step(double time __attribute__((unused)))
{
//--------IMPLEMENTACIÓN MODELO DE CONSUMO-----------------//

  // Actualizar velocidad desde el simulador
  rover_vel.x = fdm.ecef_ecef_vel.x; 
  rover_vel.y = fdm.ecef_ecef_vel.y;  
  //printf("Velocidad x electrical = %f      Velocidad y electrical = %f\n", rover_vel.x, rover_vel.y);
  // Calcular módulo de la velocidad
  double speed = FLOAT_VECT2_NORM(rover_vel);

  // Simulación de consumo de batería
  
  double consumo = (16.242643*pow(throttle,2) + 0.085211*throttle - 1.491132) * time; // No me cuadra esta ecuación ya que si es el consumo a v=0 se estaría cargando la batería.
  if (consumo < 0 ){
    consumo = 0
  }
  
  batery -= consumo;  // Restar el consumo de batería (asumiendo que es descarga)

  electrical.vsupply = batery;

  // Imprimir para depuración
  //printf("Bateria = %f\n", electrical.vsupply);
  //printf("Consumo = %f\n", consumo);
  //printf("time = %f\n", time);
  //printf("velocidad = %f\n", speed);
}


