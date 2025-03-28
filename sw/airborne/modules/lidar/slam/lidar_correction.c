/*
 * Copyright (C) 2025 Alejandro Rochas Fernández <alrochas@ucm.es>
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

// ------------------------------------


// TODO ESTO ESTA SIN REVISAR NI PROBAR, ME SIRVE PARA TERMINAR LA ESTRUCTURA GENERAL DEL CODIGO
// Y VER QUE FUNCIONA


#include "lidar_correction.h"
#include "modules/ins/ins_int.h"
#include "modules/nav/waypoints.h"

#include "math/pprz_algebra.h"
#include <math.h>
#include <float.h>
#include <state.h>


struct WallSystem wall_system;  // Sistema de paredes global

 

// Calcula la distancia de un punto P a un segmento AB y devuelve el punto C más cercano
static float distance_to_segment(const struct FloatVect2 *P, const struct FloatVect2 *A, 
  const struct FloatVect2 *B, struct FloatVect2 *C) {

  struct FloatVect2 vecAB = {B->x - A->x, B->y - A->y};
  struct FloatVect2 vecAP = {P->x - A->x, P->y - A->y};

  float length_sq = vecAB.x * vecAB.x + vecAB.y * vecAB.y;
  float dot_product = vecAP.x * vecAB.x + vecAP.y * vecAB.y;
  float t = (length_sq != 0) ? dot_product / length_sq : 0.0f;

  t = (t < 0.0f) ? 0.0f : ((t > 1.0f) ? 1.0f : t);

  C->x = A->x + t * vecAB.x;
  C->y = A->y + t * vecAB.y;

  float dx = P->x - C->x;
  float dy = P->y - C->y;
  return sqrtf(dx * dx + dy * dy);
}


float find_nearest_wall(const struct FloatVect2 *obstacle_pos, struct FloatVect2 *nearest_point) {

  if (!wall_system.converted_to_ltp) {
    return FLT_MAX;
  }

  float min_distance = FLT_MAX;
  
  // Iterar sobre todas las paredes
  for (uint8_t w = 0; w < wall_system.wall_count; w++) {
    struct Wall *wall = &wall_system.walls[w];
    
    // Iterar sobre todos los segmentos de la pared
    for (uint8_t p = 0; p < wall->count - 1; p++) {
      struct FloatVect2 p1 = wall->points_ltp[p];
      struct FloatVect2 p2 = wall->points_ltp[p+1];
      
      // Calcular distancia al segmento de línea
      // p1 y p2 son los extremos de la pared. El resultado se almacena en nearest_point
      float distance = distance_to_segment(obstacle_pos, &p1, &p2, nearest_point);
      
      if (distance < min_distance) {
        min_distance = distance;
        // waypoint_move_xy_i(16, POS_BFP_OF_REAL(p1.x), POS_BFP_OF_REAL(p1.y));
      }
    }
  }
  
  return min_distance;
}



// OBSTACULOS (ESTO HABRIA QUE PONERLO EN ALGUN MOMENTO EN UN XML)


void init_walls(void) {


  wall_system.wall_count = 0; // Por si acaso

  /* ==================== PISTA DE PÁDEL ==================== */
  struct Wall *padel_south = &wall_system.walls[wall_system.wall_count++];
  padel_south->points_wgs84[0] = (struct LlaCoor_f){40.451263, -3.729174, 0.0};
  padel_south->points_wgs84[1] = (struct LlaCoor_f){40.451234, -3.729169, 0.0};
  padel_south->count = 2;

  struct Wall *padel_northwest = &wall_system.walls[wall_system.wall_count++];
  padel_northwest->points_wgs84[0] = (struct LlaCoor_f){40.451197, -3.729123, 0.0}; // Esquina interior
  padel_northwest->points_wgs84[1] = (struct LlaCoor_f){40.451212, -3.728983, 0.0}; 
  padel_northwest->points_wgs84[2] = (struct LlaCoor_f){40.451221, -3.728912, 0.0}; // NE
  padel_northwest->count = 3;

  /* ==================== GRADAS ==================== */ 
  struct Wall *gradas_west = &wall_system.walls[wall_system.wall_count++];
  gradas_west->points_wgs84[0] = (struct LlaCoor_f){40.451918, -3.729198, 0.0}; 
  gradas_west->points_wgs84[1] = (struct LlaCoor_f){40.452028, -3.728153, 0.0}; 
  gradas_west->count = 2;


  wall_system.converted_to_ltp = false;

}


void convert_walls_to_ltp(void) {
  if (wall_system.converted_to_ltp || !ins_int.ltp_initialized) return;

  struct LtpDef_f *ned_origin = stateGetNedOrigin_f();

  for (int w = 0; w < wall_system.wall_count; w++) {
    for (int p = 0; p < wall_system.walls[w].count; p++) {
      struct NedCoor_f ned = {0.0f, 0.0f, 0.0f};
      ned_of_lla_point_f(&ned, ned_origin, &wall_system.walls[w].points_wgs84[p]);

      wall_system.walls[w].points_ltp[p].x = ned.x; 
      wall_system.walls[w].points_ltp[p].y = ned.y;

      wall_system.walls[w].points_ltp[p].x = ned_origin->lla.lat; 
      wall_system.walls[w].points_ltp[p].y = ned_origin->lla.lon;
    }
  }
  wall_system.converted_to_ltp = true;
}


// // Por si acaso 
// static void reset_wall(void) {
//   wall_system.converted_to_ltp = false;
// }