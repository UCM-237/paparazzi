/*
 * Copyright (C) 2025 Alejandro Rochas Fernández <alrochas@ucm.es>
 *
 * This file is part of paparazzi
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
/**
 * @file "filters/moving_average_filter.h"
 *
 * Moving Average Filter implementation (used in INS SLAM EKF for the accelerations)
 */


#ifndef MOVING_AVERAGE_FILTER_H_
#define MOVING_AVERAGE_FILTER_H_

#define MAX_MOVING_AVG_WINDOW_SIZE  20


#include "std.h"

typedef struct {
  float buffer[MAX_MOVING_AVG_WINDOW_SIZE];        // Buffer circular de muestras
  uint16_t window_size; // Tamaño de la ventana
  uint16_t index;       // Índice actual del buffer circular
  bool initialized;     // Flag de inicialización
  float sum;            // Suma actual del buffer (para optimización)
} moving_avg_filter_t;


extern void moving_avg_filter_init(moving_avg_filter_t *filter, uint16_t window_size);
extern float moving_avg_filter_update(moving_avg_filter_t *filter, float new_value);


#endif /* MOVING_AVERAGE_FILTER_H_ */
