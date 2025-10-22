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
 * @file "filters/moving_average_filter.c"
 *
 * Moving Average Filter implementation (used in INS SLAM EKF for the accelerations)
 */


#include "string.h"
#include "filters/moving_average_filter.h"
#include "math/pprz_algebra_float.h"


// Iniciate the moving average filter
void moving_avg_filter_init(moving_avg_filter_t *filter, uint16_t window_size) {
  filter->window_size = window_size;
  filter->index = 0;
  filter->initialized = false;
  filter->sum = 0.0f;
  
  // Buffer to 0
  for(uint16_t i = 0; i < window_size; i++) {
    filter->buffer[i] = 0.0f;
  }
}


// Update the filter
float moving_avg_filter_update(moving_avg_filter_t *filter, float new_value) {
  // If the buffer is not full yet
  if (!filter->initialized) {
    for(uint16_t i = 0; i < filter->window_size; i++) {
      filter->buffer[i] = new_value;
    }
    filter->sum = new_value * filter->window_size;
    filter->initialized = true;
    return new_value;
  }
  
  // Update sum
  float old_value = filter->buffer[filter->index];
  filter->sum = filter->sum - old_value + new_value;
  
  filter->buffer[filter->index] = new_value;
  filter->index = (filter->index + 1) % filter->window_size;
  
  return filter->sum / filter->window_size;
}







