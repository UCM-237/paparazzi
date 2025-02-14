/*
 * Copyright (C) 2025 Alejandro Rochas <alrochas@ucm.es>
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

/** @file modules/lidar/sweep_lidar.h
 *  @brief driver for the Scanse Sweep lidar
 *
 */


// TODO esto esta sin probar, pero lo guardo aqui

#ifndef SWEEP_LIDAR_H
#define SWEEP_LIDAR_H

struct SweepLidar {
    struct uart_periph *device;
    uint8_t parse_status;
    float distance;
    float angle;
    uint8_t signal_strength;
};

extern struct SweepLidar sweep_lidar;

extern void sweep_init(void);
extern void sweep_event(void);
extern void sweep_parse(uint8_t byte);

#endif