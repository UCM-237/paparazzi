/*
 * Copyright (C) 2011 Gautier Hattenberger
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
 * @file peripherals/hmc58xx_regs.h
 * Register defs for Honeywell HMC5843 and HMC5883 magnetometers.
 */

#ifndef QMC5883_REGS_H
#define QMC5883_REGS_H

/* default I2C address */
#define QMC5883_ADDR 0x0D

/* Registers */
//DANI

#define QMC5883_REG_DATXL  0x00
#define QMC5883_REG_DATXM  0x01
#define QMC5883_REG_DATYL  0x02
#define QMC5883_REG_DATYM  0x03
#define QMC5883_REG_DATZL  0x04
#define QMC5883_REG_DATZM  0x05


#endif // QMC5883_REGS_H
