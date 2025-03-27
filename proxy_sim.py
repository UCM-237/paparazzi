#!/usr/bin/env python
#
# Copyright (C) 2022 Jesús Bautista Villar <jesbauti20@gmail.com>
#
# This file is part of paparazzi.
#
# paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi; see the file COPYING.  If not, see
# <http://www.gnu.org/licenses/>.
#


'''\
Script to generate an easy call for pprzlink_proxy.
	-> python3 proxy_sim.py -ids 5,6 -pi 4248,4250 -po 4249,4251
	-> python3 proxy_sim.py -ids 5,6,200 -pi 4248,4250,4252 -po 4249,4251,4253
'''

import os
from time import sleep

PAPARAZZI_HOME = os.getenv("PAPARAZZI_HOME")

PAPARAZZI_PROXY = PAPARAZZI_HOME + "/sw/ground_segment/tmtc/pprzlink_proxy.py"
AIRFRAME_DIR = "airframes/UGR/"
AIRFRAME_DIR_LONG = PAPARAZZI_HOME + "/conf/" + AIRFRAME_DIR

INIT_OUT_PORT = 4244
INIT_IN_PORT = 4245
INIT_AC_ID = 1 # should be more than 0 and less than 255

NUM_AGENTS = 3

# -------------------------------------------------------------------------------------------------------------------------------

if __name__ == '__main__':
	import argparse

	parser = argparse.ArgumentParser(
		description= '''IVY bus proxy for paparazzi multi-robot simulations (it calls pprzlink_proxy)\n
			Examples:
			\t-> python3 proxy_sim.py -ids 1,2,3 -pi 4245,4247,4249 -po 4244,4246,4248
			\t-> python3 proxy_sim.py -ids 5,6 -pi 4249,4251 -po 4248,4250
			\t-> python3 proxy_sim.py -ids 5,6,200 -pi 4248,4250,4252 -po 4249,4251,4253
			''')
	parser.add_argument('-v', '--v', dest='verbose', action='store_true', help="Pprz_proxy verbose argument")
	parser.add_argument('-ids', '--ids', dest='ids', default=None, help="AC IDs")
	parser.add_argument('-pi', '--pi', dest='in_ports', default=None, help="AC ports in")
	parser.add_argument('-po', '--po', dest='out_ports', default=None, help="AC ports out")
	args = parser.parse_args()

	verbose = args.verbose
	
	ids = args.ids
	in_ports = args.in_ports
	out_ports = args.out_ports
	
	# Generate the pprzlink_proxy args
	pprz_proxy_args = " "
	if ids is not None and in_ports is not None and out_ports is not None:
		ids = ids.split(",")
		in_ports = in_ports.split(",")
		out_ports = out_ports.split(",")

		for i in range(len(ids)):
			ac_id = ids[i]
			out_port = out_ports[i]
			in_port = in_ports[i]

			pprz_proxy_args += "--ac=" + str(ac_id) + ":" + str(out_port) + ":" + str(in_port) + " "
	else:
		for n in range(NUM_AGENTS):
			ac_id = INIT_AC_ID + n
			out_port = INIT_OUT_PORT + 2*n
			in_port = INIT_IN_PORT + 2*n

			pprz_proxy_args += "--ac=" + str(ac_id) + ":" + str(out_port) + ":" + str(in_port) + " "

	sleep(1)

	try:
		# Execute pprzlink_proxy with the generated args
		print(" ------------------------ ")
		print("Custom pprzlink_proxy arguments:\n\t" + pprz_proxy_args + "\n")
		if verbose:
			os.system("python3 " + PAPARAZZI_PROXY + pprz_proxy_args + "-v")
		else:
			os.system("python3 " + PAPARAZZI_PROXY + pprz_proxy_args)

	except (KeyboardInterrupt, SystemExit):
		pass
		
		

	
	
	


		


		
	

    
