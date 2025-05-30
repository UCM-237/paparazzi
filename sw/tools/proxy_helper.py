#!/usr/bin/env python
#
# Copyright (C) 2022 Jesus Chacon Sombria <jeschaco@ucm.es>
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
import xml.etree.ElementTree as ET
import os
import subprocess
import signal

PAPARAZZI_HOME = os.getenv('PAPARAZZI_HOME') or '/'
CONF_FILE = os.path.join(PAPARAZZI_HOME, 'conf/airframes/UCM/conf.xml')
WORK_DIR = os.path.join(PAPARAZZI_HOME, 'sw/tools')

def load_aircrafts(xml_path):
  try:
    tree = ET.parse(xml_path)
    root = tree.getroot()

    if root.tag != "conf":
      raise ValueError(f"Unexpected root tag: {root.tag}")

    aircraft_list = []

    for aircraft in root.findall("aircraft"):
      props = aircraft.attrib
      aircraft_list.append(props)

    return aircraft_list

  except ET.ParseError as e:
    print(f"XML parse error: {e}")
  except FileNotFoundError:
    print(f"File not found: {xml_path}")
  except Exception as e:
    print(f"Unexpected error: {e}")


def load_targets(xml_path):
  try:
    tree = ET.parse(xml_path)
    root = tree.getroot()
    firmware = root.find("firmware")
    if firmware is None:
      raise ValueError("Node <firmware> not found in <airframe>")

    for target in firmware.findall("target"):
      if target.get("name") == "nps":
        target_data = {
          "name": target.get("name"),
          "board": target.get("board"),
          "modules": [],
          "defines": {},
          "configures": {}
        }

        for child in target:
          if child.tag == "module":
            target_data["modules"].append({
              "name": child.get("name"),
              "type": child.get("type")
            })
          elif child.tag == "define":
            target_data["defines"][child.get("name")] = child.get("value")
          elif child.tag == "configure":
              target_data["configures"][child.get("name")] = child.get("value")
        return target_data

    return None

  except ET.ParseError as e:
    print(f"XML parse error: {e}")
  except FileNotFoundError:
    print(f"File not found: {xml_path}")
  except Exception as e:
    print(f"Unexpected error: {e}")


def generate_proxy_command(aircrafts: dict) -> str:
  if not aircrafts:
    return

  ports = [aircraft_to_ports(k) for k in aircrafts if 'nps.xml' in k['settings']]
  ids = [p['id'] for p in ports]
  pi = [p['pi'] for p in ports]
  po = [p['po'] for p in ports]
  return [
    "python3",
    f"{os.path.join(WORK_DIR, 'proxy_sim.py')}",
    "-ids", f"{','.join(ids)}",
    "-pi",  f"{','.join(pi)}",
    "-po",  f"{','.join(po)}"
  ]

def aircraft_to_ports(aircraft: dict) -> dict:
  target = load_targets(f"{PAPARAZZI_HOME}/conf/{aircraft['airframe']}")
  if not 'configures' in target:
      raise ValueError('<configures> not found in <target>')

  return {'id': aircraft['ac_id'], 'pi': target['configures']['MODEM_PORT_IN'], 'po': target['configures']['MODEM_PORT_OUT']}


## Start proxy
if __name__ == "__main__":
    aircrafts = load_aircrafts(CONF_FILE)
    cmd = generate_proxy_command(aircrafts)
    process = subprocess.Popen(cmd)
    signal.signal(signal.SIGTERM, lambda signum, frame: process.terminate())
    process.wait()

