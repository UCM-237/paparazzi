#!/usr/bin/env python

import sys
from os import path, getenv
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python")

from ivy.std_api import *
import pprzlink.ivy

import pprzlink.messages_xml_map as messages_xml_map
import pprzlink.message as message
import time

# Function called when a PING message is received
def recv_callback(ac_id, pprzMsg):
    # Print the message and the sender id
    print ("Received message %s from %s" % (pprzMsg,ac_id))
    # Send back a PONG message
    ivy.send(message.PprzMessage("datalink", "NUM_WAYPOINT_MOVED_DATALINK"))

# Creation of the ivy interface
ivy = pprzlink.ivy.IvyMessagesInterface(
            agent_name="PprzlinkIvyTutorial",   # Ivy agent name
            start_ivy=False,                    # Do not start the ivy bus now
            ivy_bus="127.255.255.255:2010")     # address of the ivy bus

try:
    # starts the ivy interface
    ivy.start()

    # Subscribe to PING messages and sets recv_callback as the callback function.
    ivy.subscribe(recv_callback,message.PprzMessage("telemetry", "NUM_WP_MOVED"), sender_id= 161, receiver_id= ac_id)

    # Wait untill ^C is pressed
    while True:
        time.sleep(5)
except KeyboardInterrupt:
    ivy.shutdown()
