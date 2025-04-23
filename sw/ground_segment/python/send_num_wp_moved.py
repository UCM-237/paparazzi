#!/usr/bin/env python

from __future__ import print_function

import sys
from os import path, getenv
import argparse
from time import sleep

# if PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python")

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage

# parse args
parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument('-ac', '--ac_id', dest='ac_id', type=int, help='Aircraft ID to save the RSSI information to')
parser.add_argument('-b', '--ivy_bus', dest='ivy_bus', help="Ivy bus address and port")
parser.add_argument('num', type=int, help="Num of waypoints sended")
args = parser.parse_args()


class SendNumWp(object):
    def __init__(self, verbose=False):
        self.verbose = verbose
        self._interface = IvyMessagesInterface("NumWaypointMoved")

    def shutdown(self):
        print("Shutting down ivy interface...")
        self._interface.shutdown()

    def __del__(self):
        self.shutdown()

    def num_moved_waypoints(self, num):
        msg = PprzMessage("datalink", "NUM_WAYPOINT_MOVED_DATALINK")
        msg['num'] = num

        print("Sending message: %s" % msg)
        self._interface.send(msg)


if __name__ == '__main__':

    num = args.num
    print(f"num código pyhton {num}")
    try:
        
        snw = SendNumWp()
        # sleep shortly in order to make sure Ivy is up, then message sent before shutting down again
        sleep(0.1)      
        snw.num_moved_waypoints(num=num)
        sleep(0.1)
    except KeyboardInterrupt:
        print("Stopping on request")
    snw.shutdown()
