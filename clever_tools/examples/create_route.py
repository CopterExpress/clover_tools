#!/usr/bin/env python
import threading
import argparse
import rospy
from mavros_msgs.msg import RCIn
from flight_routines import *

rospy.init_node('create_route')

add_trigger = threading.Event()
stop_trigger = threading.Event()

add_rc_channel = 5
add_comparison_sign = 1
add_threshold = 1500
stop_rc_channel = 2
stop_comparison_sign = -1
stop_threshold = 1250

def callback(data):
    global state
    if data.channels[add_rc_channel-1]*add_comparison_sign > add_threshold*add_comparison_sign:
        add_trigger.set()
    else:
        add_trigger.clear()
    if data.channels[stop_rc_channel-1]*stop_comparison_sign > stop_threshold*stop_comparison_sign:
        stop_trigger.set()
    else:
        stop_trigger.clear()

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser()
    parser.add_argument('filename', nargs='?', default='route.csv',
                        help="Filename of route csv table")
    args = parser.parse_args()

    rospy.Subscriber('mavros/rc/in', RCIn, callback)

    create_route(args.filename, add_trigger, stop_trigger)

