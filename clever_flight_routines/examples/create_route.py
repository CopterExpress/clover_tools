#!/usr/bin/env python
import threading
import argparse
import rospy
from mavros_msgs.msg import RCIn
from clever_flight_routines import create_route

rospy.init_node('create_route')

add_trigger = threading.Event()
stop_trigger = threading.Event()

add_rc_channel = 2
add_comparison_sign = 1
add_threshold = 1800
stop_rc_channel = 2
stop_comparison_sign = -1
stop_threshold = 1200

def comparison_word(statement):
    if statement > 0:
        return 'upper'
    return 'lower'

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
    
    parser = argparse.ArgumentParser(description="Create flying route by moving drone in space and recording its coordinates by triggering RC pitch stick")
    parser.add_argument('filename', nargs='?', default='route.csv',
                        help="Filename of route csv table. Default is route.csv.")
    parser.add_argument('-f','--frame_id', type=str, default='map',
                        help="Coordinates will be recorded relative to frame_id parameter. Default is map.")
    args = parser.parse_args()

    rospy.loginfo('Set stick on RC channel {} {} than {} us to add point'.format(
                add_rc_channel,comparison_word(add_comparison_sign),add_threshold)) 
    rospy.loginfo('Set stick on RC channel {} {} than {} us to stop creating route.'.format(
                stop_rc_channel,comparison_word(stop_comparison_sign),stop_threshold))

    rospy.Subscriber('mavros/rc/in', RCIn, callback)

    create_route(args.filename, add_trigger, stop_trigger, args.frame_id)

