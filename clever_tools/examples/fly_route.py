#!/usr/bin/env python
import threading
import argparse
import rospy
from mavros_msgs.msg import RCIn
from flight_routines import *

rospy.init_node('fly_route')

# copter parameters

speed = 1
z = 1
frame = 'map'

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser()
    parser.add_argument('filename', nargs='?', default='route.csv',
                        help="Filename of the route csv table")
    parser.add_argument('--repeat', action='store_const', const=True, default=False,
                        help="Repeat the route infinite if use this arg")
    args = parser.parse_args()

    try:
        route = read_route(args.filename)
    except Exception:
        rospy.logfatal("Can't read the route file")
        quit()

    
        
    takeoff(z)
    fly_route(route=route, z=z, speed=speed, frame_id=frame)
    while args.repeat:
        fly_route(route=route, z=z, speed=speed, frame_id=frame)
    land()