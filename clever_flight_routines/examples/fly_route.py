#!/usr/bin/env python
import argparse
import rospy
import math
from mavros_msgs.msg import RCIn
from clever_flight_routines import read_route, takeoff, fly_route, land

rospy.init_node('fly_route')

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description="Fly route from the csv table with x y z values")
    parser.add_argument('filename', nargs='?', default='route.csv',
                        help="Filename of the route csv table")
    parser.add_argument('-r', '--repeat', action='store_const', const=True, default=False,
                        help="Program will repeat the route infinite if you use this arg.")
    parser.add_argument('-z', '--height', type=float, default=1.0,
                        help="Height of flight. Default is 1 m. If height is 'nan' then the drone will takeoff to 1 m and fly on the route point heights.")
    parser.add_argument('-f','--frame_id', type=str, default='map',
                        help="Coordinates will be used relative to frame_id parameter. Default is map.")
    parser.add_argument('-s','--speed', type=float, default=1,
                        help="Copter speed. Default is 1 m/s.")
    args = parser.parse_args()

    # Try to open csv file
    try:
        route = read_route(args.filename)
    except Exception:
        rospy.logfatal("Can't read the route file")
        quit()

    rospy.loginfo('Route file was read') 

    # Fly route 
    if math.isnan(args.height):
        takeoff()
    else:     
        takeoff(args.height)
    fly_route(route=route, z=args.height, speed=args.speed, frame_id=args.frame_id)
    while args.repeat:
        fly_route(route=route, z=args.height, speed=args.speed, frame_id=args.frame_id)
    land()