#!/usr/bin/env python
import sys
import math
import time
import logging
import threading
import rospy
from clever import srv
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
from std_srvs.srv import Trigger

logger = logging.getLogger(__name__)

# create proxies to services
navigate = rospy.ServiceProxy('/navigate', srv.Navigate)
set_position = rospy.ServiceProxy('/set_position', srv.SetPosition)
set_rates = rospy.ServiceProxy('/set_rates', srv.SetRates)
set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
landing = rospy.ServiceProxy('/land', Trigger)

logger.info("Proxy services inited")

# globals
FREQUENCY = 40              # hz
TOLERANCE = 0.2             # m
SPEED = 1.0                 # m/s
TAKEOFF_SPEED = 1.0         # m/s
TAKEOFF_HEIGHT = 1.0        # m
TIMEOUT = 5.0               # s
TIMEOUT_ARMED = 2.0         # s
FLIP_MIN_HEIGHT = 2.0       # m
LOCAL_FRAME_ID = 'map'
COPTER_FRAME_ID = 'body'
INTERRUPTER = threading.Event()

# Get distance betwwen 2 points
def get_distance(x1, y1, z1, x2, y2, z2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)

# Takeoff to specified height
def takeoff(height=TAKEOFF_HEIGHT, speed=TAKEOFF_SPEED, tolerance=TOLERANCE, frame_id=LOCAL_FRAME_ID):
    logger.info("Takeoff started...")
    rate = rospy.Rate(FREQUENCY)
    start = get_telemetry(frame_id=frame_id)
    result = navigate(z=height, speed=speed, yaw=float('nan'), frame_id=COPTER_FRAME_ID, auto_arm=True)
    logger.info(result)
    while True:
        telem = get_telemetry(frame_id=frame_id)
        climb = abs(telem.z - start.z)
        logger.info("Takeoff to {:.2f} of {:.2f} meters".format(climb, height))
        if abs(climb-height) < tolerance:
            break
        rate.sleep()
    logger.info("Takeoff succeeded!")

# Reach specified point. Copter needs to be armed.
def reach_point(x, y, z, yaw=float('nan'), speed=SPEED, tolerance=TOLERANCE, frame_id=LOCAL_FRAME_ID):
    logger.info("Reaching point...")
    rate = rospy.Rate(FREQUENCY)
    result = navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, tolerance=tolerance, frame_id=frame_id)
    logger.info(result)
    while True:
        telem = get_telemetry(frame_id=frame_id)
        if get_distance(x, y, z, telem.x, telem.y, telem.z) < tolerance:
            break
        rate.sleep()