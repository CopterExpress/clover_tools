#!/usr/bin/env python3

# Converts mavros/imu/mag from sensor_msgs/MagneticField to geometry_msgs/TwistStamped
# for displaying in rviz
# https://jsk-docs.readthedocs.io/en/latest/jsk_visualization/doc/jsk_rviz_plugins/index.html

import rospy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import MagneticField


rospy.init_node('mag_to_vector')
vec = TwistStamped()
vec_pub = rospy.Publisher('mag_vec', TwistStamped, queue_size=1)


def mag_cb(msg):
    vec.header = msg.header
    vec.twist.linear = msg.magnetic_field
    vec_pub.publish(vec)


rospy.Subscriber('mavros/imu/mag', MagneticField, mag_cb)
rospy.spin()
