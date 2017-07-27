#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix

import sys

if len(sys.argv) < 4:
    print('usage: {} lat lng alt'.format(sys.argv[0]))
    exit()

lat = float(sys.argv[1])
lng = float(sys.argv[2])
alt = float(sys.argv[3])

rospy.init_node("target_gps_broadcaster")
pub = rospy.Publisher('target_gps_fix', NavSatFix, queue_size=1)

rospy.sleep(1)

fix = NavSatFix()
fix.header.stamp = rospy.Time.now()
fix.header.frame_id = 'gps_frame'

fix.latitude = lat
fix.longitude = lng
fix.altitude = alt
fix.status.status = 0
fix.status.service = 1

pub.publish(fix)

