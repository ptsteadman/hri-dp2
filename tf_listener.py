#!/usr/bin/env python  
import rospy

import math
import tf2_ros

BASE_FRAME = 'camera_link'
FRAMES = [
        'head',
        'neck',
        'torso',
        'left_shoulder',
        'left_elbow',
        'left_hand',
        'left_hip',
        'left_knee',
        'left_foot',
        'right_shoulder',
        'right_elbow',
        'right_hand',
        'right_hip',
        'right_knee',
        'right_foot',
        ]

if __name__ == '__main__':
    rospy.init_node('camera_link')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            frames = []
            for frame in FRAMES:
                trans = tfBuffer.lookup_transform(BASE_FRAME, "%s_%d" % (frame, 3), rospy.Time())
                frames.append(trans) 
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e: 
            print e
        print frames
        rate.sleep()
