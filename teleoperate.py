#!/usr/bin/env python

"""
Human Robot Interation, Design Project II
"""
import argparse
import sys

import rospy
import tf2_ros
import math
import numpy as np

import baxter_interface

from baxter_interface import CHECK_VERSION

BASE_FRAME = 'camera_depth_frame'
FRAMES = [
        'torso',
        'left_shoulder',
        'left_elbow',
        'left_hand',
        'right_shoulder',
        'right_elbow',
        'right_hand',
        ]


def try_float(x):
    try:
        return float(x)
    except ValueError:
        return None


def get_joint_angles(user, tfBuffer):
    """

    @param line: the line described in a list to process
    @param names: joint name keys
    """
    joint_angles = dict()
    joint_angles['left'] = dict()
    joint_angles['right'] = dict()

    frame_positions = get_frame_positions(user, tfBuffer)
    print frame_positions
    # check if frame_positions is not none

    # do the math to find joint angles
    joint_angles['left']['left_s0'] = 0.0 
    joint_angles['left']['left_s1'] = 0.0
    joint_angles['left']['left_e0'] = 0.0
    joint_angles['left']['left_e1'] = 0.0
    joint_angles['left']['left_w0'] = 0.0
    joint_angles['left']['left_w1'] = 0.0
    joint_angles['left']['left_w2'] = 0.0
    joint_angles['right']['right_s0'] = 0.0
    joint_angles['right']['right_s1'] = 0.0
    joint_angles['right']['right_e0'] = 0.0
    joint_angles['right']['right_e1'] = 0.0
    joint_angles['right']['right_w0'] = 0.0
    joint_angles['right']['right_w1'] = 0.0
    joint_angles['right']['right_w2'] = 0.0

    return joint_angles

def get_frame_positions(user, tfBuffer):
    frame_positions = dict()
    try:
        for frame in FRAMES:
            transformation= tfBuffer.lookup_transform(BASE_FRAME, "%s_%d" % (frame, user), rospy.Time())
            translation = transformation.transform.translation
            pos = np.array([translation.x, translation.y, translation.z])
            frame_positions[frame] = pos
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e: 
        print "Problem with kinect tracking, teleoperation paused."
        print e
        return None
    return frame_positions

def teleoperate(rate, user):
    """
    Teleoperates the robot based on tf2 frames.

    @param rate: rate at which to sample joint positions in ms

    """
    rate = rospy.Rate(rate)
    # TODO: make these attributes of a class
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)


    # resetting the grippers 
    if grip_left.error():
        grip_left.reset()
    if grip_right.error():
        grip_right.reset()
    if (not grip_left.calibrated() and
        grip_left.type() != 'custom'):
        grip_left.calibrate()
    if (not grip_right.calibrated() and
        grip_right.type() != 'custom'):
        grip_right.calibrate()


    while not rospy.is_shutdown():
        joint_angles = get_joint_angles(user, tfBuffer)

        # left.move_to_joint_positions(joint_angles['left'])
        # right.move_to_joint_positions(joint_angles['right'])

        rate.sleep()
    return True


def main():
    """
    Note: This version of simply drives the joints towards the next position at
    each time stamp. Because it uses Position Control it will not attempt to
    adjust the movement speed to hit set points "on time".
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.add_argument(
        '-r', '--rate', type=int, default=1000,
        help='rate to sample the joint positions'
    )

    parser.add_argument(
        '-u', '--user', type=int, default=1,
        help='kinect user number to use'
    )
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("teleoperation")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    def clean_shutdown():
        print("\nExiting...")
        if not init_state:
            print("Disabling robot...")
            rs.disable()
    rospy.on_shutdown(clean_shutdown)

    print("Enabling robot... ")
    rs.enable()

    teleoperate(args.rate, args.user)

if __name__ == '__main__':
    main()
