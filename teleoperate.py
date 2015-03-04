#!/usr/bin/env python

"""
Human Robot Interation, Design Project II
"""
import argparse
import sys

import rospy
import tf2_ros
import math

import baxter_interface

from baxter_interface import CHECK_VERSION


def try_float(x):
    try:
        return float(x)
    except ValueError:
        return None


def get_joint_angles(user):
    """
    Cleans a single line of recorded joint positions

    @param line: the line described in a list to process
    @param names: joint name keys
    """
    joint_angles = dict()
    joint_angles['left'] = dict()
    joint_angles['right'] = dict()
    try:
        joint_angles['left']['left_s0'] = get_frame_rotation(user, '')
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

        trans = tfBuffer.lookup_transform(BASE_FRAME, "%s_%d" % (frame, 1), rospy.Time())
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e: 
        print e

    return joint_angles

def get_frame_rotation(user, frame):
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    # trans = tfBuffer.lookup_transform(BASE_FRAME, "%s_%d" % (frame, user), rospy.Time())
    return 0.0

def teleoperate(rate, user):
    """
    Teleoperates the robot based on tf2 frames.

    @param rate: rate at which to sample joint positions in ms

    """
    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    rate = rospy.Rate(rate)


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
        joint_angles = get_joint_angles(user)

        left.move_to_joint_positions(joint_angles['left'])
        right.move_to_joint_positions(joint_angles['right'])

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
        '-r', '--rate', type=int, default=1,
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
        print("\nExiting example...")
        if not init_state:
            print("Disabling robot...")
            rs.disable()
    rospy.on_shutdown(clean_shutdown)

    print("Enabling robot... ")
    rs.enable()

    teleoperate_robot(args.rate, args.user)

if __name__ == '__main__':
    main()
