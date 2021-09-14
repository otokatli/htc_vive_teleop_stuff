#!/usr/bin/env python3

import time
import openvr
from math import sqrt, copysign
import rospy
import tf2_ros
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped

"""
Getting twists and buttons into ROS.

Controller velocities are published as /left_controller_twist and
/right_controller_twist.
Button presses in Joy topics /vive_left /vive_right .

Author: Ozan Tokatli <ozan.tokatli@gmail.com>
"""


def get_controller_ids(vrsys=None):
    if vrsys is None:
        vrsys = openvr.VRSystem()
    else:
        vrsys = vrsys
    left = None
    right = None
    for i in range(openvr.k_unMaxTrackedDeviceCount):
        device_class = vrsys.getTrackedDeviceClass(i)
        if device_class == openvr.TrackedDeviceClass_Controller:
            role = vrsys.getControllerRoleForTrackedDeviceIndex(i)
            if role == openvr.TrackedControllerRole_RightHand:
                right = i
            if role == openvr.TrackedControllerRole_LeftHand:
                left = i
    return left, right


def get_lighthouse_ids(vrsys=None):
    if vrsys is None:
        vrsys = openvr.VRSystem()
    else:
        vrsys = vrsys
        lighthouse_ids = []
    for i in range(openvr.k_unMaxTrackedDeviceCount):
        device_class = vrsys.getTrackedDeviceClass(i)
        if device_class == openvr.TrackedDeviceClass_TrackingReference:
            lighthouse_ids.append(i)
    return lighthouse_ids


def get_generic_tracker_ids(vrsys=None):
    if vrsys is None:
        vrsys = openvr.VRSystem()
    else:
        vrsys = vrsys
        generic_tracker_ids = []
    for i in range(openvr.k_unMaxTrackedDeviceCount):
        device_class = vrsys.getTrackedDeviceClass(i)
        if device_class == openvr.TrackedDeviceClass_GenericTracker:
            generic_tracker_ids.append(i)
    return generic_tracker_ids


def from_vector_to_twist(lin_vel, ang_vel, stamp, frame_id):
    twist = TwistStamped()
    
    twist.header.stamp = stamp
    twist.header.frame_id = frame_id
    
    twist.twist.linear.x = lin_vel[0]
    twist.twist.linear.y = lin_vel[1]
    twist.twist.linear.z = lin_vel[2]
    
    twist.twist.angular.x = ang_vel[0]
    twist.twist.angular.y = ang_vel[1]
    twist.twist.angular.z = ang_vel[2]
    
    return twist


def from_controller_to_joy(prev_unPacketNum,
                           pControllerState,
                           stamp,
                           frame_id):
    # docs: https://github.com/ValveSoftware/openvr/wiki/IVRSystem::GetControllerState

    d = {}
    d['unPacketNum'] = pControllerState.unPacketNum
    # on trigger .y is always 0.0 says the docs
    d['trigger'] = pControllerState.rAxis[1].x
    # 0.0 on trigger is fully released
    # -1.0 to 1.0 on joystick and trackpads
    d['trackpad_x'] = pControllerState.rAxis[0].x
    d['trackpad_y'] = pControllerState.rAxis[0].y
    # These are published and always 0.0
    # for i in range(2, 5):
    #     d['unknowns_' + str(i) + '_x'] = pControllerState.rAxis[i].x
    #     d['unknowns_' + str(i) + '_y'] = pControllerState.rAxis[i].y
    d['ulButtonPressed'] = pControllerState.ulButtonPressed
    d['ulButtonTouched'] = pControllerState.ulButtonTouched
    # To make easier to understand what is going on
    # Second bit marks menu button
    d['menu_button'] = bool(pControllerState.ulButtonPressed >> 1 & 1)
    # 32 bit marks trackpad
    d['trackpad_pressed'] = bool(pControllerState.ulButtonPressed >> 32 & 1)
    d['trackpad_touched'] = bool(pControllerState.ulButtonTouched >> 32 & 1)
    # third bit marks grip button
    d['grip_button'] = bool(pControllerState.ulButtonPressed >> 2 & 1)
    # System button can't be read, if you press it
    # the controllers stop reporting

    j = Joy()
    j.header.frame_id = frame_id
    j.header.stamp = stamp
    # Axes
    # Trigger, Trackpad X, Trackpad Y
    j.axes = [
        d['trigger'],
        d['trackpad_x'],
        d['trackpad_y']
    ]
    # Buttons
    # Trigger, Trackpad touched, trackpad pressed, menu, grip
    j.buttons = [
        d['trigger'] == 1.0,
        d['trackpad_touched'],
        d['trackpad_pressed'],
        d['menu_button'],
        d['grip_button']
    ]
    new_msg = prev_unPacketNum != d['unPacketNum']
    return new_msg, j


if __name__ == '__main__':
    print("===========================")
    print("Initializing OpenVR...")
    retries = 0
    max_init_retries = 4
    while retries < max_init_retries:
        try:
            openvr.init(openvr.VRApplication_Scene)
            break
        except openvr.OpenVRError as e:
            print("Error when initializing OpenVR (try {} / {})".format(
                  retries + 1, max_init_retries))
            print(e)
            retries += 1
            time.sleep(2.0)
    else:
        print("Could not initialize OpenVR, aborting.")
        print("Make sure the system is correctly plugged, you can also try")
        print("to do:")
        print("killall -9 vrcompositor vrmonitor vrdashboard")
        print("Before running this program again.")
        exit(0)

    print("Success!")
    print("===========================")

    print("VRSystem...")
    vrsystem = openvr.VRSystem()

    left_id, right_id = None, None
    print("===========================")
    print("Waiting for controllers...")
    try:
        while left_id is None or right_id is None:
            left_id, right_id = get_controller_ids(vrsystem)
            if left_id and right_id:
                break
            print("Waiting for controllers...")
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("Control+C pressed, shutting down...")
        openvr.shutdown()

    print("Left controller ID: " + str(left_id))
    print("Right controller ID: " + str(right_id))
    print("===========================")

    lighthouse_ids = get_lighthouse_ids(vrsystem)
    print("Lighthouse IDs: " + str(lighthouse_ids))

    generic_tracker_ids = get_generic_tracker_ids(vrsystem)
    print("Generic tracker IDs:" + str(generic_tracker_ids))

    poses_t = openvr.TrackedDevicePose_t * openvr.k_unMaxTrackedDeviceCount
    poses = poses_t()

    print("Initializing ROS...")
    rospy.init_node('HTCViveROS')

    joy_left_pub = rospy.Publisher('vive_left', Joy, queue_size=1)
    prev_unPacketNum_left = 0
    joy_right_pub = rospy.Publisher('vive_right', Joy, queue_size=1)
    prev_unPacketNum_right = 0
    left_controller_twist_pub = rospy.Publisher('/left_controller_twist',
                                                TwistStamped, queue_size=1)
    right_controller_twist_pub = rospy.Publisher('/right_controller_twist',
                                                 TwistStamped, queue_size=1)
    # Give a bit of time to initialize...
    rospy.sleep(3.0)
    print("Running!")

    # Vibration topic for each controller
    def vibration_cb(data, controller_id):
        # strength 0-3999, data contains a float expected
        # to be in between 0.0 and 1.0
        if data.data > 1.0:
            strength = 3999
        elif data.data < 0.0:
            strength = 0
        else:
            strength = int(data.data * 3999)
        vrsystem.triggerHapticPulse(controller_id, 0, strength)

    vib_left = rospy.Subscriber('vive_left_vibration', Float64, vibration_cb,
                                callback_args=left_id, queue_size=1)

    vib_right = rospy.Subscriber('vive_right_vibration', Float64, vibration_cb,
                                 callback_args=right_id, queue_size=1)

    # Internet says the tracking can be up until 250Hz
    r = rospy.Rate(250)
    while not rospy.is_shutdown():
        r.sleep()
        poses = poses_t()
        vrsystem.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseSeated,
            0,
            poses)

        now = rospy.Time.now()
        # twists = []
        
        # Hmd is always 0
        linear_vel = poses[0].vVelocity
        angular_vel = poses[0].vAngularVelocity
        hmd_twist = from_vector_to_twist(linear_vel, angular_vel, now, "hmd")
        # twists.append(hmd_twist)

        for idx, _id in enumerate(lighthouse_ids):
            linear_vel = poses[_id].vVelocity
            angular_vel = poses[_id].vAngularVelocity
            lhouse_twist = from_vector_to_twist(linear_vel, angular_vel, now,
                                                "lighthouse_" + str(idx))                                   
            # twists.append(lhouse_twist)

        if left_id:
            linear_vel = poses[left_id].vVelocity
            angular_vel = poses[left_id].vAngularVelocity
            left_twist = from_vector_to_twist(linear_vel, angular_vel, now,
                                              "left_controller")                                   
            # twists.append(left_twist)
            result, pControllerState = vrsystem.getControllerState(left_id)
            new_msg, j = from_controller_to_joy(prev_unPacketNum_left,
                                                pControllerState,
                                                now,
                                                "left_controller")
            prev_unPacketNum_left = pControllerState.unPacketNum
            if new_msg:
                joy_left_pub.publish(j)

        if right_id:
            linear_vel = poses[right_id].vVelocity
            angular_vel = poses[right_id].vAngularVelocity
            right_twist = from_vector_to_twist(linear_vel, angular_vel, now,
                                               "right_controller")
            # twists.append(right_twist)
            result, pControllerState = vrsystem.getControllerState(right_id)
            new_msg, j = from_controller_to_joy(prev_unPacketNum_right,
                                                pControllerState,
                                                now,
                                                "right_controller")
            prev_unPacketNum_right = pControllerState.unPacketNum
            if new_msg:
                joy_right_pub.publish(j)

        for idx, _id in enumerate(generic_tracker_ids):
            linear_vel = poses[_id].vVelocity
            angular_vel = poses[_id].vAngularVelocity
            gen_track_twist = from_vector_to_twist(linear_vel, angular_vel, now,
                                                   "generic_tracker_" + str(idx))
            # twists.append(gen_track_twist)

        # Publish twists
        left_controller_twist_pub.publish(left_twist)
        right_controller_twist_pub.publish(right_twist)
        # br.sendTransform(transforms)

    openvr.shutdown()
