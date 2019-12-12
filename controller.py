#!/usr/bin/env python

import time
import pprint
import openvr
import numpy
import sys
import math

X_ZERO = -0.0832
Y_ZERO =  1.0645

class Controller:
    @staticmethod
    def convert_steam_vr_matrix(pose):
        return numpy.array((
            (pose[0][0], pose[1][0], pose[2][0], 0.0),
            (pose[0][1], pose[1][1], pose[2][1], 0.0),
            (pose[0][2], pose[1][2], pose[2][2], 0.0),
            (pose[0][3], pose[1][3], pose[2][3], 1.0),
        ), dtype=numpy.float32)

    @staticmethod
    def get_zeros(pose):
        return (pose[0][3], pose[1][3])

    @staticmethod
    def get_alden_angle(pose):
        x = pose[0][3] - X_ZERO
        y = pose[1][3] - Y_ZERO
        print(x, y)
        return 90 - numpy.arctan(y/x)*180.0/3.1415

    @staticmethod
    def from_controller_state_to_dict(pControllerState):
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
        for i in range(2, 5):
            d['unknowns_' + str(i) + '_x'] = pControllerState.rAxis[i].x
            d['unknowns_' + str(i) + '_y'] = pControllerState.rAxis[i].y
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
        return d
    

    def get_controller_ids(self, vrsys=None):
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

    def get_digital_action_state(self, action, device_path=None):
        """
        Purpose: Returns true if the action is active and its state is true
        """
        action_data = openvr.VRInput().getDigitalActionData(action, openvr.k_ulInvalidInputValueHandle)
        if device_path is not None:
            if action_data.bActive:
                origin_info = openvr.VRInput().getOriginTrackedDeviceInfo(action_data.activeOrigin)
                device_path = origin_info.devicePath
        return action_data.bActive and action_data.bState, device_path

    def __init__(self):
        self.left = None
        self.right = None

    def setup(self):
        max_init_retries = 4
        retries = 0
        print("===========================")
        print("Initializing OpenVR...")
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
        self.vrsystem = openvr.VRSystem()
        openvr.VRInput().setActionManifestPath("C:\\Users\\alexweiss\\Desktop\\actions.json")


    def get_data(self):
        action_sets = (openvr.VRActiveActionSet_t * 1)()
        action_set = action_sets[0]
        action_set.ulActionSet = openvr.VRInput().getActionSetHandle('/actions/demo')
        openvr.VRInput().updateActionState(action_sets)
        
        pose_data = openvr.VRInput().getPoseActionDataForNextFrame(
                openvr.VRInput().getActionHandle('/actions/demo/in/Hand_Right'),
                openvr.TrackingUniverseStanding,
                openvr.k_ulInvalidInputValueHandle,
            )
        if not pose_data.bActive:
            print("b")
            return
        if not pose_data.pose.bPoseIsValid:
            print("a")
            return
        #print(get_zeros(pose_data.pose.mDeviceToAbsoluteTracking))
        print(self.get_alden_angle(pose_data.pose.mDeviceToAbsoluteTracking))

        active = self.get_digital_action_state(openvr.VRInput().getActionHandle('/actions/demo/in/hidecubes'))[0]
        print(active)
        print()



if __name__ == '__main__':
    x = Controller()
    x.setup()

    try:
        while True:
            x.get_data()
            time.sleep(0.2)

    except KeyboardInterrupt:
        print("Control+C pressed, shutting down...")
        openvr.shutdown()
