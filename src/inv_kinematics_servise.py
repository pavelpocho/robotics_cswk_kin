#! /usr/bin/env python3

import rospy
import math

from std_msgs.msg import Float64
from geometry_msgs.msg import Point

from cswk_kin.srv import IKinMsg, IKinMsgRequest, IKinMsgResponse

### DO NOT RUN THIS ON THE ROBOT!!!!! TEST IT FIRST TO HAVE CORRECT ANGLE OFFSETS


class InvKin:
    def __init__(self):
        self._as = rospy.Service("/inv_kin", IKinMsg, self.calculateJointPositions)
        rospy.spin()

    def calculateJointPositions(self, msg: IKinMsgRequest) -> IKinMsgResponse:
        pos = msg.position
        x = pos.x
        y = pos.y
        z = pos.z

        # This does not take into account the length of the final arm.
        # Use L_3 to compensate for this in the preliminary target position
        # to keep the calculations the same

        ### DO NOT RUN THIS ON THE ROBOT!!!!!
        # TEST IT FIRST TO HAVE CORRECT ANGLE OFFSETS (AKA ORIENTATIONS)

        # 1. Find joint limits and their respective positions
        # 2. Look at how that impacts the logic

        rsp = IKinMsgResponse()
        rsp.joint_positions = []
        for i in range(4):
            rsp.joint_positions.append(Float64())
            rsp.joint_positions[i].data = -1

        L_1 = 0.13
        L_2 = 0.124
        L_3 = 0.126

        # is this correct? we want it to point down..
        # what if we want to approach it from the side?
        TARGET_ANGLE = -math.pi / 2

        r = max(0.0001, math.sqrt(x**2 + y**2))
        alpha = math.asin(y / r)

        # the target coordinates (r, z) need to be offset by the distance of the third
        # arm (length of end-effector) and the angle at which it is
        # this is calculated from TARGET_ANGLE

        # note that this is likely configured for the wrong orientations
        ee_dist_z = L_3 * math.sin(TARGET_ANGLE)
        ee_dist_r = L_3 * math.cos(TARGET_ANGLE)
        # ee_dist_r = 0
        # ee_dist_z = 0

        # again, the signs might be wrong here
        r -= ee_dist_r
        z -= ee_dist_z

        c_2 = (r**2 + z**2 - L_1**2 - L_2**2) / (2 * L_1 * L_2)
        # this is just to make sure it will always fail until we figure out what
        # offsets it needs to have
        if c_2 < -1 or c_2 > 1:
            # if True:
            rsp.success.data = False
            return rsp

        s_2 = -math.sqrt(1 - c_2**2)
        # this might possibly have to be negative for correct "elbow position"

        theta_1 = math.atan2(z, r) - math.atan2(L_2 * s_2, L_1 + L_2 * c_2)
        theta_2 = math.atan2(s_2, c_2)
        theta_3 = TARGET_ANGLE - theta_2 - theta_1

        rsp.success.data = True
        rsp.joint_positions[0].data = alpha
        rsp.joint_positions[1].data = theta_1
        rsp.joint_positions[2].data = theta_2
        rsp.joint_positions[3].data = theta_3

        return rsp


### DO NOT RUN THIS ON THE ROBOT!!!!! TEST IT FIRST TO HAVE CORRECT ANGLE OFFSETS
if __name__ == "__main__":
    rospy.init_node("inv_kin_service")
    try:
        inv_kin = InvKin()
    except rospy.ROSInterruptException:
        pass
