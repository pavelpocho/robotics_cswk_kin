#! /usr/bin/env python

import rospy
import math

from std_msgs.msg import Float64
from geometry_msgs.msg import Point

from robotics_cswk_kin.srv import IKinMsg, IKinMsgRequest, IKinMsgResponse

### DO NOT RUN THIS ON THE ROBOT!!!!! TEST IT FIRST TO HAVE CORRECT ANGLE OFFSETS


class InvKin:
    def __init__(self):
        self._as = rospy.Service("/inv_kin", IKinMsg, self.calculateJointPositions)
        rospy.spin()

    def calculateJointPositions(self, msg):
        pos = msg.position
        x = pos.x
        y = pos.y
        z = pos.z

        # ADJUSTS ------------------------
        # Joint 1 is fine
        # Joint 2 must be set to -0.280 rad to point straight
        # Joint 3 must be set to -1.3 rad to point straight

        # This does not take into account the length of the final arm.
        # Use L_3 to compensate for this in the preliminary target position
        # to keep the calculations the same

        ### DO NOT RUN THIS ON THE ROBOT!!!!!
        # TEST IT FIRST TO HAVE CORRECT ANGLE OFFSETS (AKA ORIENTATIONS)

        # 1. Find joint limits and their respective positions
        # 2. Look at how that impacts the logic

        # Changes made to original model
        # 1. Set z as negative of itself
        # 2. Add 1.3 to joint 3

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
        TARGET_ANGLE = msg.angle.data

        # If target angle is 0, its going to be horizontal
        # If target angle is +90 (pi/2), its going to be vertical down

        r = max(0.000001, math.sqrt(x**2 + y**2))
        alpha = math.asin(y / r)

        # the target coordinates (r, z) need to be offset by the distance of the third
        # arm (length of end-effector) and the angle at which it is
        # this is calculated from TARGET_ANGLE

        # note that this is likely configured for the wrong orientations
        ee_dist_z = L_3 * (-math.sin(TARGET_ANGLE))
        ee_dist_r = L_3 * math.cos(TARGET_ANGLE)
        # ee_dist_r = 0
        # ee_dist_z = 0

        print('EE dists would be ')
        print(ee_dist_r)
        print(ee_dist_z)

        # again, the signs might be wrong here
        r -= ee_dist_r
        z -= ee_dist_z

        print('Corrected coords for EE')
        print('R:', r)
        print('Z:', z)

        # This c_2 is of an angle that has some other angle in it, which is the default offset
        # So c_2 is cosine of second angle (theta_2), which is joint angle 2 + 73.96deg (1.29rad) (from fwd kin)
        c_2 = (r**2 + z**2 - L_1**2 - L_2**2) / (2 * L_1 * L_2)
        print('C2:', c_2)
        # this is just to make sure it will always fail until we figure out what
        # offsets it needs to have
        if c_2 < -1 or c_2 > 1:
            # if True:
            rsp.success.data = False
            return rsp

        # s_2 is sin of second angle (theta_2), which is joint angle 2 + 73.96deg (1.29rad) (from fwd kin)
        s_2 = -math.sqrt(1 - c_2**2)
        # this might possibly have to be negative for correct "elbow position"
        # But it correctly identifies the theta_2 angle, which is joint 2 + 1.29 rad
        theta_2 = math.atan2(s_2, c_2)

        # This is the pain point
        # This formula essentially says 'target angle - second angle'
        # So what is wrong with this second angle?
        # Try #1: Put in sin and cos of the actual angle, not theta_2
        # Orig: theta_1 = math.atan2(z, r) - math.atan2(L_2 * s_2, L_1 + L_2 * c_2)
        # Try #1:
        theta_1 = math.atan2(z, r) - math.atan2(L_2 * math.sin(theta_2), L_1 + L_2 * math.cos(theta_2))
        
        theta_3 = TARGET_ANGLE - (- theta_2 - 1.29) - (- theta_1 + 1.29)
        # This needs to be 0 if the real robot is in its initial pose (all angles 0)

        # WITHOUT JOINT 4, THIS IS CORRECT

        rsp.success.data = True
        rsp.joint_positions[0].data = alpha
        rsp.joint_positions[1].data = - theta_1 + 1.29 # This also looks to be correct
        rsp.joint_positions[2].data = - theta_2 - 1.29 # This is absolutely correct
        rsp.joint_positions[3].data = theta_3

        # Joint 2 ([1]) can go down to -1.5 rad and up to 1.5 rad
        # Joint 3 ([2]) can go down to -1.5 rad and up to 1.5 rad 
        # Joint 4 ([3]) can go down to -1.8 rad and up to 2 rad

        # for r in rsp.joint_positions:
        #     if r.data > math.pi / 2 or r.data < math.pi / 2:
        #         rsp.success.data = False
        #         return rsp

        # Actual joint limits
        # realistic limits should be between pi / 2 and - pi / 2


        return rsp


### DO NOT RUN THIS ON THE ROBOT!!!!! TEST IT FIRST TO HAVE CORRECT ANGLE OFFSETS
if __name__ == "__main__":
    rospy.init_node("inv_kin_service")
    try:
        inv_kin = InvKin()
    except rospy.ROSInterruptException:
        pass
