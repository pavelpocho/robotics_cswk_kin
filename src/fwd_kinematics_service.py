#! /usr/bin/env python

import rospy
import math

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point

from robotics_cswk_kin.srv import FKinMsg, FKinMsgResponse


class FwdKin:

    _joints = []

    def __init__(self):
        self.joint_state_sub = rospy.Subscriber(
            "joint_states", JointState, self.saveJoints
        )

        self._as = rospy.Service("/fwd_kin", FKinMsg, self.calculateCameraPosition)
        rospy.spin()

    def saveJoints(self, jointState):
        self._joints = jointState.position

    def calculateCameraPosition(self, _):
        wait_for_joints = rospy.Rate(2)
        if len(self._joints) < 4:
            wait_for_joints.sleep()

        # Note this does not take into account the length of the
        # final arm where the gripper is, or the position of the
        # camera.
        # Use theta_3 for that along with information about
        # where the camera is in relation to the last joint

        # Changes made compared to original calculation:
        # Subtracted 1.29 from joint 2
        # Added 1.29 to joint 3
        # Set z as negative

        alpha = self._joints[0]
        theta_1 = self._joints[1] - 1.29
        theta_2 = self._joints[2] + 1.29
        theta_3 = self._joints[3]

        L_1 = 0.13
        L_2 = 0.124
        L_3 = 0.070 # This is for the camera, but there is also a shift in z
        L_3Z = 0.052 # This is the z shift of the camera

        # The actual formula here is
        # r = L_1 * math.cos((joint_1 - 73.96)) + L_2 * math.cos((joint_1 - 73.96) + (joint_2 + 73.96))
        # z = L_1 * math.sin((joint_1 - 73.96)) + L_2 * math.sin((joint_1 - 73.96) + (joint_2 + 73.96))
        # This is exactly what I did when setting theta_1 and theta_2 and is correct

        print(theta_1 + theta_2 + theta_3)

        r = (
            L_1 * math.cos(theta_1)
            + L_2 * math.cos(theta_1 + theta_2)
            + L_3 * math.cos(theta_1 + theta_2 + theta_3)
            + L_3Z * math.cos(theta_1 + theta_2 + theta_3 - math.pi/2)
        )
        z = -(
            L_1 * math.sin(theta_1)
            + L_2 * math.sin(theta_1 + theta_2)
            + L_3 * math.sin(theta_1 + theta_2 + theta_3)
            + L_3Z * math.sin(theta_1 + theta_2 + theta_3 - math.pi/2)
        )

        y = math.sin(alpha) * r
        x = math.sqrt(r**2 - y**2)

        # WITHOUT JOINT 4, THIS IS CORRECT

        rsp = FKinMsgResponse()
        rsp.position = Point()
        rsp.position.x = x
        rsp.position.y = y
        rsp.position.z = z
        rsp.angle.data = theta_1 + theta_2 + theta_3
        return rsp


if __name__ == "__main__":
    rospy.init_node("fwd_kin_service")
    try:
        fwd_kin = FwdKin()
    except rospy.ROSInterruptException:
        pass
