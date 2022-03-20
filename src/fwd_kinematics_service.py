#! /usr/bin/env python3

import rospy
import math

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point

from cswk_kin.srv import FKinMsg, FKinMsgResponse


class FwdKin:

    _joints = []

    def __init__(self):
        self.joint_state_sub = rospy.Subscriber(
            "joint_states", JointState, self.saveJoints
        )

        self._as = rospy.Service("/fwd_kin", FKinMsg, self.calculateCameraPosition)
        rospy.spin()

    def saveJoints(self, jointState: JointState):
        self._joints = jointState.position

    def calculateCameraPosition(self, _) -> FKinMsgResponse:
        wait_for_joints = rospy.Rate(2)
        if len(self._joints) < 4:
            wait_for_joints.sleep()

        # Note this does not take into account the length of the
        # final arm where the gripper is, or the position of the
        # camera.
        # Use theta_3 for that along with information about
        # where the camera is in relation to the last joint

        alpha = self._joints[0]
        theta_1 = self._joints[1]
        theta_2 = self._joints[2]
        theta_3 = self._joints[3]

        L_1 = 0.13
        L_2 = 0.124

        r = L_1 * math.cos(theta_1) + L_2 * math.cos(theta_1 + theta_2)
        z = L_1 * math.sin(theta_1) + L_2 * math.sin(theta_1 + theta_2)

        y = math.sin(alpha) * r
        x = math.sqrt(r**2 - y**2)

        rsp = FKinMsgResponse()
        rsp.position = Point()
        rsp.position.x = x
        rsp.position.y = y
        rsp.position.z = z
        return rsp


if __name__ == "__main__":
    rospy.init_node("fwd_kin_service")
    try:
        fwd_kin = FwdKin()
    except rospy.ROSInterruptException:
        pass
