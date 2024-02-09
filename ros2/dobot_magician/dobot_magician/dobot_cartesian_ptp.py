import time
import numpy as np
from numpy import sin, cos, pi, who

from dobot_magician_interface.action import PTP

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node

from dobot_magician.dobot_client import DobotClient

GOAL_EPSILON = 1


# forward kinematics
# note: joint angles in degrees, position in mm
def fk(j1, j2, j3):
    j1 = pi / 180 * j1
    j2 = pi / 180 * j2
    j3 = pi / 180 * j3

    x = 3 * cos(j1) * (49 * cos(j2 + j3) + 45 * sin(j2) + 20)
    y = 3 * sin(j1) * (49 * cos(j2 + j3) + 45 * sin(j2) + 20)
    z = 135 * cos(j2) - 147 * sin(j2 + j3) + 68

    return np.array([x, y, z])


# Jacobian of the forward kinematics
# note: joint angles in degrees, position in mm
def fk_jacobian(j1, j2, j3):
    j1 = pi / 180 * j1
    j2 = pi / 180 * j2
    j3 = pi / 180 * j3

    J1 = [
        -3 * sin(j1) * (49 * cos(j2 + j3) + 45 * sin(j2) + 20),
        -3 * cos(j1) * (49 * sin(j2 + j3) - 45 * cos(j2)),
        -147 * sin(j2 + j3) * cos(j1),
    ]
    J2 = [
        3 * cos(j1) * (49 * cos(j2 + j3) + 45 * sin(j2) + 20),
        -3 * sin(j1) * (49 * sin(j2 + j3) - 45 * cos(j2)),
        -147 * sin(j2 + j3) * sin(j1),
    ]

    J3 = [0, -147 * cos(j2 + j3) - 135 * sin(j2), -147 * cos(j2 + j3)]

    return np.array([J1, J2, J3])


# inverse kinematics
def ik(j1, j2, j3, pg):
    p = fk(j1, j2, j3)
    p0 = p.copy()

    dpg = pg - p

    # could be node parameter
    default_step = 1

    N = int(np.ceil(np.linalg.norm(dpg) / default_step))
    if N == 0:
        jout = np.zeros((1, 3))
        jout[0, :] = np.array([[j1, j2, j3]])
        pout = np.zeros((1, 3))
        pout[0, :] = np.array([p])

        return jout, pout

    jout = np.zeros((N, 3))
    pout = np.zeros((N, 3))

    for n in range(N):
        dp = p0 + dpg * (n + 1) / N - p
        J = fk_jacobian(j1, j2, j3)
        dj = 180 / pi * np.linalg.solve(J, dp)

        j1 = j1 + dj[0]
        j2 = j2 + dj[1]
        j3 = j3 + dj[2]

        jout[n, :] = np.array([j1, j2, j3])

        p = fk(j1, j2, j3)
        pout[n, :] = p

    return jout, pout


class DobotCartesianPTP(Node):
    def __init__(self):
        super().__init__("dobot_cartesian_ptp")
        self.goal_handle = None
        self.action_server = ActionServer(
            self,
            PTP,
            "dobot_cartesian_ptp_action",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
        )
        self.dobot = DobotClient()
        self.pgoal = np.zeros(3)
        self.jvalues = np.zeros((1, 3))
        self.pvalues = np.zeros((1, 3))

    def destroy(self):
        self.action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        # Check if goal is valid
        j1, j2, j3, j4 = self.dobot.get_joint_state()
        x, y, z = goal_request.cartesian
        pg = np.array([x, y, z])

        self.jvalues, self.pvalues = ik(j1, j2, j3, pg)
        j1, j2, j3 = self.jvalues[-1, :]

        print(self.jvalues)
        print(self.pvalues)

        is_goal_valid = self.dobot.is_goal_valid(j1, j2, j3, j4)

        if is_goal_valid:
            self.pgoal = pg
            return GoalResponse.ACCEPT
        else:
            return GoalResponse.REJECT

    def handle_accepted_callback(self, goal_handle):
        if self.goal_handle is not None and self.goal_handle.is_active:
            self.get_logger().info("Aborting previous goal")
            # Abort the existing goal
            self.goal_handle.abort()
        self.goal_handle = goal_handle
        goal_handle.execute()

    def execute_callback(self, goal_handle):
        # Start executing the action
        _, _, _, j4 = self.dobot.get_joint_state()
        for n in range(np.size(self.jvalues, 0)):
            j1, j2, j3 = self.jvalues[n, :]
            self.dobot.set_joint_ptp(j1, j2, j3, j4)

        feedback_msg = PTP.Feedback()
        while True:
            # If goal is flagged as no longer active (ie. another goal was accepted),
            # then stop executing
            if not goal_handle.is_active:
                self.get_logger().info("Goal aborted")
                return PTP.Result()

            # Publish the feedback
            j1, j2, j3, j4 = self.dobot.get_joint_state()
            x, y, z = fk(j1, j2, j3)
            feedback_msg.cartesian = [x, y, z, j4]
            goal_handle.publish_feedback(feedback_msg)

            # Check if goal reached
            if (
                abs(x - self.pgoal[0]) < GOAL_EPSILON
                and abs(y - self.pgoal[1]) < GOAL_EPSILON
                and abs(z - self.pgoal[2]) < GOAL_EPSILON
            ):
                break

            time.sleep(0.5)

        goal_handle.succeed()

        # Populate result message
        result = PTP.Result()
        result.success = True

        return result


def main(args=None):
    rclpy.init(args=args)

    node = DobotCartesianPTP()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
