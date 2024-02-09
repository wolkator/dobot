import time

from dobot_magician_interface.action import PTP

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node

from dobot_magician.dobot_client import DobotClient

GOAL_EPSILON = 1e-3


class DobotPTP(Node):
    def __init__(self):
        super().__init__("dobot_ptp")
        self.goal_handle = None
        self.action_server = ActionServer(
            self,
            PTP,
            "dobot_ptp_action",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
        )
        self._dobot_client = DobotClient()

    def destroy(self):
        self.action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        # Check if goal is valid
        j1, j2, j3, j4 = goal_request.joints
        is_goal_valid = self._dobot_client.is_goal_valid(j1, j2, j3, j4)
        if is_goal_valid:
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
        gj1, gj2, gj3, gj4 = goal_handle.request.joints
        self._dobot_client.set_ptp(gj1, gj2, gj3, gj4)

        feedback_msg = PTP.Feedback()
        while True:
            # If goal is flagged as no longer active (ie. another goal was accepted),
            # then stop executing
            if not goal_handle.is_active:
                self.get_logger().info("Goal aborted")
                return PTP.Result()

            # Publish the feedback
            j1, j2, j3, j4 = self._dobot_client.get_joints()
            feedback_msg.joints = [j1, j2, j3, j4]
            goal_handle.publish_feedback(feedback_msg)

            # Check if goal reached
            if (
                abs(gj1 - j1) < GOAL_EPSILON
                and abs(gj2 - j2) < GOAL_EPSILON
                and abs(gj3 - j3) < GOAL_EPSILON
                and abs(gj4 - j4) < GOAL_EPSILON
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

    node = DobotPTP()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
