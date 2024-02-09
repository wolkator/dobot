import rclpy
from rclpy.node import Node
from math import radians

from sensor_msgs.msg import JointState

from dobot_magician.dobot_client import DobotClient


class DobotJointState(Node):
    def __init__(self):
        super().__init__("dobot_joint_state")
        self.publisher = self.create_publisher(
            JointState, "dobot_joint_state_topic", 10
        )
        self.dobot = DobotClient()
        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.callback)

    def callback(self):
        j1, j2, j3, j4 = self.dobot.get_joint_state()

        msg = JointState()
        msg.name = ["joint1", "joint2", "joint3", "joint4"]
        msg.position = [radians(j) for j in [j1, j2, j3, j4]]
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = DobotJointState()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
