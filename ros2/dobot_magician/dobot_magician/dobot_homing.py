from std_srvs.srv import Empty

import rclpy
from rclpy.node import Node

from dobot_magician.dobot_client import DobotClient


class DobotHoming(Node):
    def __init__(self):
        super().__init__("dobot_homing")
        self.service = self.create_service(Empty, "dobot_homing_service", self.callback)
        self.dobot = DobotClient()

    def callback(self, request, response):
        self.dobot.start_homing()
        return response


def main(args=None):
    rclpy.init(args=args)

    node = DobotHoming()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
