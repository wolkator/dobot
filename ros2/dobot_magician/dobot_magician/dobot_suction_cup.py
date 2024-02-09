from std_srvs.srv import SetBool

import rclpy
from rclpy.node import Node

from dobot_magician.dobot_client import DobotClient


class DobotSuctionCup(Node):
    def __init__(self):
        super().__init__("dobot_suction_cup")
        self.service = self.create_service(
            SetBool, "dobot_suction_cup_service", self.callback
        )
        self.dobot = DobotClient()

    def callback(self, request, response):
        self.dobot.set_suction_cup(request.data)
        if request.data:
            response.message = "Suction cup enabled"
        else:
            response.message = "Suction cup disabled"

        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)

    node = DobotSuctionCup()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
