import rclpy
from rclpy.node import Node

from std_srvs.srv import SetBool

from dobot_magician_interface.msg import Teleop
from dobot_magician.dobot_client import DobotClient, LIMITS

def helper(x, minval, maxval, defval):
    if x > 0:
        return maxval
    elif x < 0:
        return minval
    else:
        return defval

class DobotTeleop(Node):

    def __init__(self):
        super().__init__('dobot_teleop')
        self.subscription = self.create_subscription(
            Teleop,
            'dobot_teleop_command',
            self.command_callback,
            10)
        self.create_service(SetBool, 'dobot_teleop_service', self.service_callback)
        self.dobot = DobotClient()
        self.state = Teleop()
        self.enabled = False

    def service_callback(self, request, response):
        self.enabled = request.data
        if request.data:
            response.message = "Dobot teleop service enabled"
        else:
            response.message = "Dobot teleop service disabled"

        response.success = True
        return response

    def command_callback(self, msg):
        if not self.enabled:
            return

        j1, j2, j3, j4 = self.dobot.get_joint_state()
        j1 = helper(msg.j1, LIMITS.J1MIN, LIMITS.J1MAX, j1)
        j2 = helper(msg.j2, LIMITS.J2MIN, LIMITS.J2MAX, j2)
        j3 = helper(msg.j3, LIMITS.J3MIN, LIMITS.J3MAX, j3)
        j4 = helper(msg.j4, LIMITS.J4MIN, LIMITS.J4MAX, j4)
        if (msg.j1 != self.state.j1
            or msg.j2 != self.state.j2
            or msg.j3 != self.state.j3
            or msg.j4 != self.state.j4):
                self.dobot.set_joint_ptp(j1, j2, j3, j4)
                self.state.j1 = msg.j1
                self.state.j2 = msg.j2
                self.state.j3 = msg.j3
                self.state.j4 = msg.j4

        if msg.suction_cup != self.state.suction_cup:
            self.dobot.set_suction_cup(msg.suction_cup)
            self.state.suction_cup = msg.suction_cup

def main(args=None):
    rclpy.init(args=args)

    node = DobotTeleop()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
