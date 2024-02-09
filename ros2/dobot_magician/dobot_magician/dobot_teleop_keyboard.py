import rclpy
from rclpy.node import Node

from dobot_magician_interface.msg import Teleop
from dobot_magician.keyboard import Keyboard

KEYS = { 'KEY_J', 'KEY_L', 'KEY_I', 'KEY_K', 'KEY_S', 'KEY_F', 'KEY_E', 'KEY_D', 'KEY_R' }

class DobotTeleopKeyboard(Node):

    def __init__(self):
        super().__init__('dobot_teleop_keyboard')
        self.publisher = self.create_publisher(Teleop, 'dobot_teleop_command', 10)
        self.keyboard = Keyboard(KEYS)
        self.state = Teleop()
        self.timer = self.create_timer(timer_period_sec=0.01, callback=self.callback)

    def callback(self):
        key = self.keyboard.read_key()
        if key is not None:
            if key[0] == 'KEY_J':
                self.state.j1 += (1 if key[1] == 1 else -1)

            if key[0] == 'KEY_L':
                self.state.j1 += (-1 if key[1] == 1 else 1)

            if key[0] == 'KEY_I':
                self.state.j2 += (1 if key[1] == 1 else -1)

            if key[0] == 'KEY_K':
                self.state.j2 += (-1 if key[1] == 1 else 1)

            if key[0] == 'KEY_E':
                self.state.j3 += (1 if key[1] == 1 else -1)

            if key[0] == 'KEY_D':
                self.state.j3 += (-1 if key[1] == 1 else 1)

            if key[0] == 'KEY_S':
                self.state.j4 += (1 if key[1] == 1 else -1)

            if key[0] == 'KEY_F':
                self.state.j4 += (-1 if key[1] == 1 else 1)

            if key[0] == 'KEY_R' and key[1] == 1:
                self.state.suction_cup = not self.state.suction_cup

            self.publisher.publish(self.state)


def main(args=None):
    rclpy.init(args=args)

    node = DobotTeleopKeyboard()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()


