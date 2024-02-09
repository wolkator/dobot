import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

from dobot_magician.dobot_client import DobotClient

LAST_EPSILON = 1

def sign(x, minval, maxval):
    if x > 0:
        return maxval
    elif x < 0:
        return minval
    else:
        return 0

class DobotJointTeleop(Node):

    def __init__(self):
        super().__init__('dobot_joint_teleop')
        self.subscription = self.create_subscription(
            Twist,
            'dobot_joint_teleop_topic',
            self.listener_callback,
            10)
        self.dobot = DobotClient()
        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.motion_callback)
        self.goal_joints = [0, 0, 0, 0]
        self.key_pressed = 0.0
        self.in_motion = False

    def listener_callback(self, msg):
        # self.get_logger().warn(f"{msg}")
        self.goal_joints[0] = msg.angular.z
        self.key_pressed = 1.0
            
    def motion_callback(self):
        j1, j2, j3, j4 = self.dobot.get_joint_state()
        if self.key_pressed >= 0.2 and not self.in_motion:
            j1 = sign(self.goal_joints[0], -90, 90)
            self.dobot.set_joint_ptp(j1, j2, j3, j4)
            self.in_motion = True
        if self.key_pressed < 0.2 and self.in_motion:
            self.dobot.set_joint_ptp(j1, j2, j3, j4)
            self.in_motion = False

        self.key_pressed *= 0.5



def main(args=None):
    rclpy.init(args=args)

    node = DobotJointTeleop()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
