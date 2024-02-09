from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="dobot_magician",
                executable="dobot_server",
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="dobot_magician",
                executable="dobot_homing",
            ),
            Node(
                package="dobot_magician",
                executable="dobot_joint_state",
            ),
            Node(
                package="dobot_magician",
                executable="dobot_joint_ptp",
            ),
            Node(
                package="dobot_magician",
                executable="dobot_teleop_keyboard",
            ),
            Node(
                package="dobot_magician",
                executable="dobot_teleop",
            ),
            Node(
                package="dobot_magician",
                executable="dobot_cartesian_ptp",
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="dobot_magician",
                executable="dobot_suction_cup",
            ),
        ]
    )
