from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_path
import os

path_to_urdf = os.path.join(get_package_share_path('dobot_magician'), 'urdf/dobot.xacro.xml')

robot_description = ParameterValue(Command(['xacro ', path_to_urdf]),
                                   value_type=str)
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
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{
                    'robot_description': robot_description,
                    'publish_frequency': 20.0,
                }],
                output="screen",
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                parameters=[{
                    'source_list': ['dobot_joint_state_topic'],
                }],
            ),
        ]
    )
