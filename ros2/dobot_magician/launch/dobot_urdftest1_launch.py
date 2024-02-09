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
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{
                    'robot_description': robot_description,
                    'publish_frequency': 20.0,
                }],
                output="screen",
            ),
            # Node(
            #     package="joint_state_publisher_gui",
            #     executable="joint_state_publisher_gui",
            # ),
        ]
    )
