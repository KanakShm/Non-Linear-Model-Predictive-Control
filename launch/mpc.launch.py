import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    path = os.path.join(
        get_package_share_directory('mpc')
    )
    trackdrive_params = path + '/config/trackdrive.yaml'

    # Declaration of launch arguments
    declare_mission_arg = DeclareLaunchArgument(
        'mission',
        default_value='trackdrive',
        description='Choose between trackdrive or ebs'
    )

    mission = LaunchConfiguration('mission')

    mpc_node = LifecycleNode(
        package='mpc',
        name='mpc_node',
        executable='MPC_executable',
        output='screen',
        parameters=[trackdrive_params, {'mission': mission}],
        namespace=''
    )

    return LaunchDescription([
        declare_mission_arg,
        mpc_node
    ])