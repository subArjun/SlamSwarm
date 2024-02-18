from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, IncludeLaunchDescription, ExecuteProcess
#from launch.event_handlers import onExecutionComplete
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python import get_package_share_directory
import xacro
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource




def generate_launch_description():
    ld = LaunchDescription()
    #launch description

    usb_command = ExecuteProcess(
    	cmd=[['echo arjun | sudo -S chmod 0777 /dev/ttyUSB0']],
    	shell=True,
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_a1_launch.py'
            ])
        ),
        launch_arguments={'frame_id':'base_scan'}.items(),
    )

    bringup = Node(
        package="slam_swarm",
        executable="bringup",
        name='bringup',
        output='screen',
    )
    

    ld.add_action(usb_command)
    ld.add_action(lidar)
    ld.add_action(bringup)
    return ld