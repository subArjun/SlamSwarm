from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, LogInfo, TimerAction
from launch.event_handlers import OnProcessIO, OnProcessStart, OnExecutionComplete, OnProcessExit
from launch.substitutions import PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from ament_index_python import get_package_share_directory
import xacro
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource

description_dir = os.path.join(get_package_share_directory('tele_mapping'), 'description','romipi.urdf.xacro')

config = os.path.join(get_package_share_directory('tele_mapping'), 'config','romipi.yaml')




def generate_launch_description():

    
    robot_desc = xacro.process_file(description_dir).toxml()    


    description = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name='robotState',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    navigation_core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py'
            ])
        ),
        launch_arguments={'use_sim_time':'false', 'params_file': config}.items()
    )
    
   

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('nav2_bringup'), 'launch', 'rviz_launch.py'
            ])
        )
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py'
            ])
        ),
        launch_arguments={'use_sim_time':'false'}.items()
    )
	
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('foxglove_bridge'), 'launch', 'foxglove_bridge_launch.xml'
            ])
        ),
        launch_arguments={'use_sim_time':'false', 'port':'8765'}.items()
    )


    return LaunchDescription([
        description,
        TimerAction(
        	period=6.0,
        	actions=[navigation_core]
        ),
        TimerAction(
        	period=15.0,
        	actions=[slam]
        ),
    ])

