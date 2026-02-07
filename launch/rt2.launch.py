from launch import  LaunchDescription
from launch_ros.actions import  Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('bme_gazebo_sensors'),
                'launch',
                'spawn_robot.launch.py'
                ]
            )
        )
    )
    return LaunchDescription(
        [   
            gazebo_launch,
            
            Node(
                package= 'assignment2_rt',
                executable = 'controller',
                name = 'controller',
                prefix='xterm -e' #nuovo terminale 
            ),
            Node(
                package= 'assignment2_rt',
                executable = 'ui',
                name = 'ui',
                prefix='xterm -e' #nuovo terminale 

            )
        ]
    )