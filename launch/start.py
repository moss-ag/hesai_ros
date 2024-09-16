from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile   
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()
    current_pkg = FindPackageShare('hesai_ros_driver')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution(
            [current_pkg, 'params', 'xt32.yaml']
        ),
        description='Full path to the ROS2 parameters file to use',
    )
    ld.add_action(declare_params_file)

    rviz_config = PathJoinSubstitution([current_pkg, 'rviz', 'rviz2.rviz'])

    hesai_node = Node(
        package='hesai_ros_driver',
        executable='hesai_node',
        output='screen',
        parameters=[ParameterFile(LaunchConfiguration('params_file'))],
    )
    ld.add_action(hesai_node)
    
    rviz_node = Node(namespace='rviz2', package='rviz2', executable='rviz2', arguments=['-d', rviz_config])
    ld.add_action(rviz_node)

    return ld
