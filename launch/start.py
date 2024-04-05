from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
    current_pkg = FindPackageShare('hesai_ros_driver')

    rviz_config = PathJoinSubstitution([current_pkg, 'rviz', 'rviz2.rviz'])

    hesai_node = Node(
            namespace='hesai_ros_driver',
            package='hesai_ros_driver',
            executable='hesai_ros_driver_node',
            output='screen',
    )
    ld.add_action(hesai_node)
    
    rviz_node = Node(namespace='rviz2', package='rviz2', executable='rviz2', arguments=['-d', rviz_config])
    ld.add_action(rviz_node)

    return ld
