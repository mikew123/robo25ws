from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robo25_stuff',
            executable='engine_node',
            name='engine'
        ),
        Node(
            package='robo25_stuff',
            executable='imu_gps_node',
            name='imu_gps'
        ),

        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a'],
            output='screen'
        ),       

    ])
