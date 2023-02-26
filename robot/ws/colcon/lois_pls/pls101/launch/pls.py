from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='pls101',
      executable='pls101_node',
      name='pls_node',
      output='screen',
      parameters=[
          {'port': '/dev/ttyUSB0'},
          {'baud': 57600},
          {'frame': 'lidar_link'},
          {'range_m': 80.0}
      ]
    )
  ])
