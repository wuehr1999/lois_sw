from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='lois_ecu',
      executable='lois_ecu_node',
      name='ecu_node',
      output='screen',
      parameters=[
          {'port': '/dev/ttyTHS1'},
          {'baud': 115200},
          {'wheel_width_m': 0.5},
          {'wheel_radius_m': 0.125},
          {'encoder_steps': 60},
          {'configfile': '/home/lois/launch/ecu_runtimeparameters.ini'},
          {'odom_parent': 'odom'},
          {'odom_child': 'base_link'}
      ]
    )
  ])
