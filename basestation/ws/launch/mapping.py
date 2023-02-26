from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    parameters=[{
          'frame_id':'lidar_link',
          'subscribe_depth':True,
          'approx_sync':True,
          'queue_size':1000}]

    remappings=[
          ('rgb/image', '/camera/color/image_raw'),
          ('rgb/camera_info', '/camera/color/camera_info'),
          ('depth/image', '/camera/depth/image_rect_raw')]

    return LaunchDescription([

        Node(
            package='rtabmap_ros', executable='icp_odometry', output='screen',
            parameters=parameters,
            remappings=remappings),

        Node(
            package='rtabmap_ros', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']),

       Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen" ,
           arguments=["0", "0", "0", "0", "0", "0", "lidar_link", "camera_link"]
        )
   ])
