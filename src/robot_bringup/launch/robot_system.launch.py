import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    urdf_file = '/home/apollo/tema2/src/robot_bringup/urdf/scara_robot.urdf'
    rviz_config_file = '/home/apollo/tema2/src/robot_bringup/rviz/scara_robot.rviz'
    waypoints_file = '/home/apollo/tema2/src/robot_bringup/config/waypoints.json' 

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
        ),
        
        Node(
            package='robot_controller',
            executable='robot_driver', 
            name='scara_controller',
            output='screen'
        ),

        Node(
            package='robot_controller',
            executable='motion_publisher_node',
            name='waypoint_sender',
            output='screen',
            parameters=[
                {'waypoint_file': waypoints_file}
            ]
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
        )
    ])