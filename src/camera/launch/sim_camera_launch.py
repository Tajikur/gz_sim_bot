from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
import os
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

import os

def generate_launch_description():
    pkg_camera_share = FindPackageShare('camera').find('camera')
    root_dir = FindPackageShare('camera')




    config_path = PathJoinSubstitution([
        FindPackageShare("camera"),
        "config",
        "ekf.yaml"
    ])
    
    world_file_path = os.path.join(pkg_camera_share, 'worlds', 'camera_world.sdf')

    rviz_config = os.path.join(pkg_camera_share, 'config', 'rvizcon.rviz')
    
    
    urdf_file = os.path.join(pkg_camera_share, 'urdf', 'bot.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()


    # Launch Gazebo with the custom world file
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': f'-r {world_file_path}'
        }.items()
    )
    create_entity_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_sim', 'create', '-topic', 'robot_description',
            '-z', '0.3'
        ],
        output='screen'
    )
    

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model'],
        output='screen'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[config_path]
    )
    return LaunchDescription([
        gz_sim_launch,
        create_entity_cmd,
        bridge,
        ekf_node,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_desc,
                         'use_sim_time': True
                         }]
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}],
            
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{ 'use_sim_time': True }],
        ),
    ])
