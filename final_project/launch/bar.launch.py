from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.event_handlers import OnProcessStart

import os

def generate_launch_description():
     
    controller_iiwa = DeclareLaunchArgument(
       'command_interface',
        default_value='velocity',
        description='iiwa launch argument'
    )
     
    controller_iiwa_robot= DeclareLaunchArgument(
        'robot_controller',
        default_value='velocity_controller',
        description='iiwa launch argument'
    )

    use_sim= DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='use gazebo simulation'
    )

    start_rviz= DeclareLaunchArgument(
        'start_rviz',
        default_value='false',
        description='use_riviz2 for visualization'
    )

    use_fake_hardware= DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='false',
        description='needed for rviz simulation'
    )

    map_file = DeclareLaunchArgument(
        'map_file',
        default_value=PathJoinSubstitution([FindPackageShare("final_project"), "maps", "counter_map.yaml"]),
        description="Full path to the yaml map file",
    )
     
    # lancio il robot iiwa in simulazione
    iiwa_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(  # serve questa riga per caricare un launch.py
            PathJoinSubstitution([
                FindPackageShare('final_project'),  
               'launch',
               'robots_display.launch.py' 
            ])
        ),
        launch_arguments={
            'use_sim': LaunchConfiguration('use_sim'),
            'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
            'start_rviz': LaunchConfiguration('start_rviz'),
            'gz_args': '-r -v 1 /home/user/ros2_ws/src/final_project/worlds/rough_bar.world',
            'command_interface': LaunchConfiguration('command_interface'),
            'robot_controller': LaunchConfiguration('robot_controller'),
        }.items()
    )
     
     
    # nodo per la detection dell'aruco tag
    simple_double = Node(
        package='aruco_ros',
        executable='double',
        name='aruco_double',
        output='screen',
        parameters=[
            {'image_is_rectified': True},
            {'normalizeImage': False},
            {'dct_components_to_remove': 0},
            {'marker_size': 0.075},
            {'marker_id1': 201},
            {'marker_id2': 301},
            {'parent_name': 'stereo_gazebo_left_camera_optical_frame'}, # camera frame
            {'child_name1': 'arucotag1'},                               # marker frame 1
            {'child_name2': 'arucotag2'},                               # marker frame 2
        ],
        remappings=[
            ('/image', '/stereo/left/image_rect_color'),
            ('/camera_info', '/stereo/left/camera_info')
        ]
    )
   

    # Lancio dell'action server    
    action_server = Node(
        package='ros2_kdl_package',
        executable='ros2_kdl_action',
        name='ros2_kdl_action',
        output='screen'
    )

    # Delay start of action server 
    delay_action_server = TimerAction(
        period=15.0,
        actions=[action_server]
    )

    # Lancio del nodo di simulazione del grab
    grab_node = Node(
        package='final_project',
        executable='grab_simulator',
        name='grab_simulator',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # lancio di amcl per la navigazione di fra2mo
    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros2_fra2mo"),  # il package che contiene il launch amcl
                "launch",
                "fra2mo_navigation.launch.py"  # nome del tuo launch amcl
            ])
        ),
        launch_arguments={
            'map': LaunchConfiguration('map_file'),
            'use_sim_time': 'true'
        }.items()
    )    

    delay_amcl_launch = TimerAction(
        period=15.0,
        actions=[amcl_launch]
    )
    
    camera_bridge= Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '--ros-args',
            '-r', '/camera:=/stereo/left/image_rect_color',
            '-r', '/camera_info:=/stereo/left/camera_info'
        ]
    )
     
    # bridge for the gazebo service which updates the aruco pose
    set_aruco_pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='marker_set_pose',
        arguments=[
            '/world/default/set_pose@ros_gz_interfaces/srv/SetEntityPose',
            '--ros-args',
            '--log-level',
            'info'
        ],
        output='screen'
    )

     
    # Ritorna la descrizione del launch 
    return LaunchDescription([
        controller_iiwa,
        controller_iiwa_robot,
        use_sim,
        use_fake_hardware,
        start_rviz,
        map_file,
        iiwa_launch,
        grab_node,
        delay_amcl_launch,
        camera_bridge,
        set_aruco_pose_bridge,
        simple_double,
        delay_action_server
    ])
