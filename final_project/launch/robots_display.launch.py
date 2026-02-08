import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, OrSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description(): 

    ##### ------------------------- AVVIO GAZEBO ------------------------ #####
    gz_args = DeclareLaunchArgument(
                'gz_args', 
                default_value='-r -v 1 /home/user/ros2_ws/src/final_project/worlds/rough_bar.world',
                description='Arguments for gz_sim')
    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                    'launch',
                                    'gz_sim.launch.py'])]),
            launch_arguments={'gz_args': LaunchConfiguration('gz_args')}.items(),
    )

    ##### ------------------------- LAUNCH IIWA ------------------------- #####

    iiwa_arguments = []     # i valori di default sono scritti nel file iiwa.config.xacro, posso non inserire quelli che non mi servono

    iiwa_arguments.append(
        DeclareLaunchArgument(
            'runtime_config_package',
            default_value='iiwa_description',
            description='Package with the controller\'s configuration in "config" folder. \
                         Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    iiwa_arguments.append(
        DeclareLaunchArgument(
            'iiwa_controllers_file',
            default_value='iiwa_controllers.yaml',
            description='YAML file with the controllers configuration.',
        )
    )
    iiwa_arguments.append(
        DeclareLaunchArgument(
            'iiwa_description_package',
            default_value='iiwa_description',
            description='Description package with robot URDF/xacro files. Usually the argument \
                         is not set, it enables use of a custom description.',
        )
    )
    iiwa_arguments.append(
        DeclareLaunchArgument(
            'iiwa_description_file',
            default_value='iiwa.config.xacro',
            description='URDF/XACRO description file with the robot.',
        )
    )
    iiwa_arguments.append(
        DeclareLaunchArgument(
            'prefix',
            default_value='""',
            description='Prefix of the joint names, useful for multi-robot setup. \
                         If changed than also joint names in the controllers \
                         configuration have to be updated. Expected format "<prefix>/"',
        )
    )
    iiwa_arguments.append(
        DeclareLaunchArgument(
            'namespace',
            default_value='iiwa/',
            description='Namespace of launched nodes, useful for multi-robot setup. \
                         If changed than also the namespace in the controllers \
                         configuration needs to be updated. Expected format "<ns>/".',
        )
    )
    iiwa_arguments.append(
        DeclareLaunchArgument(
            'use_sim',
            default_value='true',
            description='Start robot in Gazebo simulation.',
        )
    )
    iiwa_arguments.append(
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='false',
            description='Start robot with fake hardware mirroring command to its states.',
        )
    )
    iiwa_arguments.append(
        DeclareLaunchArgument(
            'start_rviz',
            default_value='false',
            description='Start RViz2 automatically with this launch file.',
        )
    )
    iiwa_arguments.append(
        DeclareLaunchArgument(
            'robot_controller',
            default_value='iiwa_arm_controller',
            description='Robot controller to start.',
        )
    )
    iiwa_arguments.append(
        DeclareLaunchArgument(
            'initial_positions_file',
            default_value='initial_positions.yaml',
            description='Configuration file of robot initial positions for simulation.',
        )
    )
    iiwa_arguments.append(
        DeclareLaunchArgument(
            'command_interface',
            default_value='position',
            description='Robot command interface [position|velocity|effort].',
        )
    )
    iiwa_arguments.append(
        DeclareLaunchArgument(
            'base_frame_file',
            default_value='base_frame.yaml',
            description='Configuration file of robot base frame wrt World.',
        )
    )

    # Initialize Arguments
    runtime_config_package = LaunchConfiguration('runtime_config_package')
    iiwa_controllers_file = LaunchConfiguration('iiwa_controllers_file')
    iiwa_description_package = LaunchConfiguration('iiwa_description_package')
    iiwa_description_file = LaunchConfiguration('iiwa_description_file')
    prefix = LaunchConfiguration('prefix')
    namespace = LaunchConfiguration('namespace')
    use_sim = LaunchConfiguration('use_sim')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    start_rviz = LaunchConfiguration('start_rviz')
    iiwa_controller = LaunchConfiguration('robot_controller')
    initial_positions_file = LaunchConfiguration('initial_positions_file')
    command_interface = LaunchConfiguration('command_interface')
    base_frame_file = LaunchConfiguration('base_frame_file')

    # Get URDF via xacro: legge i parametri del launch e li passa alla xacro per l'inizializzazione
    iiwa_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare(iiwa_description_package), 'config', iiwa_description_file]
            ),
            ' ',
            'runtime_config_package:=', #10
            runtime_config_package,
            ' ',
            'controllers_file:=',#11
            iiwa_controllers_file,
            ' ',
            'description_package:=', #9
            iiwa_description_package,
            ' ',
            'prefix:=', #3
            prefix,
            ' ',
            'namespace:=', #12
            namespace,
            ' ',
            'use_sim:=', #1
            use_sim,
            ' ',
            'use_fake_hardware:=', #2
            use_fake_hardware,
            ' ',
            'initial_positions_file:=', #6
            initial_positions_file,
            ' ',
            'command_interface:=', #7
            command_interface,
            ' ',
            'base_frame_file:=', #8
            base_frame_file,
        ]
    )

    iiwa_description = {'robot_description': iiwa_description_content}

    iiwa_controllers = PathJoinSubstitution(
        [
            FindPackageShare(runtime_config_package),
            'config',
            iiwa_controllers_file,
        ]
    )

    ##### ------------------------- IIWA NODES ------------------------- #####
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[iiwa_description, iiwa_controllers],
        output='both',
        namespace=namespace,
        condition=UnlessCondition(use_sim),
    )


    iiwa_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        output='both',
        parameters=[iiwa_description],
    )

    spawn_iiwa = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        namespace=namespace,
        arguments=['-topic', 'robot_description',
                   '-name', 'iiwa',
                   '-allow_renaming', 'true'],
        condition=IfCondition(use_sim),
    )

    iiwa_joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        #namespace=namespace,
        arguments=['joint_state_broadcaster', '--controller-manager',
                   [namespace,'controller_manager']]                         
    )

    external_torque_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ets_state_broadcaster', '--controller-manager',
                   [namespace, 'controller_manager']],
        condition=UnlessCondition(use_sim),
    )

    iiwa_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        #namespace=namespace,
        arguments=[iiwa_controller, '--controller-manager', [namespace,'controller_manager']] #[namespace,'controller_manager']
    )

    iiwa_gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        #namespace=namespace,
        arguments=["gripper_controller", '--controller-manager', [namespace,'controller_manager']] #[namespace,'controller_manager']
    )

    # Delay `joint_state_broadcaster` after spawn_entity
    delay_joint_state_broadcaster_spawner_after_spawn_entity = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_iiwa,
            on_exit=[iiwa_joint_state_broadcaster_spawner],
        ),
        condition=IfCondition(use_sim),
    )

    # Delay `joint_state_broadcaster` after control_node
    delay_joint_state_broadcaster_spawner_after_control_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[iiwa_joint_state_broadcaster_spawner],
        ),
        condition=UnlessCondition(use_sim),
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=iiwa_joint_state_broadcaster_spawner,
            on_exit=[iiwa_controller_spawner],
        )
    )

    # Delay start of gripper_controller after `joint_state_broadcaster`
    delay_gripper_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=iiwa_joint_state_broadcaster_spawner,
            on_exit=[iiwa_gripper_controller_spawner],
        )
    )
    
    ##### ------------------------- LAUNCH FRA2MO ------------------------- #####

    fra2mo_arguments = []

    fra2mo_arguments.append(
        DeclareLaunchArgument(
            'fra2mo_description_package',
            default_value='ros2_fra2mo',
            description='Description package with robot URDF/xacro files. Usually the argument \
                         is not set, it enables use of a custom description.',
        )
    )
    fra2mo_arguments.append(
        DeclareLaunchArgument(
            'fra2mo_description_file',
            default_value='fra2mo.urdf.xacro',
            description='URDF/XACRO description file with the robot.',
        )
    )
    fra2mo_arguments.append(
        DeclareLaunchArgument(
            'fra2mo_spawn_x',
            default_value='0',
            description='Initial X coordinate',
        )
    )
    fra2mo_arguments.append(
        DeclareLaunchArgument(
            'fra2mo_spawn_y',
            default_value='0',
            description='Initial Y coordinate',
        )
    )
    fra2mo_arguments.append(
        DeclareLaunchArgument(
            'fra2mo_spawn_z',
            default_value='0.100',
            description='Initial Z coordinate',
        )
    )

    fra2mo_description_package = LaunchConfiguration('fra2mo_description_package')
    fra2mo_description_file = LaunchConfiguration('fra2mo_description_file')
    fra2mo_x = LaunchConfiguration('fra2mo_spawn_x')
    fra2mo_y = LaunchConfiguration('fra2mo_spawn_y')
    fra2mo_z = LaunchConfiguration('fra2mo_spawn_z')

    fra2mo_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare(fra2mo_description_package), 'urdf', fra2mo_description_file]
            )
        ]
    )

    fra2mo_description = {'robot_description': ParameterValue(fra2mo_description_content, value_type=str)}

    ##### ------------------------- FRA2MO NODES ------------------------- #####

    # Nodo robot_state_publisher
    fra2mo_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        namespace = 'fra2mo',
        parameters=[fra2mo_description,
                    {"use_sim_time": True}],            
        #remappings=[('/robot_description', '/fra2mo/robot_description')] # Namespace!
    )
    
    # Nodo joint_state_publisher
    fra2mo_joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{"use_sim_time": True}],
        namespace='fra2mo'
        #remappings=[('joint_states','fra2mo/joint_states')]
    )

    spawn_fra2mo = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        namespace='fra2mo',
        arguments=['-topic', 'robot_description',
                   '-name', 'fra2mo',
                   '-allow_renaming', 'true',
                    "-x", fra2mo_x,
                    "-y", fra2mo_y,
                    "-z", fra2mo_z]
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                   '/model/fra2mo/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                   '/model/fra2mo/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
                   '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'],
                   #'/lidar/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'], 
        output='screen'
    )

    odom_tf = Node(
        package='ros2_fra2mo',
        executable='dynamic_tf_publisher',
        name='odom_tf',
        parameters=[{"use_sim_time": True}]
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_clock_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )

    '''
        remappings=[
                ("/tf", "tf"),
                ("/tf_static", "tf_static"),
            ],
    '''
        
    pkg_fra2mo = get_package_share_directory('ros2_fra2mo')
    pkg_iiwa = get_package_share_directory('iiwa_description')
    pkg_project = get_package_share_directory('final_project')
    resource_env = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', 
        value=os.path.join(pkg_project, 'meshes') + ':' + 
              os.path.join(pkg_fra2mo, 'meshes') + ':' + 
              os.path.join(pkg_iiwa, 'meshes') + ':' + 
              os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    )

    nodes = [
        gazebo,
        control_node,
        spawn_iiwa,
        iiwa_state_pub_node,
        delay_joint_state_broadcaster_spawner_after_control_node,
        delay_joint_state_broadcaster_spawner_after_spawn_entity,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_gripper_controller_spawner_after_joint_state_broadcaster_spawner,
        #external_torque_broadcaster_spawner,
        fra2mo_state_publisher_node,
        fra2mo_joint_state_publisher_node,
        spawn_fra2mo,
        bridge,
        odom_tf,
        clock_bridge
    ]

    return LaunchDescription([resource_env] + [gz_args] + iiwa_arguments + fra2mo_arguments + nodes)