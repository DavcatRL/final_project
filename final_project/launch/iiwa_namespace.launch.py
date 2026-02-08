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
                default_value='-r -v 1 empty.sdf',
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

    nodes = [
        gazebo,
        control_node,
        spawn_iiwa,
        iiwa_state_pub_node,
        delay_joint_state_broadcaster_spawner_after_control_node,
        delay_joint_state_broadcaster_spawner_after_spawn_entity,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        external_torque_broadcaster_spawner
    ]

    return LaunchDescription(iiwa_arguments + [gz_args] + nodes)