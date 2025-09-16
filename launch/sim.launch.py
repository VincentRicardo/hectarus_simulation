import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the Xacro file
    pkg_path = os.path.join(get_package_share_directory('hectarus_sim'))
    xacro_file = os.path.join(pkg_path,'description','hectarus_gazebo.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    stairs_sdf = os.path.join(pkg_path, 'world', 'stairs.sdf')

    #open world_init gazevo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_path,
            'world',
            'world_init.sdf'
        ])}.items(),
    )

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        #output='screen',
        parameters=[params]
    )

    # Rviz2
    node_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        #output='screen',
    )

    #spawn robot in gazebo world
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        #output='screen',
        arguments=['-topic', 'robot_description', '-name',
                   'hectarus', '-allow_renaming', 'true'],
    )


    joint_state_broadcaster = ExecuteProcess(
        cmd =['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output = 'screen'
    )

    joint_trajectory_controller = ExecuteProcess(
        cmd =['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_trajectory_controller'],
        output = 'screen'
    )
    #unpause gazebo world
    unpause = ExecuteProcess(
        cmd =['gz', 'service', '-s', 'world/hectarus_world/control',
              '--reqtype', 'gz.msgs.WorldControl',
              '--reptype', 'gz.msgs.Boolean',
              '--timeout', '2000',
              '--req', 'pause: false'],
        #output = 'screen'
    )

    #ros2 gz bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',],
        #output='screen'
    )
    #set_pose robot
    set_pose = ExecuteProcess(
        cmd =['gz', 'service', '-s', 'world/hectarus/set_pose', 
              '--reqtype', 'gz.msgs.Pose',
              '--reptype', 'gz.msgs.Boolean',
              '--timeout', '2000',
              '--req', 'name: "hectarus" id: 10'
              ' position {x: 0.0 y: 0.0 z: 0.052}'
              ' orientation {x: 0.0 y: 0.0 z: 0.0 w:1.0}'],
        #output = 'screen'
    )

    req_string = f'sdf_filename:"{stairs_sdf}", name:"stairs"'

    spawn_stairs = ExecuteProcess(
    cmd=[
        'gz', 'service', '-s', 'world/hectarus_world/create',
        '--reqtype', 'gz.msgs.EntityFactory',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '2000',
        '--req', req_string
    ],
    output='screen'
)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        node_robot_state_publisher,
        node_rviz2,
        gz_sim,
        gz_spawn_entity,
        bridge,
        unpause,
        TimerAction(
            period=5.0,  # Delay of 5 seconds
            actions=[
                set_pose,
                spawn_stairs,
                joint_state_broadcaster,
                joint_trajectory_controller
                ])
    ])