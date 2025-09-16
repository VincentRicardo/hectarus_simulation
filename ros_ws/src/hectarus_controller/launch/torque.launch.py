from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import launch_ros.actions



def generate_launch_description():
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/coxa1_torque@geometry_msgs/msg/Wrench@gz.msgs.Wrench',
                   '/coxa2_torque@geometry_msgs/msg/Wrench@gz.msgs.Wrench',
                   '/coxa3_torque@geometry_msgs/msg/Wrench@gz.msgs.Wrench',
                   '/coxa4_torque@geometry_msgs/msg/Wrench@gz.msgs.Wrench',
                   '/coxa5_torque@geometry_msgs/msg/Wrench@gz.msgs.Wrench',
                   '/coxa6_torque@geometry_msgs/msg/Wrench@gz.msgs.Wrench',
                   '/femur1_torque@geometry_msgs/msg/Wrench@gz.msgs.Wrench',
                   '/femur2_torque@geometry_msgs/msg/Wrench@gz.msgs.Wrench',
                   '/femur3_torque@geometry_msgs/msg/Wrench@gz.msgs.Wrench',
                   '/femur4_torque@geometry_msgs/msg/Wrench@gz.msgs.Wrench',
                   '/femur5_torque@geometry_msgs/msg/Wrench@gz.msgs.Wrench',
                   '/femur6_torque@geometry_msgs/msg/Wrench@gz.msgs.Wrench',
                   '/tibia1_torque@geometry_msgs/msg/Wrench@gz.msgs.Wrench',
                   '/tibia2_torque@geometry_msgs/msg/Wrench@gz.msgs.Wrench',
                   '/tibia3_torque@geometry_msgs/msg/Wrench@gz.msgs.Wrench',
                   '/tibia4_torque@geometry_msgs/msg/Wrench@gz.msgs.Wrench',
                   '/tibia5_torque@geometry_msgs/msg/Wrench@gz.msgs.Wrench',
                   '/tibia6_torque@geometry_msgs/msg/Wrench@gz.msgs.Wrench',],
        #output='screen'
    )
    return LaunchDescription([
        bridge,
        launch_ros.actions.Node(
                package = 'hectarus_controller', executable= 'hectarus_torque_data')
        ])