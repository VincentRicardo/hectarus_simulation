from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_ros.actions



def generate_launch_description():
    launch_dir = PathJoinSubstitution([FindPackageShare('hectarus_sim'), 'launch'])
    return LaunchDescription([
        IncludeLaunchDescription(
            PathJoinSubstitution([launch_dir, 'sim.launch.py'])),
        launch_ros.actions.Node(
                package = 'hectarus_controller', executable= 'gazebo_joint_publisher'),
        launch_ros.actions.Node(
                package = 'hectarus_controller', executable= 'hectarus_imu_data')
        ])