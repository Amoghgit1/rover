from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_name = 'rover2'

    # Path to Gazebo ROS launch file
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    gazebo_launch_file = os.path.join(gazebo_ros_dir, 'launch', 'launch.py')
    
    # Path to URDF
    urdf_path = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        'rover_first.urdf'
    )
    
    # Validate URDF file exists
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"❌ URDF file not found at: {urdf_path}")
    
    print(f"✅ Using URDF path: {urdf_path}")
    
    # Read URDF file
    with open(urdf_path, 'r') as urdf_file:
        robot_desc = urdf_file.read()

    return LaunchDescription([
        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_file),
            launch_arguments={'verbose': 'true'}.items()
        ),
        
        # Publish robot description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        
        # Spawn rover in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'rover',
                '-topic', '/robot_description',
                '-z', '0.3'
            ],
            output='screen'
        )
    ])