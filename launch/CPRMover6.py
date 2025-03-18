import launch
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import  DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():


    use_sim_time = LaunchConfiguration('use_sim_time',default='false')
    urdf_file_name = 'CPRMover6.urdf.xacro'
        
    urdf= os.path.join(get_package_share_directory('cpr_ros2'),'urdf',urdf_file_name)
    
    
    rviz_file_name = 'CPRMover6.rviz'
    rviz_file_path= os.path.join(get_package_share_directory('cpr_ros2'),'urdf',rviz_file_name)
    
    with open(urdf, 'r') as infp:
    	robot_desc = infp.read()
    #print(urdf)
    #print(robot_desc)
    return LaunchDescription([
        # Declare the launch arguments
        DeclareLaunchArgument('robot_type', default_value='CPRMover6', description='Type of the robot'),
        DeclareLaunchArgument('robot_description', default_value=PathJoinSubstitution([FindPackageShare('cpr_ros2'), '/robots/CPRMover6.urdf.xacro']), description='Path to the robot description'),
        DeclareLaunchArgument('use_gui', default_value='True', description='Whether to use GUI in RViz'),
        
        
        

        # Log information about the launch
        #LogInfo(condition=LaunchConfiguration('use_gui'), msg="Launching with GUI"),

        # Robot State Publisher node
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]
        ),

        # RViz node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d', rviz_file_path]
        ),
        
        # Custom robot node (your robot's main node)
        Node(
            package='cpr_ros2',
            executable='CPRMover6',
            name='CPRMover6',
            output='screen',
        )
    ])

