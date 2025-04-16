from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Launch TurtleBot3 Simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('my_robot_controller'),
                    'launch',
                    'my_turtlebot3.launch.py'  # This should launch the simulation environment
                ])
            ]),
        ),
        
        # Launch Cartographer for SLAM
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('turtlebot3_cartographer'),
                    'launch',
                    'cartographer.launch.py'  # Cartographer launch file for SLAM
                ])
            ]),
            launch_arguments={'use_sim_time': 'True'}.items(),  # Use Gazebo simulated time
        ),
        
        # Launch your Autonomous Mapping Node
        Node(
            package='my_robot_controller',  # Ensure the package is correct
            executable='mapping',  # This should be the executable for your robot's movement/exploration
            name='control',  # Name for the node
            output='screen',  # Outputs the node's logs to the terminal
            parameters=[{
                'use_sim_time': True,  # Use simulated time
            }],
        ),

        # Optionally, you can include a mapping display in RViz
        Node(
            package='rviz2',  # Launch RViz
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{
                'use_sim_time': True,  # Use the same time as Gazebo
            }],
            arguments=['-d', PathJoinSubstitution([FindPackageShare('turtlebot3_gazebo'), 'rviz', 'turtlebot3_robomaster.rviz'])]  # Optionally load a custom RViz config
        )
    ])
