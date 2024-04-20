
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_dir = get_package_share_directory('quad_off_board')
    # bash_script_path = os.path.join(package_dir, 'scripts', 'TerminatorScript.sh')
    return LaunchDescription([
        # ExecuteProcess(cmd=['bash', bash_script_path], output='screen'),
        Node(
            package='quad_off_board',
            namespace='quad_off_board',
            executable='viz',
            name='visualizer'
        ),
        Node(
            package='quad_off_board',
            namespace='quad_off_board',
            executable='off_board',
            prefix='gnome-terminal --',
        ),
        Node(
            package='quad_off_board',
            namespace='quad_off_board',
            executable='vel_control',
            name='velocity'
        ),
        # Node(
        #     package='rviz2',
        #     namespace='',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', [os.path.join(package_dir, 'visualize.rviz')]]
        # )
    ])
