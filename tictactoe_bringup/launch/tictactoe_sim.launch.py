from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('tictactoe_bringup')
    webpage_dir = os.path.join(pkg_share, 'webpage')

    piper_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('piper_gazebo'),
                'launch',
                'piper_with_gripper',
                'piper_gazebo.launch.py'
            )
        )
    )

    move_group_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('piper_with_gripper_moveit'),
                'launch',
                'move_group_sim.launch.py'
            )
        )
    )

    board_processor = Node(
        package='board_perception',
        executable='board_processor_node',
        name='ttt_board_processor',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    trajectory_server = Node(
        package='piper_trajectory',
        executable='trajectory_server',
        name='ttt_trajectory_server',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    move_manager = Node(
        package='move_manager',
        executable='move_manager_node',
        name='ttt_manager',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    rosbridge_xml = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rosbridge_server'),
                'launch',
                'rosbridge_websocket_launch.xml'
            )
        )
    )

    web_video = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='screen'
    )

    http_server = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', '7000'],
        cwd=webpage_dir,
        output='screen'
    )

    return LaunchDescription([
        piper_gazebo,
        move_group_sim,
        board_processor,
        trajectory_server,
        move_manager,
        rosbridge_xml,
        web_video,
        http_server
    ])
