from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import XMLLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('tictactoe_bringup')
    webpage_dir = os.path.join(pkg_share, 'webpage')

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('piper_with_gripper_moveit'),
                'launch',
                'move_group.launch.py'
            )
        )
    )

    board_processor = Node(
        package='board_perception',
        executable='board_processor_server',
        name='board_processor',
        output='screen'
    )

    trajectory_executor = Node(
        package='piper_trajectory',
        executable='trajectory_executor',
        name='trajectory_executor',
        output='screen'
    )

    move_manager = Node(
        package='tictactoe_manager',
        executable='tictactoe_manager',
        name='tictactoe_manager',
        output='screen'
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
        moveit_launch,
        board_processor,
        trajectory_executor,
        move_manager,
        rosbridge_xml,
        web_video,
        http_server
    ])
