"""Dedicated Manual Relocalization Demo: MuJoCo non-origin spawn, FAST-LIO2, Localizer and RViz."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share = Path(get_package_share_directory('nav_stack'))
    output = Path(LaunchConfiguration('output_dir').perform(context)).resolve()
    map_pcd = Path(LaunchConfiguration('map_pcd').perform(context)).resolve()
    map_yaml = Path(LaunchConfiguration('map_yaml').perform(context)).resolve()
    output.mkdir(parents=True, exist_ok=True)

    spawn_x = float(LaunchConfiguration('spawn_x').perform(context))
    spawn_y = float(LaunchConfiguration('spawn_y').perform(context))
    spawn_yaw = float(LaunchConfiguration('spawn_yaw').perform(context))

    rviz_file = share/'config/rviz_relocalize.rviz'

    simulator = Node(
        package='nav_stack', executable='simulator', name='omni_simulator', output='screen',
        parameters=[dict(
            world_type='omni_arena',
            field_path=str(share/'worlds/obstacle_arena_10x10.xml'),
            robot_path=str(share/'urdf/omni_mid360.urdf'),
            output_dir=str(output),
            rays=12000,
            spawn_x=spawn_x,
            spawn_y=spawn_y,
            spawn_yaw=spawn_yaw,
            gui=LaunchConfiguration('gui'),
            realtime_factor=LaunchConfiguration('realtime_factor')
        )]
    )

    return [
        simulator,
        RegisterEventHandler(OnProcessExit(
            target_action=simulator,
            on_exit=[EmitEvent(event=Shutdown(reason='Omni simulator exited'))]
        )),
        Node(
            package='fast_lio', executable='fastlio_mapping', name='fastlio_mapping', output='screen',
            parameters=[str(share/'config/simulation.yaml'), dict(
                use_sim_time=True, filter_size_surf=.10, filter_size_map=.10
            )]
        ),
        # Prior Map Relocalization Node with manual_mode=True
        Node(
            package='nav_stack', executable='localizer', name='prior_map_localizer', output='screen',
            parameters=[dict(
                use_sim_time=True,
                map_pcd=str(map_pcd),
                map_yaml=str(map_yaml),
                imu_height=0.15,
                initial_pose_x=0.0,
                initial_pose_y=0.0,
                initial_pose_yaw=0.0,
                manual_mode=True
            )]
        ),
        Node(
            package='nav_stack', executable='bridge', output='screen',
            parameters=[str(share/'config/simulation.yaml'), str(share/'config/terrain.yaml'),
                dict(use_sim_time=True, imu_height=.15, publish_sensor_tf=False)]
        ),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher', output='screen',
            parameters=[dict(use_sim_time=True, robot_description=(share/'urdf/omni_mid360.urdf').read_text())]
        ),
        Node(
            package='rviz2', executable='rviz2', name='relocalize_rviz', output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', str(rviz_file)],
            parameters=[{'use_sim_time': True}]
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('output_dir', default_value='artifacts/relocalize'),
        DeclareLaunchArgument('map_pcd', default_value='maps/map.pcd'),
        DeclareLaunchArgument('map_yaml', default_value='maps/map.yaml'),
        DeclareLaunchArgument('spawn_x', default_value='-2.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.523'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('realtime_factor', default_value='1.0'),
        OpaqueFunction(function=setup)
    ])
