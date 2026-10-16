"""10 x 10 m obstacle arena, mecanum robot, sensor SLAM and Nav2 Omni."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            TimerAction, RegisterEventHandler, EmitEvent)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share = Path(get_package_share_directory('nav_stack'))
    output = Path(LaunchConfiguration('output_dir').perform(context)).resolve()
    output.mkdir(parents=True, exist_ok=True)
    cfg = yaml.safe_load((share/'config/omni_arena.yaml').read_text())
    rviz = yaml.safe_load((share/'config/rviz.rviz').read_text())
    rviz['Visualization Manager']['Displays'][0]['Name'] = 'MID360 mecanum robot'
    rviz_file = output/'omni.rviz'
    rviz_file.write_text(yaml.safe_dump(rviz))
    simulator = Node(package='nav_stack', executable='simulator', name='omni_simulator', output='screen',
        parameters=[dict(world_type='omni_arena', field_path=str(share/'worlds/obstacle_arena_10x10.xml'),
            robot_path=str(share/'urdf/omni_mid360.urdf'), output_dir=str(output), rays=12000,
            spawn_x=float(cfg['spawn'][0]), spawn_y=float(cfg['spawn'][1]), spawn_yaw=float(cfg['spawn'][2]),
            gui=LaunchConfiguration('gui'), realtime_factor=LaunchConfiguration('realtime_factor'))])
    return [simulator,
        RegisterEventHandler(OnProcessExit(target_action=simulator,
            on_exit=[EmitEvent(event=Shutdown(reason='Omni simulator exited'))])),
        Node(package='fast_lio', executable='fastlio_mapping', name='fastlio_mapping', output='screen',
            parameters=[str(share/'config/simulation.yaml'), dict(use_sim_time=True,
                        filter_size_surf=.10,filter_size_map=.10)]),
        Node(package='nav_stack', executable='fastlio_map', name='fastlio_map', output='screen',
            parameters=[str(share/'config/terrain.yaml'), dict(use_sim_time=True,
                output_dir=str(output/'slam'), save_on_shutdown=True, voxel_size=.04,
                imu_height=.15, map_display_height=-.15,
                observed_footprint_length=.54, observed_footprint_width=.48)]),
        Node(package='nav_stack', executable='bridge', output='screen',
            parameters=[str(share/'config/simulation.yaml'), str(share/'config/terrain.yaml'),
                dict(use_sim_time=True, imu_height=.15, publish_sensor_tf=False)]),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
            parameters=[dict(use_sim_time=True, robot_description=(share/'urdf/omni_mid360.urdf').read_text())]),
        TimerAction(period=6., actions=[
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(
                get_package_share_directory('nav2_bringup'))/'launch/navigation_launch.py')),
                launch_arguments=dict(use_sim_time='true', params_file=str(share/'config/nav2_omni.yaml'),
                                      use_composition='False', autostart='true').items())]),
        Node(package='rviz2', executable='rviz2', name='omni_navigation_rviz', output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')), arguments=['-d',str(rviz_file)],
            parameters=[{'use_sim_time':True}])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('output_dir', default_value='artifacts/omni_navigation'),
        DeclareLaunchArgument('gui',default_value='true'),
        DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('realtime_factor',default_value='1.0'),
        OpaqueFunction(function=setup)])
