# Modified from upstream to apply Nav2 launch parameters and scope the controller override.
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory('articubot_one')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')

    lifecycle_nodes = ['controller_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower']

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={
                'use_sim_time': use_sim_time,
                'map_subscribe_transient_local': map_subscribe_transient_local,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )
    # Keep the override last for the controller process, including its local costmap.
    # FollowPath and its critics still come from the main parameter file.
    controller_override = ParameterFile(
        RewrittenYaml(
            source_file=os.path.join(bringup_dir, 'config', 'dwb_critics_override.yaml'),
            root_key=namespace,
            param_rewrites={},
        )
    )
    default_bt = os.path.join(
        get_package_share_directory('nav2_bt_navigator'),
        'behavior_trees',
        'navigate_to_pose_w_replanning_and_recovery.xml',
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(bringup_dir, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('default_bt_xml_filename', default_value=default_bt),
        DeclareLaunchArgument('map_subscribe_transient_local', default_value='true'),
        GroupAction(actions=[
            PushRosNamespace(namespace),
            Node(
                package='nav2_controller', executable='controller_server',
                output='screen', parameters=[configured_params, controller_override],
                remappings=remappings,
            ),
            Node(
                package='nav2_planner', executable='planner_server', name='planner_server',
                output='screen', parameters=[configured_params], remappings=remappings,
            ),
            Node(
                package='nav2_behaviors', executable='behavior_server', name='behavior_server',
                output='screen', parameters=[configured_params], remappings=remappings,
            ),
            Node(
                package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
                output='screen', parameters=[
                    configured_params,
                    {'default_nav_to_pose_bt_xml': default_bt_xml_filename},
                ],
                remappings=remappings,
            ),
            Node(
                package='nav2_waypoint_follower', executable='waypoint_follower',
                name='waypoint_follower', output='screen',
                parameters=[configured_params], remappings=remappings,
            ),
            Node(
                package='nav2_lifecycle_manager', executable='lifecycle_manager',
                name='lifecycle_manager_navigation', output='screen', parameters=[
                    {'use_sim_time': use_sim_time, 'autostart': autostart, 'node_names': lifecycle_nodes},
                ],
            ),
        ]),
    ])
