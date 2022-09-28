from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

"""
    Launch this to test the system without physically moving the robot.
    Runs all functionality except the motor controller
"""
def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('navigation'),
        'config',
        'nav_params.yaml'
    )

    global_time_node = Node(
        package="rover_utils",
        executable="time",
    )
    waypoint_service = Node(
        package="rover_utils",
        executable="waypoint_service",
    )
    gps_node = Node(
        package="navigation",
        executable="gps",
    )

    localization_node = Node(
        package="navigation",
        executable="localization",
        name='localization',
        parameters = [config]
    )
    navigator_node = Node(
        package="navigation",
        executable="navigator",
        name='navigator',
        parameters = [config]
    )
    trip_planner_node = Node(
        package="navigation",
        executable="trip_planner",
    )
    mission_ctrl_node = Node(
        package="navigation",
        executable="mission_ctrl",
    )

    ld.add_action(global_time_node)
    ld.add_action(waypoint_service)
    ld.add_action(gps_node)

    ld.add_action(localization_node)
    ld.add_action(navigator_node)
    ld.add_action(trip_planner_node)
    ld.add_action(mission_ctrl_node)

    return ld