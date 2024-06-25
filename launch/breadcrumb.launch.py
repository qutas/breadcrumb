#!/usr/bin/env python3

from typing import Union

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from typing import Union


def get_settable_arg(name:str, description:str, default_value:Union[str|int|float,bool,None] = None):
    cfg = LaunchConfiguration(name, default=default_value)
    arg = DeclareLaunchArgument(
        name,
        default_value=str(default_value),
        description=description
    )
    return (cfg, arg)


def generate_launch_description():
    (cfg_search_heuristic, arg_search_heuristic) = get_settable_arg(
        "search_heuristic",
        "Search heuristic to us on grid (manhattan, euclidean, octagonal)",
        default_value='euclidean'
    )
    (cfg_allow_diagonals, arg_allow_diagonals) = get_settable_arg(
        "allow_diagonals",
        "Allow diagonal movements in the solution",
        default_value=True
    )
    (cfg_obstacle_threshold, arg_obstacle_threshold) = get_settable_arg(
        "obstacle_threshold",
        "Threshold to consider an obstacle valid in the occupancy map",
        default_value=50
    )
    (cfg_calc_sparse_path, arg_calc_sparse_path) = get_settable_arg(
        "calc_sparse_path",
        "Calculate a sparse path along with the complete path",
        default_value=True
    )
    (cfg_any_angle, arg_any_angle) = get_settable_arg(
        "any_angle",
        "Also use the Theta* 'Any Angle' method for line-of-sight movement",
        default_value=True
    )

    return LaunchDescription([
        # arg_devices,
        arg_search_heuristic,
        arg_allow_diagonals,
        arg_obstacle_threshold,
        arg_calc_sparse_path,
        arg_any_angle,
        Node(
            name='breadcrumb',
            namespace='',
            package='breadcrumb',
            executable='breadcrumb',
            parameters=[
                {"search_heuristic": cfg_search_heuristic},
                {"allow_diagonals": cfg_allow_diagonals},
                {"obstacle_threshold": cfg_obstacle_threshold},
                {"calc_sparse_path": cfg_calc_sparse_path},
                {"any_angle": cfg_any_angle},
            ],
            remappings=[
                ('~/grid', 'grid'),
            ]
        ),
    ])

