"""Cave profile of the projector chain.

Thin wrapper around ``go2_pointlio_projected.launch.py`` that sets the cave
defaults in one place, so a cave run does not depend on remembering a list of
launch arguments:

  * ``projector_params_file`` -> ``projector_params_cave.yaml``
    (profile=cave: tighter seed/ring/ref bands for a 1.5 x 1.5 m arched
    tunnel, plus the optional clearance layer)
  * ``slope_fit_cells:=1`` -> slope from a least-squares plane over a 0.3 m
    window instead of a two-cell gradient. In a 1.5 m corridor a 0.9-1.1 m
    window would average both walls into the floor; 0.3 m is the largest
    window that still stays inside the corridor.
  * ``rviz_config`` -> the slope-coloured cave view

Everything remains overridable from the command line, e.g.
``ros2 launch pgo_gridmap_projector go2_pointlio_projected_cave.launch.py
slope_fit_cells:=2``.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory('pgo_gridmap_projector')
    elevation_share = get_package_share_directory('elevation_mapping')

    cave_params = os.path.join(share, 'config', 'projector_params_cave.yaml')
    cave_rviz = os.path.join(
        elevation_share, 'rviz2', 'go2_elevmap_fastlio_cave.rviz')

    projector_params_file_arg = DeclareLaunchArgument(
        'projector_params_file', default_value=cave_params,
        description='Cave defaults: reference + profile=cave + clearance.')
    slope_fit_cells_arg = DeclareLaunchArgument(
        'slope_fit_cells', default_value='1',
        description='Slope window in cells: 1 -> 0.3 m plane fit (cave).')
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value=cave_rviz,
        description='Slope-coloured cave RViz view.')

    include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch',
                         'go2_pointlio_projected.launch.py')),
        launch_arguments=[
            ('projector_params_file', LaunchConfiguration(
                'projector_params_file')),
            ('slope_fit_cells', LaunchConfiguration('slope_fit_cells')),
            ('rviz_config', LaunchConfiguration('rviz_config')),
        ],
    )

    return LaunchDescription(
        [projector_params_file_arg, slope_fit_cells_arg, rviz_config_arg,
         include])
