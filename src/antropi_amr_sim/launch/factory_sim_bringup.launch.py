import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    pkg_dir = get_package_share_directory('antropi_amr_sim')
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    tb3_nav2_dir = get_package_share_directory('turtlebot3_navigation2')

    # Resolve default paths via ament_index (no hardcoded paths)
    default_map_path = os.path.join(tb3_nav2_dir, 'map', 'map.yaml')
    default_params_path = os.path.join(pkg_dir, 'config', 'nav2_factory_params.yaml')

    # --- Declare Launch Arguments ---
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock'
    )
    map_arg = DeclareLaunchArgument(
        'map', default_value=default_map_path,
        description='Full path to map YAML file'
    )
    params_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params_path,
        description='Full path to Nav2 parameters YAML file'
    )

    # --- 1. Gazebo Simulation (TurtleBot3 World with cylindrical pillars) ---
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # --- 2. Nav2 Stack + RViz2 (delayed 5s for Gazebo /clock to initialize) ---
    nav2_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(tb3_nav2_dir, 'launch', 'navigation2.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'map': LaunchConfiguration('map'),
                    'params_file': LaunchConfiguration('params_file'),
                }.items()
            )
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_arg,
        params_arg,
        gazebo_launch,
        nav2_launch,
    ])
