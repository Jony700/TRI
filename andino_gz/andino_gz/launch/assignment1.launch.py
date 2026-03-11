import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_andino_gz = get_package_share_directory('andino_gz')

    # Include the main andino launch file, but pass our new world explicitly to world_name
    andino_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_andino_gz, 'launch', 'andino_gz.launch.py')
        ),
        # FIXED: The argument must be 'world_name'
        launch_arguments={'world_name': 'assignment1.sdf'}.items()
    )

    return LaunchDescription([
        andino_simulation
    ])
