from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
from launch.actions import Shutdown

def generate_launch_description():
    bag_filename_arg = DeclareLaunchArgument(
        'bag_filename',
        description='Full path to the bag file to play'
    )

    cartographer_offline_node = Node(
        package="cartographer_ros",
        executable="cartographer_offline_node",
        on_exit = Shutdown(),
        output="screen",
        parameters=[{'use_sim_time': True}],
        arguments=[
            "-configuration_directory", PathJoinSubstitution([
                    FindPackageShare("cartographer_ros"),
                    "configuration_files",
                ]),
            "-configuration_basenames", "uzh_tracking_area_run2_3D.lua",
            "-bag_filenames", LaunchConfiguration('bag_filename'),
            "--collect_metrics",
        ],
        remappings=[
            ("/points2", "/os_cloud_node/points"),
            ("/imu", "/os_cloud_node/imu")
        ]
    )

    occupancy_grid_node = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        arguments=["-resolution", "0.05"]
    )

    return LaunchDescription([
        bag_filename_arg,
        cartographer_offline_node,
        occupancy_grid_node,
        # rviz_node,
        # rosbag_play_process
    ])
