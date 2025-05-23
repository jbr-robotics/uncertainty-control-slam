from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess


def generate_launch_description():
    bag_filename_arg = DeclareLaunchArgument(
        'bag_filename',
        description='Full path to the bag file to play'
    )

    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        output="screen",
        parameters=[{'use_sim_time': True}],
        arguments=[
            "-configuration_directory", PathJoinSubstitution([
                    FindPackageShare("cartographer_ros"),
                    "configuration_files",
                ]),
            "-configuration_basename", "basement_3D.lua",
            "--collect_metrics",
        ],
        # remappings=[
        #     ("/points2", "/os_cloud_node/points"),
        #     ("/imu", "/os_cloud_node/imu")
        # ]
    )

    occupancy_grid_node = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        arguments=["-resolution", "0.05"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution([
                FindPackageShare("cartographer_ros"),
                "configuration_files",
                "basement_3D.rviz"
            ])
        ],
        output="screen",
        name="rviz"
    )

    rosbag_play_process = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag_filename'), '--clock'],
        name='playbag',
        output='screen'
    )


    static_transform_publisher_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "os_imu"]
    )

    return LaunchDescription([
        bag_filename_arg,
        cartographer_node,
        occupancy_grid_node,
        rviz_node,
        rosbag_play_process,
        static_transform_publisher_node
    ])
