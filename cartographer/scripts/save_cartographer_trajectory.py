#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cartographer_ros_msgs.srv import TrajectoryQuery
from geometry_msgs.msg import PoseStamped, Quaternion # For response processing
import sys
import math
import os

def quaternion_to_yaw(q: Quaternion) -> float:
    """
    Convert a geometry_msgs/Quaternion to a Yaw angle (rotation around Z).

    Args:
        q: The Quaternion message.

    Returns:
        The yaw angle in radians.
    """
    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw

def time_to_microseconds(time_msg) -> int:
    """
    Convert a builtin_interfaces/Time message to microseconds since epoch.

    Args:
        time_msg: The builtin_interfaces/Time message.

    Returns:
        Timestamp in microseconds.
    """
    nanoseconds = time_msg.sec * 1_000_000_000 + time_msg.nanosec
    microseconds = nanoseconds // 1_000
    return microseconds


class TrajectorySaver(Node):
    """
    Node to query a Cartographer trajectory and save poses to a file.
    """
    def __init__(self, service_name='/trajectory_query'):
        """
        Initialize the node and create the service client.

        Args:
            service_name: The name of the trajectory_query service.
        """
        super().__init__('trajectory_saver_client')
        self.client = self.create_client(TrajectoryQuery, service_name)
        self.service_name = service_name

        # Wait for the service to become available
        while not self.client.wait_for_service(timeout_sec=2.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for the service. Exiting.')
                # Raise an exception or return a specific status might be better in a library
                # but for a script, exiting is reasonable.
                sys.exit(1)
            self.get_logger().info(f'Service "{self.service_name}" not available, waiting again...')

    def query_and_save_trajectory(self, trajectory_id: int, output_filename: str):
        """
        Sends the trajectory query request and saves the result to a file.

        Args:
            trajectory_id: The integer ID of the trajectory to query.
            output_filename: The path to the file where poses should be saved.
        """
        if not isinstance(trajectory_id, int):
             self.get_logger().error(f"Trajectory ID must be an integer, got: {trajectory_id}")
             return False # Indicate failure

        # Prepare the request
        request = TrajectoryQuery.Request()
        request.trajectory_id = trajectory_id

        self.get_logger().info(f'Calling TrajectoryQuery service for trajectory_id: {trajectory_id}...')

        # Call the service asynchronously and wait for the result
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()

        # Process the response
        if response:
            self.get_logger().info(f"Service call status: Code={response.status.code}, Message='{response.status.message}'")
            if response.status.code == 0: # Success
                # Use the deprecated 'trajectory' field as it directly provides PoseStamped
                # which matches the requested output format (timestamp, x, y, yaw).
                # Reconstructing world poses from 'submap_trajectory' is more complex.
                poses = response.trajectory
                num_poses = len(poses)
                self.get_logger().info(f"Received {num_poses} poses for trajectory {trajectory_id}.")

                if num_poses == 0:
                    self.get_logger().warn(f"Trajectory {trajectory_id} has no poses in the 'trajectory' field. Is it finished/frozen?")
                    return False # Indicate potential issue

                try:
                    # Ensure output directory exists
                    output_dir = os.path.dirname(output_filename)
                    if output_dir and not os.path.exists(output_dir):
                         os.makedirs(output_dir)
                         self.get_logger().info(f"Created output directory: {output_dir}")

                    with open(output_filename, 'w') as f:
                        self.get_logger().info(f"Saving poses to {output_filename}...")
                        for pose_stamped in poses:
                            timestamp_us = time_to_microseconds(pose_stamped.header.stamp)
                            x = pose_stamped.pose.position.x
                            y = pose_stamped.pose.position.y
                            yaw = quaternion_to_yaw(pose_stamped.pose.orientation)
                            # Format: timestamp,x,y,yaw (no spaces)
                            line = f"{timestamp_us},{x:.6f},{y:.6f},{yaw:.6f}\n"
                            f.write(line)
                    self.get_logger().info(f"Successfully saved {num_poses} poses to {output_filename}")
                    return True # Indicate success

                except IOError as e:
                    self.get_logger().error(f"Failed to write to file {output_filename}: {e}")
                    return False # Indicate failure
                except Exception as e:
                    self.get_logger().error(f"An unexpected error occurred during file writing: {e}")
                    return False # Indicate failure

            else:
                 self.get_logger().warn(f"Service call reported an issue for trajectory {trajectory_id}: {response.status.message}")
                 return False # Indicate failure
        else:
            self.get_logger().error(f'Service call to {self.service_name} failed: {future.exception()}')
            return False # Indicate failure


def main(args=None):
    rclpy.init(args=args)

    # --- Argument Parsing ---
    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} <trajectory_id> <output_filename>")
        rclpy.shutdown()
        sys.exit(1)

    try:
        trajectory_id_to_query = int(sys.argv[1])
    except ValueError:
        print(f"Error: Invalid trajectory_id '{sys.argv[1]}'. Must be an integer.")
        rclpy.shutdown()
        sys.exit(1)

    output_file = sys.argv[2]
    # Basic validation for output file path (can be enhanced)
    if not output_file:
         print(f"Error: Output filename cannot be empty.")
         rclpy.shutdown()
         sys.exit(1)

    # --- Node Execution ---
    # You could make the service name an argument too if needed
    trajectory_saver_node = TrajectorySaver(service_name='/trajectory_query')

    success = False
    try:
        success = trajectory_saver_node.query_and_save_trajectory(trajectory_id_to_query, output_file)
    except Exception as e:
         trajectory_saver_node.get_logger().error(f"An unexpected error occurred: {e}", exc_info=True)
    finally:
        # Cleanup
        trajectory_saver_node.destroy_node()
        rclpy.shutdown()
        if success:
             print("Trajectory saved")

if __name__ == "__main__":
    main()
