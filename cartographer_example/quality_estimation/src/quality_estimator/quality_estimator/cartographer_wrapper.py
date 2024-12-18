import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import PointCloud2, LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster

from .config_manager import ConfigManager

class CartographerWrapper(Node):
    def __init__(self):
        super().__init__('cartographer_wrapper')
        
        # Initialize config manager
        self.config_manager = ConfigManager()
        
        # Declare parameters
        self.declare_parameter('config_file_path', '/workspace/configuration_files/uzh_tracking_area_run2_3D.lua')
        
        # Get parameters
        self.config_file_path = self.get_parameter('config_file_path').get_parameter_value().string_value
        self.optimize_every_n_nodes = self.get_parameter('optimize_every_n_nodes').get_parameter_value().integer_value
        self.use_laser_scan = self.get_parameter('use_laser_scan').get_parameter_value().bool_value
        
        # Load configuration
        self.load_configuration()
        
        # Initialize transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Set up publishers
        self.map_publisher = self.create_publisher(
            OccupancyGrid,
            'map',
            10
        )
        
        self.trajectory_publisher = self.create_publisher(
            PoseStamped,
            'trajectory',
            10
        )
        
        # Set up subscribers based on input type
        if self.use_laser_scan:
            self.scan_subscriber = self.create_subscription(
                LaserScan,
                self.get_parameter('scan_topic').get_parameter_value().string_value,
                self.laser_scan_callback,
                10
            )
        else:
            self.pointcloud_subscriber = self.create_subscription(
                PointCloud2,
                self.get_parameter('pointcloud_topic').get_parameter_value().string_value,
                self.pointcloud_callback,
                10
            )
        
        self.get_logger().info("Cartographer Wrapper initialized successfully")

    def load_configuration(self):
        """Load and configure Cartographer settings."""
        try:
            self.config_manager.load_config(self.config_file_path)
            self.get_logger().info(f"Loaded configuration from: {self.config_file_path}")
            
            # Set optimization parameter
            original_value = self.config_manager['map_builder.pose_graph.optimize_every_n_nodes']
            self.config_manager["map_builder.pose_graph.optimize_every_n_nodes"] = self.optimize_every_n_nodes
            
            self.get_logger().info(f"Updated optimize_every_n_nodes from {original_value} to {self.optimize_every_n_nodes}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load configuration: {e}")
            raise

    def laser_scan_callback(self, msg: LaserScan):
        """Handle incoming laser scan data."""
        try:
            # TODO: Process laser scan data with Cartographer
            self.get_logger().debug("Processing laser scan data")
            self.update_map()
        except Exception as e:
            self.get_logger().error(f"Error processing laser scan: {e}")

    def pointcloud_callback(self, msg: PointCloud2):
        """Handle incoming pointcloud data."""
        try:
            # TODO: Process pointcloud data with Cartographer
            self.get_logger().debug("Processing pointcloud data")
            self.update_map()
        except Exception as e:
            self.get_logger().error(f"Error processing pointcloud: {e}")

    def update_map(self):
        """Update and publish the current map."""
        try:
            # TODO: Get updated map from Cartographer
            # TODO: Publish map and trajectory
            pass
        except Exception as e:
            self.get_logger().error(f"Error updating map: {e}")

    def get_configuration(self) -> str:
        """Get current configuration as LUA string."""
        return self.config_manager.to_lua_string("options")

def main(args=None):
    rclpy.init(args=args)
    
    cartographer_wrapper = CartographerWrapper()
    executor = MultiThreadedExecutor()
    executor.add_node(cartographer_wrapper)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        cartographer_wrapper.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 