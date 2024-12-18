import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import socket

from .config_manager import ConfigManager


# class QualityEstimatorNode(Node):
#     def __init__(self):
#         super().__init__('quality_estimator_node')

#         # Declare and get parameters
#         self.declare_parameter('topic_name', '/default_topic')
#         topic_name = self.get_parameter('topic_name').get_parameter_value().string_value

#         # Create subscription to sensor_msgs/Image
#         self.subscription = self.create_subscription(
#             Image,
#             topic_name,
#             self.listener_callback,
#             10)
#         self.get_logger().info(f"TCP Sender Node started and subscribed to topic: {topic_name}")

#         # Set up TCP client
#         self.socket = None
#         self.start_tcp_client()

#     def start_tcp_client(self):
#         host = '0.0.0.0'  # The IP address or hostname of the receiver
#         port = 65432  # Port used by the receiver
#         try:
#             self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#             # Wait until the receiver is ready
#             while True:
#                 try:
#                     self.socket.connect((host, port))
#                     self.get_logger().info("Connected to receiver")
#                     break
#                 except socket.error:
#                     self.get_logger().info('Waiting for receiver to be ready...')
#         except Exception as e:
#             self.get_logger().error(f"Failed to create socket: {e}")

#     def listener_callback(self, msg: Image):
#         # Extract image metadata and data
#         image_data = msg.data
#         height = msg.height
#         width = msg.width
#         encoding = msg.encoding

#         # Log image metadata
#         self.get_logger().info(f"Received image: {height}x{width}, encoding: {encoding}")

#         if self.socket:
#             try:

#                 # Prepare metadata
#                 metadata = struct.pack('!IIH', height, width, len(encoding)) + encoding.encode('utf-8')

#                 # Combine metadata and image data
#                 payload = metadata + image_data

#                 # Send the total length of the payload first
#                 total_length = len(payload)
#                 self.socket.send(struct.pack('!I', total_length))  # Send total length as 4 bytes
#                 self.socket.send(payload)  # Send the actual payload

#                 self.get_logger().info("Sent image data with length prefix")
#             except Exception as e:
#                 self.get_logger().error(f"Failed to send data: {e}")

def main(args=None):
    rclpy.init(args=args)
    # quality_estimator_node = QualityEstimatorNode()
    # executor = MultiThreadedExecutor()
    # executor.add_node(quality_estimator_node)
    # try:
    #     executor.spin()
    # finally:
        # if quality_estimator_node.socket:
            # quality_estimator_node.socket.close()
        # quality_estimator_node.destroy_node()
        # rclpy.shutdown()
    config_manager = ConfigManager()
    config_manager.load_config("/workspace/configuration_files/uzh_tracking_area_run2_3D.lua")
    print(f"Original optimize_every_n_nodes: {config_manager['map_builder.pose_graph.optimize_every_n_nodes']}")
    config_manager["map_builder.pose_graph.optimize_every_n_nodes"] = -100
    print(config_manager.to_lua_string("options"))
    
if __name__ == '__main__':
    main()