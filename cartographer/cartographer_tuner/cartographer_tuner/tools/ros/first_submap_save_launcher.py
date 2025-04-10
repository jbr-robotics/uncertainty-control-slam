from launch import LaunchDescription
from launch_ros.actions import Node as LaunchNode

from cartographer_tuner.tools.ros.base_ros_tool import BaseRosTool
from cartographer_tuner.submap_analyzer.submap_subscriber_node import SubmapSubscriberNode
import rclpy
from rclpy.node import Node
from cartographer_ros_msgs.msg import SubmapList
import numpy as np
import sys


__all__ = [
    "FirstSubmapSaveLauncher"
]

class FirstSubmapSaveLauncher(BaseRosTool):
    """
    Launcher for saving the first submap from Cartographer.
    """

    @classmethod
    def _register_params(cls):
        cls.register_parameter(
            "output_path",
            str,
            required=True,
            help="Path where to save the first submap"
        )
    
    def generate_launch_description(self) -> LaunchDescription:
        # Create a node to run the SubmapMonitorNode
        # print(f"Path: {sys.path}")
        start_submap_monitor_cmd = [
            "import sys",
            f"sys.path={sys.path}"
            "from cartographer_tuner.tools.ros.first_submap_save_launcher import SubmapMonitorNode",
            "rclpy.init()",
            "node = SubmapMonitorNode()",
            "rclpy.spin(node)",
            "node.destroy_node()",
            "rclpy.shutdown()"
        ]
        # print(f"Cmd: {start_submap_monitor_cmd}")
        submap_monitor_node = LaunchNode(
            package='cartographer_tuner',
            executable=sys.executable,
            name='submap_monitor',
            parameters=[{
                'output_path': self.get_launch_configuration('output_path')
            }],
            arguments=['-c', ';'.join(start_submap_monitor_cmd)],
            output='screen'
        )
        
        description = self.get_launch_arguments() + [
            submap_monitor_node
        ]
        
        return LaunchDescription(description) 
    
class SubmapMonitorNode(Node):
    def __init__(self):
        super().__init__('submap_monitor')
        
        self.declare_parameter('output_path')
        self._output_path = self.get_parameter('output_path').value
        
        self._has_multiple_submaps = False
        self._saved_first_submap = False
        
        self._submap_subscriber = SubmapSubscriberNode(
            list_update_callback=self._on_submap_list,
            submap_update_callback=self._on_submap_data
        )
        
        self.get_logger().info(f"SubmapMonitorNode initialized, will save first submap to {self._output_path}")
    
    def _on_submap_list(self, submap_list: SubmapList):
        num_submaps = len(submap_list.submap)
        self.get_logger().info(f"Received submap list with {num_submaps} submaps")
        
        if num_submaps > 1 and not self._has_multiple_submaps:
            self._has_multiple_submaps = True
            self.get_logger().info("Multiple submaps detected, will query and save the first one")
            
            trajectory_id = 0
            submap_index = 0
            self.get_logger().info(f"Querying first submap (trajectory {trajectory_id}, index {submap_index})")
            
            submap_data = self._submap_subscriber.query_submap(
                trajectory_id, 
                submap_index
            )
                    
            if submap_data is not None:
                # TODO: Save the submap (placeholder, will be implemented later)
                # os.makedirs(os.path.dirname(self._output_path), exist_ok=True)
                # np.save(self._output_path, submap_data)
                
                self.get_logger().info(f"First submap saved to {self._output_path}")
                self._saved_first_submap = True
                
                self.get_logger().info("First submap saved, terminating...")
                rclpy.shutdown()
            else:
                self.get_logger().error("Failed to query first submap")
    
    def _on_submap_data(self, trajectory_id: int, submap_index: int, submap_data: np.ndarray):
        pass
        # self.get_logger().info(f"Received submap data for trajectory {trajectory_id}, index {submap_index}")
    
    def destroy_node(self):
        self._submap_subscriber.shutdown()
        super().destroy_node()