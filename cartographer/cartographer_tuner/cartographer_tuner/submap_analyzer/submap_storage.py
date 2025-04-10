# This class is likely outdated, but I'm keeping it here for now
# TODO: Remove this file if it's not used

# import rclpy
# from rclpy.node import Node
# import threading
# import numpy as np
# import time
# from collections import defaultdict
# from cartographer_ros_msgs.msg import SubmapList
# from typing import Dict, List, Optional, Tuple

# from .submap_subscriber import SubmapSubscriber

# class SubmapStorage:
#     def __init__(self, node: Optional[Node] = None):
#         self._node = node
#         self._logger = node.get_logger() if node else None
        
#         # Store the latest submap list
#         self._latest_submap_list = None
#         self._submap_list_lock = threading.Lock()
        
#         self._submap_history = defaultdict(list)
#         self._history_lock = threading.Lock()
        
#         self._subscriber = SubmapSubscriber(
#             list_update_callback=self._on_submap_list_update,
#             submap_update_callback=self._on_submap_data_update
#         )
        
#         if self._logger:
#             self._logger.info("SubmapStorage initialized")
#         elif hasattr(self._subscriber, '_node') and self._subscriber._node:
#             # Use subscriber's logger if available
#             self._logger = self._subscriber._node.get_logger()
#             self._logger.info("SubmapStorage initialized")
    
#     def _on_submap_list_update(self, submap_list: SubmapList) -> None:
#         with self._submap_list_lock:
#             self._latest_submap_list = submap_list
    
#     def _on_submap_data_update(self, trajectory_id: int, submap_index: int, 
#                                data: np.ndarray) -> None:
#         submap_key = (trajectory_id, submap_index)
        
#         with self._history_lock:
#             # Store in history
#             self._submap_history[submap_key].append(data)
    
#     def get_latest_submap_list(self) -> Optional[SubmapList]:
#         with self._submap_list_lock:
#             return self._latest_submap_list
    
#     def get_submap_history(self, trajectory_id: int, submap_index: int) -> List[np.ndarray]:
#         submap_key = (trajectory_id, submap_index)
        
#         with self._history_lock:
#             return self._submap_history.get(submap_key, []).copy()
    
#     def get_latest_submap(self, trajectory_id: int, submap_index: int) -> Optional[np.ndarray]:
#         history = self.get_submap_history(trajectory_id, submap_index)
#         return history[-1] if history else None
    
#     def get_all_submap_keys(self) -> List[Tuple[int, int]]:
#         with self._history_lock:
#             return list(self._submap_history.keys())
    
#     def process_events(self, timeout_sec=0.1):
#         self._subscriber.process_events(timeout_sec)
    
#     def shutdown(self):
#         self._subscriber.shutdown()
        
#         if self._logger:
#             self._logger.info("SubmapStorage shutdown complete") 