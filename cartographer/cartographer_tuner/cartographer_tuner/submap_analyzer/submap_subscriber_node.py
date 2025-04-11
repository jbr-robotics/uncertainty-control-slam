import rclpy
from rclpy.node import Node
import numpy as np
import time
from queue import Queue, Empty
from cartographer_ros_msgs.msg import SubmapList
from cartographer_ros_msgs.srv import SubmapQuery
from typing import Callable, Dict, List, Set, Tuple, Optional

from cartographer_tuner.submap_analyzer.submap import Submap

class SubmapSubscriberNode(Node):
    def __init__(
        self,
        list_update_callback: Callable[[SubmapList], None],
        submap_update_callback: Callable[[int, int, Submap], None],
    ):
        super().__init__('submap_subscriber')
        self._logger = self.get_logger()

        self._on_list_update = list_update_callback
        self._on_submap_data = submap_update_callback

        self._previous_submaps = set()
        self._query_queue = Queue()
        self._active_queries = set()
        self._running = True
        self._last_queried_submap = None

        self._ros_submap_subscriber = self.create_subscription(
            SubmapList,
            "/submap_list",
            self._handle_submap_list,
            10
        )

        self._ros_query_client = self.create_client(SubmapQuery, "/submap_query")

        self._logger.info("Waiting for /submap_query service...")
        while not self._ros_query_client.wait_for_service(timeout_sec=1.0) and self._running:
            self._logger.info("Waiting for /submap_query service...")
            if not rclpy.ok():
                raise RuntimeError("Cannot find server")
        

        self._logger.debug("SubmapSubscriber initialized")

    def query_submap(self, trajectory_id: int, submap_index: int) -> Optional[Submap]:
        self._logger.debug(f"Querying submap ({trajectory_id}, {submap_index})")
        
        request = SubmapQuery.Request()
        request.trajectory_id = trajectory_id
        request.submap_index = submap_index

        try:
            future = self._ros_query_client.call_async(request)
            
            start_time = time.time()
            while not future.done() and (time.time() - start_time) < 5.0:
                rclpy.spin_once(self, timeout_sec=0.1)
                time.sleep(0.01)
                
            if not future.done():
                self._logger.warn(f"Timeout waiting for submap query response")
                return None
                
            response = future.result()
        except Exception as e:
            self._logger.warn(f"Error calling service for query: {e}")
            return None

        try:
            submap = Submap.from_SubmapQuery(response)
        except Exception as e:
            self._logger.warn(f"Could not create Submap object for ({trajectory_id}, {submap_index}): {e}")
            return None

        if self._on_submap_data:
            self._on_submap_data(trajectory_id, submap_index, submap)
            
        self._logger.debug(f"Processed submap ({trajectory_id}, {submap_index})")
        
        return submap

    def process_events(self, timeout_sec=0.1):
        """
        Should be called regularly in main loop.
        Handles both ROS events and pending submap queries.
        """
        if not self._running or not rclpy.ok():
            return

        # Process ROS callbacks
        rclpy.spin_once(self, timeout_sec=timeout_sec)

        # Process one submap query per call
        try:
            submap_key = self._query_queue.get_nowait()

            if submap_key not in self._active_queries:
                self._active_queries.add(submap_key)
                self._process_submap(submap_key)
                self._active_queries.discard(submap_key)

            self._query_queue.task_done()
        except Empty:
            pass

    def _handle_submap_list(self, submap_list: SubmapList) -> None:
        try:
            if self._on_list_update:
                try:
                    self._on_list_update(submap_list)
                except Exception as e:
                    self._logger.error(f"Error in list update callback: {e}")

            current_submaps = self._extract_submap_keys(submap_list)

            new_submaps = current_submaps - self._previous_submaps
            
            if new_submaps:
                if len(new_submaps) > 1:
                    self._logger.warn(f"Found {len(new_submaps)} new submaps in one update. Querying one.")
                new_submap = list(sorted(new_submaps))[0]
                self._last_queried_submap = new_submap
                self._query_queue.put(new_submap)
            elif self._last_queried_submap:
                self._logger.debug(f"No new submaps, re-querying last known submap ({self._last_queried_submap[0]}, {self._last_queried_submap[1]})")
                self._query_queue.put(self._last_queried_submap)

            self._previous_submaps = current_submaps
        except Exception as e:
            self._logger.error(f"Error handling submap list: {e}")

    def _extract_submap_keys(self, submap_list: SubmapList) -> Set[Tuple[int, int]]:
        result = set()
        assert hasattr(submap_list, 'submap'), "SubmapList message does not have 'submap' field"
        for submap in submap_list.submap:
            result.add((submap.trajectory_id, submap.submap_index))
        return result

    def _process_submap(self, submap_key: Tuple[int, int]) -> None:
        trajectory_id, submap_index = submap_key
        self._logger.debug(f"Processing submap ({trajectory_id}, {submap_index})")
        self.query_submap(trajectory_id, submap_index)

    def shutdown(self) -> None:
        """Clean up resources."""
        self._running = False

        if hasattr(self, '_ros_submap_subscriber'):
            self.destroy_subscription(self._ros_submap_subscriber)

        if hasattr(self, '_ros_query_client'):
            self.destroy_client(self._ros_query_client)

        self._logger.debug("SubmapSubscriber shutdown complete")
 