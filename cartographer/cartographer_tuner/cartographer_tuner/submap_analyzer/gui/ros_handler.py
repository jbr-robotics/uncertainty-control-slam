#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import threading
import time
from typing import Optional

from ..submap_storage import SubmapStorage

class RosHandler:
    """
    Simple handler for ROS communication.
    
    Manages the ROS node, SubmapStorage, and provides thread-safe access to submap data.
    Runs ROS event processing in a separate thread.
    """
    # Class variable to ensure only one instance exists
    _instance = None
    _lock = threading.Lock()
    
    @classmethod
    def get_instance(cls):
        """Get the singleton instance of RosHandler."""
        with cls._lock:
            if cls._instance is None:
                cls._instance = RosHandler()
            return cls._instance
    
    def __init__(self):
        """Initialize the ROS handler."""
        # Initialize ROS if needed, safely handling the case where it's already initialized
        self._init_ros()
            
        # Create a ROS node
        self.node = Node('submap_analyzer_gui')
        self.node.get_logger().info("Starting Submap Analyzer GUI")
        
        # Create the submap storage
        self.storage = SubmapStorage(node=self.node)
        self.node.get_logger().info("Submap Storage initialized")
        
        # Thread control with better synchronization
        self._running = False
        self.thread = None
        self._thread_lock = threading.Lock()
    
    def _init_ros(self):
        """Initialize ROS safely."""
        try:
            if not rclpy.ok():
                rclpy.init()
        except RuntimeError as e:
            # ROS might already be initialized in another context
            if "rclpy.init() called while already initialized" not in str(e):
                raise
    
    def start(self):
        """Start the ROS processing thread safely."""
        with self._thread_lock:
            if self._running:
                return  # Already running
                
            self._running = True
            self.thread = threading.Thread(target=self._ros_thread)
            self.thread.daemon = True
            self.thread.start()
            
            self.node.get_logger().info("ROS processing thread started")
        
    def stop(self):
        """Stop the ROS processing thread safely."""
        with self._thread_lock:
            if not self._running:
                return  # Already stopped
                
            self._running = False
            
        if self.thread is not None:
            self.thread.join(timeout=1.0)
            
        # Clean up ROS resources
        try:
            self.storage.shutdown()
            self.node.destroy_node()
        except Exception as e:
            print(f"Error during shutdown: {e}")
            # Continue with shutdown even if there are errors
        
        self.node.get_logger().info("ROS processing stopped")
    
    def _ros_thread(self):
        """ROS processing thread function with better error handling."""
        while self._running and rclpy.ok():
            try:
                # Process events for storage
                self.storage.process_events(timeout_sec=0.1)
                
                # Process events for our main node
                rclpy.spin_once(self.node, timeout_sec=0.1)
                
                time.sleep(0.01)
            except Exception as e:
                print(f"Error in ROS thread: {e}")
                time.sleep(1)  # Pause before retrying to avoid tight error loops
                
                # If we've had a fatal error, stop the thread
                if not rclpy.ok():
                    break
    
    def shutdown(self):
        """Shut down ROS completely."""
        self.stop()
        
        # Don't call rclpy.shutdown() as it might be used elsewhere
        # Only shutdown if we're really exiting the application
    
    def __del__(self):
        """Destructor to ensure clean shutdown."""
        self.stop() 