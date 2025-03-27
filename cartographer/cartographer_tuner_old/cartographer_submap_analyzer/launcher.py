"""Launcher for the Submap Analyzer GUI."""

import os
import sys
import time
import threading
import asyncio
import queue
import logging
import argparse
from pathlib import Path
import tempfile

# Configure logging first, before any other imports
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] [%(name)s] %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S',
    force=True,  # Force reconfiguration of the root logger
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler('/tmp/submap_analyzer.log')
    ]
)
logger = logging.getLogger('submap_analyzer.launcher')
logger.setLevel(logging.INFO)

# Now import other modules
import rclpy
from rclpy.executors import SingleThreadedExecutor
import tkinter as tk
from launch import LaunchDescription

from .node.analyzer_node import SubmapAnalyzerNode
from .gui.launcher import launch_gui

class SubmapAnalyzerLauncher:
    """Launcher for the Submap Analyzer GUI.
    
    This launcher connects to a running Cartographer node, monitors submaps in real-time,
    and displays them in a GUI for analysis with integrated quality metrics.
    """
    
    def __init__(self, output_dir=None, timeout=30.0, **kwargs):
        """Initialize the submap analyzer launcher.
        
        Args:
            output_dir: Directory to save exported submaps
            timeout: Timeout in seconds for waiting for services and data
            **kwargs: Additional keyword arguments
        """
        # Create output directory if not specified
        if output_dir is None:
            self.output_dir = tempfile.mkdtemp(prefix="cartographer_submaps_")
            print(f"No output directory specified. Using temporary directory: {self.output_dir}")
            logger.info(f"Using temporary directory: {self.output_dir}")
        else:
            self.output_dir = output_dir
            Path(self.output_dir).mkdir(parents=True, exist_ok=True)
            logger.info(f"Using output directory: {self.output_dir}")
        
        self.timeout = timeout
    
    def generate_launch_description(self) -> LaunchDescription:
        """Generate launch description for the submap analyzer.
        
        This launcher doesn't use the launch system, so this method returns an empty
        launch description.
        """
        return LaunchDescription([])
    
    def run(self) -> None:
        """Run the submap analyzer.
        
        This method initializes ROS, creates the submap analyzer node, and starts the GUI.
        """
        logger.info("[TRACK] Starting Submap Analyzer")
        logger.info("[TRACK] Logging is configured and working")
        
        # Initialize ROS
        rclpy.init()
        logger.info("[TRACK] ROS initialized")
        
        # Create update queue for communication between ROS node and GUI
        update_queue = queue.Queue()
        logger.info("[TRACK] Created update queue for communication")
        
        # Create asyncio event loop
        loop = asyncio.new_event_loop()
        logger.info("[TRACK] Created asyncio event loop")
        
        try:
            # Create node with the event loop
            logger.info("[TRACK] Creating SubmapAnalyzerNode")
            node = SubmapAnalyzerNode(self.output_dir, update_queue, loop)
            logger.info("[TRACK] Node created successfully")
            
            # Create executor
            logger.info("[TRACK] Creating SingleThreadedExecutor")
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            logger.info("[TRACK] Executor created and node added")
            
            # Start executor in a separate thread
            logger.info("[TRACK] Starting executor thread")
            executor_thread = threading.Thread(target=executor.spin, daemon=True)
            executor_thread.start()
            logger.info("[TRACK] Executor thread started")
            
            # Set the event loop
            logger.info("[TRACK] Setting event loop")
            asyncio.set_event_loop(loop)
            logger.info("[TRACK] Event loop set")
            
            # Start monitoring task in a separate thread
            logger.info("[TRACK] Starting monitor thread")
            monitor_thread = threading.Thread(
                target=lambda: loop.run_until_complete(node.run()),
                daemon=True
            )
            monitor_thread.start()
            logger.info("[TRACK] Monitor thread started")
            
            # Start the GUI
            logger.info("[TRACK] Starting GUI")
            launch_gui(self.output_dir, update_queue)
            logger.info("[TRACK] GUI closed")
            
        except Exception as e:
            logger.error(f"Error in main loop: {e}", exc_info=True)
            raise
        finally:
            # Ensure ROS is shutdown
            if rclpy.ok():
                rclpy.shutdown()
                logger.info("ROS shutdown")
            
            # Close the event loop
            if 'loop' in locals() and loop.is_running():
                loop.stop()
                logger.info("Event loop stopped")


def main():
    """Command line entry point for the submap analyzer."""
    parser = argparse.ArgumentParser(
        description="Analyze Cartographer submaps in real-time with quality metrics."
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        help="Directory to save exported submaps",
        default=None
    )
    parser.add_argument(
        "--timeout",
        type=float,
        help="Timeout in seconds for waiting for services and data",
        default=30.0
    )
    
    args = parser.parse_args()
    
    # Create and run the launcher
    launcher = SubmapAnalyzerLauncher(
        output_dir=args.output_dir,
        timeout=args.timeout
    )
    launcher.run()


if __name__ == "__main__":
    main() 