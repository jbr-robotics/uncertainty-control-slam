"""Launcher for the submap analyzer GUI.

This module provides functions to launch the GUI and connect it to the ROS node.
"""

import os
import logging
import queue
import threading
from typing import Optional

from .main_window import SubmapAnalyzerGUI

# Configure logging
logger = logging.getLogger('submap_analyzer.gui.launcher')


def launch_gui(output_dir: str, update_queue: queue.Queue) -> None:
    """Launch the GUI in the main thread.
    
    Args:
        output_dir: Directory where submaps are saved
        update_queue: Queue for communication with the ROS node
    """
    try:
        # Create and run the GUI
        gui = SubmapAnalyzerGUI(output_dir, update_queue)
        gui.run()
    except Exception as e:
        logger.error(f"Error in GUI: {e}", exc_info=True)


def launch_gui_in_thread(output_dir: str, update_queue: queue.Queue) -> threading.Thread:
    """Launch the GUI in a separate thread.
    
    Args:
        output_dir: Directory where submaps are saved
        update_queue: Queue for communication with the ROS node
        
    Returns:
        Thread object running the GUI
    """
    # Create a thread for the GUI
    gui_thread = threading.Thread(
        target=launch_gui,
        args=(output_dir, update_queue),
        daemon=True
    )
    
    # Start the thread
    gui_thread.start()
    logger.info("Started GUI thread")
    
    return gui_thread 