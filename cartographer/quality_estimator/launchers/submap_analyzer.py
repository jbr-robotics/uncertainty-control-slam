#!/usr/bin/env python3

import os
import time
import threading
import asyncio
from pathlib import Path
from typing import Dict, List, Optional, Any, Tuple, Set
import tempfile
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from launch import LaunchDescription
import numpy as np
from PIL import Image, ImageTk
import tkinter as tk
from tkinter import ttk
import io
import gzip
import queue
import logging

from cartographer_ros_msgs.srv import SubmapQuery
from cartographer_ros_msgs.msg import SubmapList, SubmapEntry

from .base import BaseLauncher
from .submap_exporter import SubmapExporterNode

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler('/tmp/submap_analyzer.log')
    ]
)
logger = logging.getLogger('submap_analyzer')


class SubmapAnalyzerNode(SubmapExporterNode):
    """ROS2 node for analyzing submaps in real-time.
    
    This extends the SubmapExporterNode to add real-time monitoring of submaps.
    """
    
    def __init__(self, output_dir: str, update_queue: queue.Queue, loop: asyncio.AbstractEventLoop):
        """Initialize the submap analyzer node.
        
        Args:
            output_dir: Directory to save exported submaps
            update_queue: Queue to send updates to the GUI
            loop: Asyncio event loop to use for background tasks
        """
        super().__init__(output_dir)
        
        # Queue for sending updates to the GUI
        self.update_queue = update_queue
        
        # Store the event loop
        self.loop = loop
        
        # Track which submaps we've already processed
        self.processed_submaps = set()
        
        # Create a separate callback group for the timer
        self.timer_callback_group = MutuallyExclusiveCallbackGroup()
        
        # Create a timer for periodic updates
        self.update_timer = self.create_timer(
            1.0, 
            self.check_for_new_submaps,
            callback_group=self.timer_callback_group
        )
        
        # Flag to indicate if we're currently processing submaps
        self.is_processing = False
        
        self.get_logger().info("Submap analyzer node initialized")
        logger.info("Submap analyzer node initialized")
    
    def submap_list_callback(self, msg: SubmapList):
        """Callback for submap list messages."""
        self.submap_list = msg
        num_submaps = len(msg.submap)
        self.get_logger().info(f"Received submap list with {num_submaps} submaps")
        logger.info(f"Received submap list with {num_submaps} submaps")
        
        # Notify the GUI that we have a new submap list
        self.update_queue.put(("submap_list_updated", num_submaps))
    
    def check_for_new_submaps(self):
        """Check for new submaps and process them."""
        if self.submap_list is None:
            logger.debug("No submap list available yet")
            return
            
        if self.is_processing:
            logger.debug("Already processing submaps, skipping check")
            return
            
        # Get list of submaps we haven't processed yet
        new_submaps = []
        for submap_entry in self.submap_list.submap:
            submap_id = (submap_entry.trajectory_id, submap_entry.submap_index)
            if submap_id not in self.processed_submaps:
                new_submaps.append(submap_entry)
        
        if new_submaps:
            num_new = len(new_submaps)
            self.get_logger().info(f"Found {num_new} new submaps to process")
            logger.info(f"Found {num_new} new submaps to process")
            
            # Schedule the processing in the event loop
            self.is_processing = True
            asyncio.run_coroutine_threadsafe(self.process_new_submaps(new_submaps), self.loop)
    
    async def process_new_submaps(self, new_submaps: List[SubmapEntry]):
        """Process new submaps in the background.
        
        Args:
            new_submaps: List of new submaps to process
        """
        try:
            for submap_entry in new_submaps:
                try:
                    trajectory_id = submap_entry.trajectory_id
                    submap_index = submap_entry.submap_index
                    submap_id = (trajectory_id, submap_index)
                    
                    # Skip if we've already processed this submap
                    if submap_id in self.processed_submaps:
                        continue
                    
                    logger.info(f"Processing new submap: trajectory={trajectory_id}, index={submap_index}")
                    self.get_logger().info(f"Processing new submap: trajectory={trajectory_id}, index={submap_index}")
                    
                    # Query submap data
                    submap_data = await self.query_submap(trajectory_id, submap_index)
                    
                    if submap_data:
                        logger.info(f"Successfully queried submap {trajectory_id}_{submap_index}")
                        
                        # Save the submap as PGM
                        saved_file = self.save_submap_as_pgm(submap_data)
                        
                        if saved_file:
                            logger.info(f"Saved submap to {saved_file}")
                            
                            # Mark as processed
                            self.processed_submaps.add(submap_id)
                            
                            # Notify the GUI
                            self.update_queue.put(("new_submap", {
                                'trajectory_id': trajectory_id,
                                'submap_index': submap_index,
                                'file_path': saved_file,
                                'resolution': submap_data['resolution'],
                                'width': submap_data['width'],
                                'height': submap_data['height']
                            }))
                            logger.info(f"Notified GUI about new submap {trajectory_id}_{submap_index}")
                    else:
                        logger.warning(f"Failed to query submap {trajectory_id}_{submap_index}")
                    
                    # Add a small delay between submaps to avoid overwhelming the system
                    await asyncio.sleep(0.1)
                    
                except Exception as e:
                    error_msg = f"Error processing submap {submap_entry.trajectory_id}_{submap_entry.submap_index}: {e}"
                    self.get_logger().error(error_msg)
                    logger.error(error_msg, exc_info=True)
        finally:
            self.is_processing = False
    
    async def monitor_submaps(self):
        """Monitor for new submaps continuously.
        
        This method runs indefinitely, checking for new submaps and processing them.
        """
        logger.info("Starting submap monitoring")
        
        # Wait for service and initial submap list
        if not await self.wait_for_service():
            error_msg = "Failed to connect to submap query service"
            self.get_logger().error(error_msg)
            logger.error(error_msg)
            return
        
        if not await self.wait_for_submap_list():
            error_msg = "Failed to receive initial submap list"
            self.get_logger().error(error_msg)
            logger.error(error_msg)
            return
        
        self.get_logger().info("Starting continuous submap monitoring")
        logger.info("Starting continuous submap monitoring")
        
        # Initial processing of existing submaps
        self.is_processing = True
        try:
            await self.process_new_submaps(self.submap_list.submap)
        finally:
            self.is_processing = False
        
        # Continue running to process timer callbacks
        while rclpy.ok():
            await asyncio.sleep(0.1)


class SubmapAnalyzerGUI:
    """GUI for displaying and analyzing submaps in real-time."""
    
    def __init__(self, root: tk.Tk, output_dir: str, update_queue: queue.Queue):
        """Initialize the GUI.
        
        Args:
            root: Tkinter root window
            output_dir: Directory where submaps are saved
            update_queue: Queue for receiving updates from the ROS node
        """
        self.root = root
        self.output_dir = Path(output_dir)
        
        # Set up the main window
        self.root.title("Cartographer Submap Analyzer")
        self.root.geometry("1200x800")
        
        # Store the update queue
        self.update_queue = update_queue
        
        # Track submaps and their images
        self.submaps = {}  # (trajectory_id, submap_index) -> submap data
        self.submap_images = {}  # (trajectory_id, submap_index) -> PhotoImage
        
        # Currently selected submap
        self.selected_submap = None
        
        # Create the GUI layout
        self._create_layout()
        
        # Start periodic check for updates
        self.root.after(100, self._check_updates)
        
        logger.info("GUI initialized")
    
    def _create_layout(self):
        """Create the GUI layout."""
        # Create main frame with padding
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Create left panel for submap list
        left_panel = ttk.Frame(main_frame, width=300)
        left_panel.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        
        # Create right panel for submap display
        right_panel = ttk.Frame(main_frame)
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        # Create submap list with scrollbar
        list_frame = ttk.Frame(left_panel)
        list_frame.pack(fill=tk.BOTH, expand=True)
        
        list_scrollbar = ttk.Scrollbar(list_frame)
        list_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        self.submap_listbox = tk.Listbox(list_frame, yscrollcommand=list_scrollbar.set, font=("TkDefaultFont", 10))
        self.submap_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        list_scrollbar.config(command=self.submap_listbox.yview)
        
        # Bind selection event
        self.submap_listbox.bind('<<ListboxSelect>>', self._on_submap_selected)
        
        # Create status label
        self.status_label = ttk.Label(left_panel, text="Waiting for submaps...")
        self.status_label.pack(side=tk.BOTTOM, fill=tk.X, pady=(10, 0))
        
        # Create canvas for submap display with scrollbars
        canvas_frame = ttk.Frame(right_panel)
        canvas_frame.pack(fill=tk.BOTH, expand=True)
        
        self.canvas_h_scrollbar = ttk.Scrollbar(canvas_frame, orient=tk.HORIZONTAL)
        self.canvas_h_scrollbar.pack(side=tk.BOTTOM, fill=tk.X)
        
        self.canvas_v_scrollbar = ttk.Scrollbar(canvas_frame)
        self.canvas_v_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        self.canvas = tk.Canvas(canvas_frame, 
                               xscrollcommand=self.canvas_h_scrollbar.set,
                               yscrollcommand=self.canvas_v_scrollbar.set,
                               bg="white")
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        self.canvas_h_scrollbar.config(command=self.canvas.xview)
        self.canvas_v_scrollbar.config(command=self.canvas.yview)
        
        # Create info panel below canvas
        info_frame = ttk.Frame(right_panel)
        info_frame.pack(side=tk.BOTTOM, fill=tk.X, pady=(10, 0))
        
        self.info_label = ttk.Label(info_frame, text="No submap selected")
        self.info_label.pack(side=tk.LEFT)
        
        # Add zoom controls
        zoom_frame = ttk.Frame(info_frame)
        zoom_frame.pack(side=tk.RIGHT)
        
        ttk.Label(zoom_frame, text="Zoom:").pack(side=tk.LEFT)
        
        self.zoom_scale = ttk.Scale(zoom_frame, from_=0.5, to=5.0, orient=tk.HORIZONTAL, length=200, value=1.0)
        self.zoom_scale.pack(side=tk.LEFT, padx=5)
        self.zoom_scale.bind("<ButtonRelease-1>", self._on_zoom_changed)
        
        self.zoom_label = ttk.Label(zoom_frame, text="100%")
        self.zoom_label.pack(side=tk.LEFT)
        
        logger.info("GUI layout created")
    
    def _check_updates(self):
        """Check for updates from the ROS node."""
        try:
            # Process all available updates
            while not self.update_queue.empty():
                update_type, data = self.update_queue.get_nowait()
                
                if update_type == "submap_list_updated":
                    # Update status label
                    self.status_label.config(text=f"Found {data} submaps")
                    logger.info(f"Updated status: Found {data} submaps")
                
                elif update_type == "new_submap":
                    # Add new submap to the list
                    self._add_submap(data)
            
        except queue.Empty:
            pass
        except Exception as e:
            logger.error(f"Error processing updates: {e}", exc_info=True)
        
        # Schedule next check
        self.root.after(100, self._check_updates)
    
    def _add_submap(self, submap_data):
        """Add a new submap to the list.
        
        Args:
            submap_data: Dictionary with submap information
        """
        trajectory_id = submap_data['trajectory_id']
        submap_index = submap_data['submap_index']
        submap_id = (trajectory_id, submap_index)
        
        # Store submap data
        self.submaps[submap_id] = submap_data
        
        # Add to listbox
        list_text = f"Trajectory {trajectory_id}, Submap {submap_index}"
        self.submap_listbox.insert(tk.END, list_text)
        
        # Update status
        status_text = f"Added submap {trajectory_id}_{submap_index}"
        self.status_label.config(text=status_text)
        logger.info(status_text)
        
        # Load the image
        try:
            file_path = submap_data['file_path']
            logger.info(f"Loading image from {file_path}")
            
            if not os.path.exists(file_path):
                logger.error(f"Image file does not exist: {file_path}")
                return
                
            image = Image.open(file_path)
            logger.info(f"Loaded image with size {image.size}")
            
            # Store the PIL image for later use with different zoom levels
            submap_data['pil_image'] = image
            
            # Create a PhotoImage for display
            photo = ImageTk.PhotoImage(image)
            self.submap_images[submap_id] = photo
            
            # If this is the first submap, select it
            if len(self.submaps) == 1:
                self.submap_listbox.selection_set(0)
                self._on_submap_selected(None)
                
        except Exception as e:
            error_msg = f"Error loading image for submap {trajectory_id}_{submap_index}: {e}"
            logger.error(error_msg, exc_info=True)
            print(error_msg)
    
    def _on_submap_selected(self, event):
        """Handle submap selection event."""
        # Get selected index
        selection = self.submap_listbox.curselection()
        if not selection:
            return
            
        index = selection[0]
        list_text = self.submap_listbox.get(index)
        
        # Parse trajectory and submap index from list text
        parts = list_text.split(", ")
        trajectory_id = int(parts[0].split(" ")[1])
        submap_index = int(parts[1].split(" ")[1])
        
        submap_id = (trajectory_id, submap_index)
        self.selected_submap = submap_id
        
        logger.info(f"Selected submap {trajectory_id}_{submap_index}")
        
        # Display the selected submap
        self._display_submap(submap_id)
    
    def _display_submap(self, submap_id):
        """Display the selected submap on the canvas.
        
        Args:
            submap_id: Tuple of (trajectory_id, submap_index)
        """
        if submap_id not in self.submaps:
            logger.warning(f"Submap {submap_id} not found in submaps dictionary")
            return
            
        submap_data = self.submaps[submap_id]
        
        # Clear canvas
        self.canvas.delete("all")
        
        # Get the photo image
        if submap_id not in self.submap_images:
            logger.warning(f"Submap {submap_id} not found in submap_images dictionary")
            return
            
        photo = self.submap_images[submap_id]
        
        # Display the image
        self.canvas.create_image(0, 0, anchor=tk.NW, image=photo)
        
        # Configure canvas scrolling
        self.canvas.config(scrollregion=self.canvas.bbox(tk.ALL))
        
        # Update info label
        trajectory_id, submap_index = submap_id
        resolution = submap_data['resolution']
        width = submap_data['width']
        height = submap_data['height']
        
        info_text = f"Trajectory {trajectory_id}, Submap {submap_index} | "
        info_text += f"Resolution: {resolution:.3f}m/pixel | "
        info_text += f"Size: {width}x{height} pixels | "
        info_text += f"Physical size: {width*resolution:.2f}m x {height*resolution:.2f}m"
        
        self.info_label.config(text=info_text)
        logger.info(f"Displayed submap {trajectory_id}_{submap_index}")
    
    def _on_zoom_changed(self, event):
        """Handle zoom level change."""
        if not self.selected_submap or self.selected_submap not in self.submaps:
            return
            
        # Get zoom level
        zoom = self.zoom_scale.get()
        self.zoom_label.config(text=f"{int(zoom*100)}%")
        
        # Get the original PIL image
        submap_data = self.submaps[self.selected_submap]
        if 'pil_image' not in submap_data:
            logger.warning(f"No PIL image found for submap {self.selected_submap}")
            return
            
        original_image = submap_data['pil_image']
        
        # Calculate new size
        new_width = int(original_image.width * zoom)
        new_height = int(original_image.height * zoom)
        
        # Resize the image
        resized_image = original_image.resize((new_width, new_height), Image.NEAREST)
        
        # Create new PhotoImage
        photo = ImageTk.PhotoImage(resized_image)
        self.submap_images[self.selected_submap] = photo
        
        # Redisplay
        self._display_submap(self.selected_submap)
        
        trajectory_id, submap_index = self.selected_submap
        logger.info(f"Applied zoom {int(zoom*100)}% to submap {trajectory_id}_{submap_index}")


class SubmapAnalyzerLauncher(BaseLauncher):
    """Launcher for the Submap Analyzer GUI.
    
    This launcher connects to a running Cartographer node, monitors submaps in real-time,
    and displays them in a GUI for analysis.
    """
    
    @classmethod
    def _register_params(cls):
        """Register parameters for this launcher."""
        cls.register_parameter(
            name="output_dir",
            param_type=str,
            required=False,
            help="Directory to save exported submaps",
            default=None
        )
        cls.register_parameter(
            name="timeout",
            param_type=float,
            required=False,
            help="Timeout in seconds for waiting for services and data",
            default=30.0
        )
    
    def __init__(self, **kwargs):
        """Initialize the submap analyzer launcher."""
        super().__init__(**kwargs)
        
        # Create output directory if not specified
        if self._output_dir is None:
            self.output_dir = tempfile.mkdtemp(prefix="cartographer_submaps_")
            print(f"No output directory specified. Using temporary directory: {self.output_dir}")
            logger.info(f"Using temporary directory: {self.output_dir}")
        else:
            self.output_dir = self._output_dir
            Path(self.output_dir).mkdir(parents=True, exist_ok=True)
            logger.info(f"Using output directory: {self.output_dir}")
        
        self.timeout = self._timeout
    
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
        logger.info("Starting Submap Analyzer")
        
        # Initialize ROS
        rclpy.init()
        logger.info("ROS initialized")
        
        # Create Tkinter root
        root = tk.Tk()
        
        # Create update queue for communication between ROS node and GUI
        update_queue = queue.Queue()
        
        # Create GUI
        gui = SubmapAnalyzerGUI(root, self.output_dir, update_queue)
        
        # Create asyncio event loop
        loop = asyncio.new_event_loop()
        
        try:
            # Create node with the event loop
            node = SubmapAnalyzerNode(self.output_dir, update_queue, loop)
            logger.info("Node created")
            
            # Create executor
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            logger.info("Executor created")
            
            # Start executor in a separate thread
            executor_thread = threading.Thread(target=executor.spin, daemon=True)
            executor_thread.start()
            logger.info("Executor thread started")
            
            # Set the event loop
            asyncio.set_event_loop(loop)
            
            # Start monitoring task in a separate thread
            monitor_thread = threading.Thread(
                target=lambda: loop.run_until_complete(node.monitor_submaps()),
                daemon=True
            )
            monitor_thread.start()
            logger.info("Monitor thread started")
            
            # Start the GUI main loop
            logger.info("Starting GUI main loop")
            root.mainloop()
            
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
            
            # Close the GUI if it's still open
            if 'root' in locals() and root.winfo_exists():
                root.destroy()
                logger.info("GUI destroyed")


# Create main function for command-line usage
main = SubmapAnalyzerLauncher.generate_main()

if __name__ == "__main__":
    main()
