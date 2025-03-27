"""Main window for the submap analyzer GUI.

This module contains the main window class for the submap analyzer GUI,
which integrates the ViewModel and UI components.
"""

import os
import tkinter as tk
from tkinter import ttk
import logging
import queue
from typing import Dict, List, Optional, Any, Tuple, Callable
import threading
import time

from .view_model import SubmapViewModel
from .callback_types import CallbackType
from .components import ControlPanel, SubmapListPanel, SubmapDisplayPanel
from .metrics import MetricsPanel

# Configure logging
logger = logging.getLogger('submap_analyzer.gui.main_window')


class SubmapAnalyzerGUI:
    """Main window for the submap analyzer GUI.
    
    This class integrates the ViewModel and UI components to create
    the complete GUI for the submap analyzer.
    """
    
    def __init__(self, output_dir: str, update_queue: queue.Queue):
        """Initialize the GUI.
        
        Args:
            output_dir: Directory where submaps are saved
            update_queue: Queue for communication with the ROS node
        """
        # Create the view model
        self.view_model = SubmapViewModel(output_dir, update_queue)
        
        # Create the main window
        self.root = tk.Tk()
        self.root.title("Submap Analyzer")
        self.root.geometry("1200x800")
        
        # Create the main layout
        self._create_layout()
        
        # Connect event handlers
        self._connect_event_handlers()
        
        # Set up periodic tasks
        self._setup_periodic_tasks()
        
        logger.info("GUI initialized")
    
    def _create_layout(self):
        """Create the GUI layout."""
        # Main frame
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Control panel at the top
        self.control_panel = ControlPanel(main_frame)
        self.control_panel.pack(fill=tk.X, pady=(0, 10))
        
        # Split the main area into left and right panels
        paned_window = ttk.PanedWindow(main_frame, orient=tk.HORIZONTAL)
        paned_window.pack(fill=tk.BOTH, expand=True)
        
        # Left panel for submap list and metrics
        left_frame = ttk.Frame(paned_window, width=300)
        paned_window.add(left_frame, weight=1)
        
        # Right panel for submap display
        right_frame = ttk.Frame(paned_window)
        paned_window.add(right_frame, weight=3)
        
        # Split the left panel into top and bottom
        left_paned = ttk.PanedWindow(left_frame, orient=tk.VERTICAL)
        left_paned.pack(fill=tk.BOTH, expand=True)
        
        # Top left for submap list
        submap_list_frame = ttk.Frame(left_paned)
        left_paned.add(submap_list_frame, weight=2)
        
        # Bottom left for metrics
        metrics_frame = ttk.Frame(left_paned)
        left_paned.add(metrics_frame, weight=1)
        
        # Submap list panel
        self.submap_list_panel = SubmapListPanel(submap_list_frame)
        self.submap_list_panel.pack(fill=tk.BOTH, expand=True)
        
        # Metrics panel
        self.metrics_panel = MetricsPanel(metrics_frame)
        self.metrics_panel.pack(fill=tk.BOTH, expand=True)
        
        # Submap display panel
        self.display_panel = SubmapDisplayPanel(right_frame)
        self.display_panel.pack(fill=tk.BOTH, expand=True)
    
    def _connect_event_handlers(self):
        """Connect event handlers between the view model and UI components."""
        # Control panel events
        self.control_panel.set_reload_command(self._on_reload_clicked)
        
        # Submap list events
        self.submap_list_panel.set_selection_command(self._on_submap_selected)
        
        # Display panel events
        self.display_panel.set_zoom_commands(
            self.display_panel.zoom_in,
            self.display_panel.zoom_out,
            self.display_panel.zoom_reset
        )
        
        # View model callbacks
        self.view_model.register_callback(CallbackType.ON_SUBMAP_ADDED, self._on_submap_added)
        self.view_model.register_callback(CallbackType.ON_SUBMAP_UPDATED, self._on_submap_updated)
        self.view_model.register_callback(CallbackType.ON_SUBMAP_SELECTED, self._on_submap_selected_from_model)
        self.view_model.register_callback(CallbackType.ON_STATUS_CHANGED, self._on_status_changed)
    
    def _setup_periodic_tasks(self):
        """Set up periodic tasks."""
        # Check for updates from the ROS node every 100ms
        def check_updates():
            self.view_model.process_updates()
            self.root.after(100, check_updates)
        
        # Start the periodic tasks
        self.root.after(100, check_updates)
    
    def run(self):
        """Run the GUI main loop."""
        logger.info("Starting GUI main loop")
        self.root.mainloop()
    
    # Event handlers
    
    def _on_reload_clicked(self):
        """Handle reload button click."""
        selected_submap_id = self.submap_list_panel.get_selected_submap_id()
        if selected_submap_id:
            logger.info(f"Reloading submap {selected_submap_id}")
            self.view_model.request_submap_reload(selected_submap_id)
        else:
            logger.warning("Cannot reload: no submap selected")
            # Update the status to inform the user
            self.control_panel.set_status("Cannot reload: no submap selected")
    
    def _on_submap_selected(self, event):
        """Handle submap selection in the listbox."""
        selected_submap_id = self.submap_list_panel.get_selected_submap_id()
        if selected_submap_id:
            self.view_model.select_submap(selected_submap_id)
    
    # View model callbacks
    
    def _on_submap_added(self, submap_id, submap_data):
        """Handle new submap added."""
        display_text = self.view_model.get_submap_display_text(submap_id)
        self.submap_list_panel.add_submap(submap_id, display_text)
    
    def _on_submap_updated(self, submap_id, submap_data):
        """Handle submap updated."""
        display_text = self.view_model.get_submap_display_text(submap_id)
        self.submap_list_panel.update_submap(submap_id, display_text)
        
        # If this is the currently selected submap, update the display
        if submap_id == self.view_model.selected_submap_id:
            self._update_display(submap_id, submap_data)
    
    def _on_submap_selected_from_model(self, submap_id, submap_data):
        """Handle submap selected from the view model."""
        # Update the listbox selection
        self.submap_list_panel.select_submap(submap_id)
        
        # Update the display
        self._update_display(submap_id, submap_data)
    
    def _update_display(self, submap_id, submap_data):
        """Update the display with the selected submap."""
        # Set the info text
        info_text = self.view_model.get_submap_info_text(submap_id)
        self.display_panel.set_info_text(info_text)
        
        # Update metrics
        self.metrics_panel.update_metrics(submap_data)
        
        # Display the image
        if 'pil_image' in submap_data:
            self.display_panel.display_image(submap_data['pil_image'])
    
    def _on_status_changed(self, status_text):
        """Handle status change."""
        self.control_panel.set_status(status_text) 