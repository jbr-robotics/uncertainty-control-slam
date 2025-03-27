"""Metrics panel for the submap analyzer GUI.

This module contains the metrics panel component for displaying metrics about submaps.
"""

import tkinter as tk
from tkinter import ttk
import logging
from typing import Dict, List, Optional, Any, Tuple, Callable

# Configure logging
logger = logging.getLogger('submap_analyzer.gui.metrics')


class MetricsPanel(ttk.Frame):
    """Panel for displaying metrics about submaps."""
    
    def __init__(self, parent, **kwargs):
        """Initialize the metrics panel.
        
        Args:
            parent: Parent widget
            **kwargs: Additional arguments to pass to ttk.Frame
        """
        super().__init__(parent, **kwargs)
        
        # Create a label
        label = ttk.Label(self, text="Metrics:")
        label.pack(side=tk.TOP, anchor=tk.W, padx=5, pady=5)
        
        # Create a frame for metrics
        self.metrics_frame = ttk.Frame(self)
        self.metrics_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Create metrics labels
        self.metrics = {}
        self._create_metric("resolution", "Resolution")
        self._create_metric("size", "Size (pixels)")
        self._create_metric("physical_size", "Physical Size (m)")
        self._create_metric("trajectory", "Trajectory")
        self._create_metric("submap_index", "Submap Index")
        
        # Add a separator
        separator = ttk.Separator(self, orient=tk.HORIZONTAL)
        separator.pack(side=tk.TOP, fill=tk.X, padx=5, pady=10)
        
        # Create a frame for custom metrics
        self.custom_metrics_frame = ttk.Frame(self)
        self.custom_metrics_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Custom metrics will be added dynamically
        self.custom_metrics = {}
    
    def _create_metric(self, name: str, label_text: str):
        """Create a metric display with label and value.
        
        Args:
            name: Name of the metric (used as key)
            label_text: Text to display as the label
        """
        frame = ttk.Frame(self.metrics_frame)
        frame.pack(side=tk.TOP, fill=tk.X, pady=2)
        
        label = ttk.Label(frame, text=f"{label_text}:", width=15, anchor=tk.W)
        label.pack(side=tk.LEFT)
        
        value = ttk.Label(frame, text="-")
        value.pack(side=tk.LEFT, fill=tk.X, expand=True)
        
        self.metrics[name] = value
    
    def add_custom_metric(self, name: str, label_text: str):
        """Add a custom metric display.
        
        Args:
            name: Name of the metric (used as key)
            label_text: Text to display as the label
        """
        if name in self.custom_metrics:
            return
            
        frame = ttk.Frame(self.custom_metrics_frame)
        frame.pack(side=tk.TOP, fill=tk.X, pady=2)
        
        label = ttk.Label(frame, text=f"{label_text}:", width=15, anchor=tk.W)
        label.pack(side=tk.LEFT)
        
        value = ttk.Label(frame, text="-")
        value.pack(side=tk.LEFT, fill=tk.X, expand=True)
        
        self.custom_metrics[name] = value
        logger.info(f"Added custom metric: {name}")
    
    def update_metrics(self, submap_data: Dict[str, Any]):
        """Update the metrics display with data from a submap.
        
        Args:
            submap_data: Dictionary with submap information
        """
        if not submap_data:
            self.clear_metrics()
            return
            
        # Update standard metrics
        trajectory_id = submap_data.get('trajectory_id', '-')
        submap_index = submap_data.get('submap_index', '-')
        resolution = submap_data.get('resolution', 0)
        width = submap_data.get('width', 0)
        height = submap_data.get('height', 0)
        
        self.metrics['resolution'].config(text=f"{resolution:.3f} m/pixel")
        self.metrics['size'].config(text=f"{width} x {height}")
        self.metrics['physical_size'].config(
            text=f"{width*resolution:.2f} x {height*resolution:.2f} m"
        )
        self.metrics['trajectory'].config(text=str(trajectory_id))
        self.metrics['submap_index'].config(text=str(submap_index))
        
        # Update custom metrics if they exist in the data
        for name, label in self.custom_metrics.items():
            if name in submap_data:
                value = submap_data[name]
                if isinstance(value, float):
                    label.config(text=f"{value:.3f}")
                else:
                    label.config(text=str(value))
            else:
                label.config(text="-")
        
        logger.info(f"Updated metrics for submap {trajectory_id}_{submap_index}")
    
    def clear_metrics(self):
        """Clear all metrics displays."""
        for label in self.metrics.values():
            label.config(text="-")
            
        for label in self.custom_metrics.values():
            label.config(text="-")
        
        logger.info("Cleared metrics display") 