"""UI components for the submap analyzer GUI.

This module contains reusable UI components for the submap analyzer GUI.
"""

import tkinter as tk
from tkinter import ttk
import logging
from typing import Dict, List, Optional, Any, Tuple, Callable
from PIL import Image, ImageTk

# Configure logging
logger = logging.getLogger('submap_analyzer.gui.components')


class ControlPanel(ttk.Frame):
    """Control panel with buttons and checkboxes."""
    
    def __init__(self, parent, **kwargs):
        """Initialize the control panel.
        
        Args:
            parent: Parent widget
            **kwargs: Additional arguments to pass to ttk.Frame
        """
        super().__init__(parent, **kwargs)
        
        # Create controls
        self.reload_button = ttk.Button(self, text="Reload Map")
        self.reload_button.pack(side=tk.LEFT, padx=5)
        
        # Status label
        self.status_label = ttk.Label(self, text="No submaps loaded")
        self.status_label.pack(side=tk.LEFT, padx=10)
    
    def set_reload_command(self, command: Callable) -> None:
        """Set the command for the reload button.
        
        Args:
            command: Function to call when the button is clicked
        """
        self.reload_button.config(command=command)
    
    def set_status(self, text: str) -> None:
        """Set the status label text.
        
        Args:
            text: Text to display in the status label
        """
        self.status_label.config(text=text)


class SubmapListPanel(ttk.Frame):
    """Panel for displaying the list of submaps."""
    
    def __init__(self, parent, **kwargs):
        """Initialize the submap list panel.
        
        Args:
            parent: Parent widget
            **kwargs: Additional arguments to pass to ttk.Frame
        """
        super().__init__(parent, **kwargs)
        
        # Create a label
        label = ttk.Label(self, text="Submaps:")
        label.pack(side=tk.TOP, anchor=tk.W, padx=5, pady=5)
        
        # Create a frame for the listbox and scrollbar
        list_frame = ttk.Frame(self)
        list_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Create a scrollbar
        scrollbar = ttk.Scrollbar(list_frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Create a listbox
        self.listbox = tk.Listbox(list_frame, yscrollcommand=scrollbar.set)
        self.listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        # Configure the scrollbar
        scrollbar.config(command=self.listbox.yview)
        
        # Store submap IDs
        self.submap_ids = []
    
    def set_selection_command(self, command: Callable) -> None:
        """Set the command for when a submap is selected.
        
        Args:
            command: Function to call when a submap is selected
        """
        self.listbox.bind('<<ListboxSelect>>', command)
    
    def clear(self) -> None:
        """Clear the list of submaps."""
        self.listbox.delete(0, tk.END)
        self.submap_ids = []
    
    def add_submap(self, submap_id: Tuple[int, int], display_text: str) -> None:
        """Add a submap to the list.
        
        Args:
            submap_id: Tuple of (trajectory_id, submap_index)
            display_text: Text to display in the listbox
        """
        self.listbox.insert(tk.END, display_text)
        self.submap_ids.append(submap_id)
        logger.info(f"Added submap {submap_id} to list")
    
    def update_submap(self, submap_id: Tuple[int, int], display_text: str) -> None:
        """Update a submap in the list.
        
        Args:
            submap_id: Tuple of (trajectory_id, submap_index)
            display_text: Text to display in the listbox
        """
        if submap_id in self.submap_ids:
            index = self.submap_ids.index(submap_id)
            self.listbox.delete(index)
            self.listbox.insert(index, display_text)
            logger.info(f"Updated submap {submap_id} in list")
    
    def get_selected_submap_id(self) -> Optional[Tuple[int, int]]:
        """Get the ID of the selected submap.
        
        Returns:
            Tuple of (trajectory_id, submap_index) or None if no submap is selected
        """
        selected = self.listbox.curselection()
        if not selected:
            return None
            
        index = selected[0]
        if index < len(self.submap_ids):
            return self.submap_ids[index]
            
        return None
    
    def select_submap(self, submap_id: Tuple[int, int]) -> None:
        """Select a submap in the list.
        
        Args:
            submap_id: Tuple of (trajectory_id, submap_index)
        """
        if submap_id in self.submap_ids:
            index = self.submap_ids.index(submap_id)
            self.listbox.selection_clear(0, tk.END)
            self.listbox.selection_set(index)
            self.listbox.see(index)
            logger.info(f"Selected submap {submap_id} in list")


class SubmapDisplayPanel(ttk.Frame):
    """Panel for displaying a submap."""
    
    def __init__(self, parent, **kwargs):
        """Initialize the submap display panel.
        
        Args:
            parent: Parent widget
            **kwargs: Additional arguments to pass to ttk.Frame
        """
        super().__init__(parent, **kwargs)
        
        # Create a frame for the canvas and scrollbars
        canvas_frame = ttk.Frame(self)
        canvas_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        
        # Create horizontal scrollbar
        h_scrollbar = ttk.Scrollbar(canvas_frame, orient=tk.HORIZONTAL)
        h_scrollbar.pack(side=tk.BOTTOM, fill=tk.X)
        
        # Create vertical scrollbar
        v_scrollbar = ttk.Scrollbar(canvas_frame)
        v_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Create canvas
        self.canvas = tk.Canvas(
            canvas_frame,
            xscrollcommand=h_scrollbar.set,
            yscrollcommand=v_scrollbar.set,
            bg='white'
        )
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        # Configure scrollbars
        h_scrollbar.config(command=self.canvas.xview)
        v_scrollbar.config(command=self.canvas.yview)
        
        # Create info panel
        info_frame = ttk.Frame(self)
        info_frame.pack(side=tk.BOTTOM, fill=tk.X, pady=5)
        
        # Create info label
        self.info_label = ttk.Label(info_frame, text="No submap selected")
        self.info_label.pack(side=tk.LEFT, padx=5)
        
        # Create zoom controls
        zoom_frame = ttk.Frame(info_frame)
        zoom_frame.pack(side=tk.RIGHT, padx=5)
        
        zoom_in_button = ttk.Button(zoom_frame, text="+", width=2)
        zoom_in_button.pack(side=tk.LEFT, padx=2)
        
        zoom_out_button = ttk.Button(zoom_frame, text="-", width=2)
        zoom_out_button.pack(side=tk.LEFT, padx=2)
        
        zoom_reset_button = ttk.Button(zoom_frame, text="1:1", width=3)
        zoom_reset_button.pack(side=tk.LEFT, padx=2)
        
        # Store zoom buttons for later configuration
        self.zoom_in_button = zoom_in_button
        self.zoom_out_button = zoom_out_button
        self.zoom_reset_button = zoom_reset_button
        
        # Store image and zoom level
        self.image = None
        self.tk_image = None
        self.image_id = None
        self.zoom_level = 1.0
    
    def set_zoom_commands(self, zoom_in: Callable, zoom_out: Callable, zoom_reset: Callable) -> None:
        """Set the commands for the zoom buttons.
        
        Args:
            zoom_in: Function to call when the zoom in button is clicked
            zoom_out: Function to call when the zoom out button is clicked
            zoom_reset: Function to call when the zoom reset button is clicked
        """
        self.zoom_in_button.config(command=zoom_in)
        self.zoom_out_button.config(command=zoom_out)
        self.zoom_reset_button.config(command=zoom_reset)
    
    def set_info_text(self, text: str) -> None:
        """Set the info label text.
        
        Args:
            text: Text to display in the info label
        """
        self.info_label.config(text=text)
    
    def display_image(self, image: Image.Image) -> None:
        """Display an image in the canvas.
        
        Args:
            image: PIL Image to display
        """
        self.image = image
        self._update_display()
    
    def zoom_in(self) -> None:
        """Zoom in on the image."""
        self.zoom_level *= 1.2
        self._update_display()
    
    def zoom_out(self) -> None:
        """Zoom out on the image."""
        self.zoom_level /= 1.2
        self._update_display()
    
    def zoom_reset(self) -> None:
        """Reset the zoom level to 1:1."""
        self.zoom_level = 1.0
        self._update_display()
    
    def _update_display(self) -> None:
        """Update the display with the current image and zoom level."""
        if self.image is None:
            return
            
        # Calculate new dimensions
        width = int(self.image.width * self.zoom_level)
        height = int(self.image.height * self.zoom_level)
        
        # Resize image
        if self.zoom_level == 1.0:
            resized_image = self.image
        else:
            resized_image = self.image.resize((width, height), Image.LANCZOS)
        
        # Convert to Tkinter image
        self.tk_image = ImageTk.PhotoImage(resized_image)
        
        # Update canvas
        if self.image_id:
            self.canvas.delete(self.image_id)
            
        self.image_id = self.canvas.create_image(0, 0, anchor=tk.NW, image=self.tk_image)
        
        # Update scrollregion
        self.canvas.config(scrollregion=(0, 0, width, height)) 