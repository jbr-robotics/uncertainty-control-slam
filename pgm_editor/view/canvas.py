import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import numpy as np
from typing import Callable, Optional, Tuple


class ImageCanvas(ttk.Frame):
    """Canvas widget for displaying and interacting with the PGM image."""
    
    def __init__(self, parent):
        """
        Initialize the image canvas.
        
        Args:
            parent: Parent widget
        """
        super().__init__(parent)
        
        # Create canvas with scrollbars
        self.canvas = tk.Canvas(self, bg="gray80", highlightthickness=0)
        self.h_scrollbar = ttk.Scrollbar(self, orient=tk.HORIZONTAL, command=self.canvas.xview)
        self.v_scrollbar = ttk.Scrollbar(self, orient=tk.VERTICAL, command=self.canvas.yview)
        
        self.canvas.config(xscrollcommand=self.h_scrollbar.set, yscrollcommand=self.v_scrollbar.set)
        
        # Grid layout
        self.canvas.grid(row=0, column=0, sticky="nsew")
        self.h_scrollbar.grid(row=1, column=0, sticky="ew")
        self.v_scrollbar.grid(row=0, column=1, sticky="ns")
        
        # Configure grid weights
        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(0, weight=1)
        
        # Image display variables
        self.image_data = None
        self.pil_image = None
        self.tk_image = None
        self.image_id = None
        self.zoom_factor = 1.0
        self.show_grid = False
        self.show_values = False
        self.grid_lines = []
        self.value_texts = []
        
        # Mouse tracking
        self.last_x = 0
        self.last_y = 0
        self.dragging = False
        
        # Callbacks
        self.click_callback = None
        self.drag_callback = None
        self.release_callback = None
        self.position_callback = None
        
        # Bind events
        self.canvas.bind("<Button-1>", self._on_click)
        self.canvas.bind("<B1-Motion>", self._on_drag)
        self.canvas.bind("<ButtonRelease-1>", self._on_release)
        self.canvas.bind("<Motion>", self._on_motion)
        self.canvas.bind("<MouseWheel>", self._on_mousewheel)  # Windows/macOS
        self.canvas.bind("<Button-4>", self._on_mousewheel)  # Linux scroll up
        self.canvas.bind("<Button-5>", self._on_mousewheel)  # Linux scroll down
        
        # Pan with middle mouse button
        self.canvas.bind("<Button-2>", self._start_pan)
        self.canvas.bind("<B2-Motion>", self._pan)
        self.canvas.bind("<ButtonRelease-2>", self._end_pan)
        
        # Pan with right mouse button
        self.canvas.bind("<Button-3>", self._start_pan)
        self.canvas.bind("<B3-Motion>", self._pan)
        self.canvas.bind("<ButtonRelease-3>", self._end_pan)
    
    def update_image(self, image_data: np.ndarray):
        """
        Update the displayed image.
        
        Args:
            image_data: NumPy array with image data
        """
        # Store reference to old images for cleanup
        old_pil_image = self.pil_image
        old_tk_image = self.tk_image
        
        self.image_data = image_data
        
        # Convert to PIL Image
        self.pil_image = Image.fromarray(image_data, mode='L')
        
        # Apply zoom
        if self.zoom_factor != 1.0:
            # Limit image size to prevent memory issues
            max_dimension = 2000  # Maximum dimension in pixels
            new_width = int(self.pil_image.width * self.zoom_factor)
            new_height = int(self.pil_image.height * self.zoom_factor)
            
            if new_width > max_dimension or new_height > max_dimension:
                # Adjust zoom factor to fit within max dimension
                scale = min(max_dimension / new_width, max_dimension / new_height)
                new_width = int(new_width * scale)
                new_height = int(new_height * scale)
                
            new_size = (new_width, new_height)
            self.pil_image = self.pil_image.resize(new_size, Image.NEAREST)
        
        # Convert to Tkinter PhotoImage
        self.tk_image = ImageTk.PhotoImage(self.pil_image)
        
        # Update canvas
        if self.image_id is None:
            self.image_id = self.canvas.create_image(0, 0, anchor=tk.NW, image=self.tk_image)
        else:
            self.canvas.itemconfig(self.image_id, image=self.tk_image)
        
        # Update canvas scrollregion
        self.canvas.config(scrollregion=self.canvas.bbox(tk.ALL))
        
        # Update grid if needed
        if self.show_grid:
            self._update_grid()
        
        # Update value display if needed
        if self.show_values:
            self._update_value_display()
            
        # Help garbage collection by explicitly deleting old images
        # This can help prevent memory leaks in Tkinter
        del old_pil_image
        del old_tk_image
    
    def _update_grid(self):
        """Update the grid overlay on the image."""
        # Clear existing grid lines
        for line_id in self.grid_lines:
            self.canvas.delete(line_id)
        self.grid_lines = []
        
        if not self.pil_image:
            return
        
        # Get image dimensions
        width, height = self.pil_image.size
        
        # Draw vertical lines
        for x in range(0, width + 1, max(1, int(10 * self.zoom_factor))):
            line_id = self.canvas.create_line(
                x, 0, x, height, 
                fill="gray50", width=1, dash=(2, 2)
            )
            self.grid_lines.append(line_id)
        
        # Draw horizontal lines
        for y in range(0, height + 1, max(1, int(10 * self.zoom_factor))):
            line_id = self.canvas.create_line(
                0, y, width, y, 
                fill="gray50", width=1, dash=(2, 2)
            )
            self.grid_lines.append(line_id)
    
    def _update_value_display(self):
        """Update the pixel value display on the image."""
        # Clear existing value texts
        for text_id in self.value_texts:
            self.canvas.delete(text_id)
        self.value_texts = []
        
        if not self.pil_image or self.image_data is None:
            return
        
        # Get image dimensions
        width, height = self.pil_image.size
        orig_width, orig_height = self.image_data.shape[1], self.image_data.shape[0]
        
        # Only show values if zoom is large enough
        if self.zoom_factor < 4.0:
            return
        
        # Calculate step size based on zoom
        step = max(1, int(20 / self.zoom_factor))
        
        # Draw values
        for y in range(0, orig_height, step):
            for x in range(0, orig_width, step):
                # Get pixel value
                value = self.image_data[y, x]
                
                # Calculate position in zoomed image
                canvas_x = int(x * self.zoom_factor) + int(self.zoom_factor / 2)
                canvas_y = int(y * self.zoom_factor) + int(self.zoom_factor / 2)
                
                # Choose text color based on pixel value
                text_color = "black" if value > 128 else "white"
                
                # Create text
                text_id = self.canvas.create_text(
                    canvas_x, canvas_y,
                    text=str(value),
                    fill=text_color,
                    font=("Arial", max(6, int(self.zoom_factor * 0.8)))
                )
                self.value_texts.append(text_id)
    
    def set_show_grid(self, show: bool):
        """
        Set whether to show the grid overlay.
        
        Args:
            show: Whether to show the grid
        """
        self.show_grid = show
        
        if show:
            self._update_grid()
        else:
            # Clear grid lines
            for line_id in self.grid_lines:
                self.canvas.delete(line_id)
            self.grid_lines = []
    
    def set_show_values(self, show: bool):
        """
        Set whether to show pixel values.
        
        Args:
            show: Whether to show pixel values
        """
        self.show_values = show
        
        if show:
            self._update_value_display()
        else:
            # Clear value texts
            for text_id in self.value_texts:
                self.canvas.delete(text_id)
            self.value_texts = []
    
    def zoom_in(self):
        """Zoom in on the image."""
        # Limit maximum zoom to prevent memory issues
        if self.zoom_factor >= 10.0:
            return
            
        self.zoom_factor *= 1.25
        if self.image_data is not None:
            self.update_image(self.image_data)
    
    def zoom_out(self):
        """Zoom out of the image."""
        self.zoom_factor /= 1.25
        if self.zoom_factor < 0.1:
            self.zoom_factor = 0.1
        if self.image_data is not None:
            self.update_image(self.image_data)
    
    def reset_zoom(self):
        """Reset zoom to original size."""
        self.zoom_factor = 1.0
        if self.image_data is not None:
            self.update_image(self.image_data)
    
    def _on_mousewheel(self, event):
        """
        Handle mouse wheel events for zooming.
        
        Args:
            event: Mouse wheel event
        """
        # Determine zoom direction
        if event.num == 4 or (hasattr(event, 'delta') and event.delta > 0):
            # Zoom in
            self.zoom_in()
        elif event.num == 5 or (hasattr(event, 'delta') and event.delta < 0):
            # Zoom out
            self.zoom_out()
    
    def _start_pan(self, event):
        """
        Start panning the image.
        
        Args:
            event: Mouse button press event
        """
        self.canvas.scan_mark(event.x, event.y)
    
    def _pan(self, event):
        """
        Pan the image.
        
        Args:
            event: Mouse motion event
        """
        self.canvas.scan_dragto(event.x, event.y, gain=1)
    
    def _end_pan(self, event):
        """
        End panning the image.
        
        Args:
            event: Mouse button release event
        """
        pass  # Nothing to do here
    
    def _on_click(self, event):
        """
        Handle mouse click events.
        
        Args:
            event: Mouse click event
        """
        self.last_x = event.x
        self.last_y = event.y
        self.dragging = True
        
        # Convert canvas coordinates to image coordinates
        x, y = self._canvas_to_image_coords(event.x, event.y)
        
        if self.click_callback and self.image_data is not None:
            if 0 <= x < self.image_data.shape[1] and 0 <= y < self.image_data.shape[0]:
                self.click_callback(x, y)
    
    def _on_drag(self, event):
        """
        Handle mouse drag events.
        
        Args:
            event: Mouse motion event
        """
        if not self.dragging:
            return
        
        # Convert canvas coordinates to image coordinates
        x, y = self._canvas_to_image_coords(event.x, event.y)
        
        if self.drag_callback and self.image_data is not None:
            if 0 <= x < self.image_data.shape[1] and 0 <= y < self.image_data.shape[0]:
                self.drag_callback(x, y)
        
        self.last_x = event.x
        self.last_y = event.y
    
    def _on_release(self, event):
        """
        Handle mouse release events.
        
        Args:
            event: Mouse release event
        """
        if not self.dragging:
            return
        
        self.dragging = False
        
        # Convert canvas coordinates to image coordinates
        x, y = self._canvas_to_image_coords(event.x, event.y)
        
        if self.release_callback and self.image_data is not None:
            if 0 <= x < self.image_data.shape[1] and 0 <= y < self.image_data.shape[0]:
                self.release_callback(x, y)
    
    def _on_motion(self, event):
        """
        Handle mouse motion events.
        
        Args:
            event: Mouse motion event
        """
        # Convert canvas coordinates to image coordinates
        x, y = self._canvas_to_image_coords(event.x, event.y)
        
        if self.position_callback and self.image_data is not None:
            if 0 <= x < self.image_data.shape[1] and 0 <= y < self.image_data.shape[0]:
                value = self.image_data[y, x]
                self.position_callback(x, y, value)
            else:
                self.position_callback(x, y)
    
    def _canvas_to_image_coords(self, canvas_x: int, canvas_y: int) -> Tuple[int, int]:
        """
        Convert canvas coordinates to image coordinates.
        
        Args:
            canvas_x: X coordinate in canvas
            canvas_y: Y coordinate in canvas
            
        Returns:
            Tuple[int, int]: X, Y coordinates in image
        """
        # Get canvas scroll position
        x_scroll = self.canvas.canvasx(canvas_x)
        y_scroll = self.canvas.canvasy(canvas_y)
        
        # Convert to image coordinates
        image_x = int(x_scroll / self.zoom_factor)
        image_y = int(y_scroll / self.zoom_factor)
        
        return image_x, image_y
    
    def set_click_callback(self, callback: Callable[[int, int], None]):
        """
        Set the callback for mouse click events.
        
        Args:
            callback: Function to call on mouse click
        """
        self.click_callback = callback
    
    def set_drag_callback(self, callback: Callable[[int, int], None]):
        """
        Set the callback for mouse drag events.
        
        Args:
            callback: Function to call on mouse drag
        """
        self.drag_callback = callback
    
    def set_release_callback(self, callback: Callable[[int, int], None]):
        """
        Set the callback for mouse release events.
        
        Args:
            callback: Function to call on mouse release
        """
        self.release_callback = callback
    
    def set_mouse_position_callback(self, callback: Callable[[int, int, Optional[int]], None]):
        """
        Set the callback for mouse position updates.
        
        Args:
            callback: Function to call on mouse movement
        """
        self.position_callback = callback
