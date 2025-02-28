import os
import tkinter as tk
from typing import Optional, Dict, Any

from pgm_editor.model.pgm_image import PGMImage
from pgm_editor.model.brush import create_brush, Brush
from pgm_editor.view.main_window import MainWindow


class AppController:
    """Controller for the PGM editor application."""
    
    def __init__(self, root: tk.Tk):
        """
        Initialize the controller.
        
        Args:
            root: The root Tkinter window
        """
        self.root = root
        self.view = MainWindow(root)
        
        # Initialize model
        self.image: Optional[PGMImage] = None
        self.current_filepath: Optional[str] = None
        
        # Initialize brush
        self.current_brush_type = "pencil"
        self.current_brush_properties = {
            'size': 5,
            'value': 0,
            'shape': 'circle',
            'hardness': 1.0
        }
        self.brush = create_brush(self.current_brush_type, **self.current_brush_properties)
        
        # Set up callbacks
        self._setup_callbacks()
        
        # Create a default image
        self._create_new_image(128, 128)
    
    def _setup_callbacks(self):
        """Set up callbacks for the view."""
        callbacks = {
            'on_new': self._on_new,
            'on_open': self._on_open,
            'on_save': self._on_save,
            'on_save_as': self._on_save_as,
            'on_undo': self._on_undo,
            'on_redo': self._on_redo,
            'on_clear': self._on_clear,
            'on_tool_changed': self._on_tool_changed,
            'on_brush_property_changed': self._on_brush_property_changed,
            'on_canvas_click': self._on_canvas_click,
            'on_canvas_drag': self._on_canvas_drag,
            'on_canvas_release': self._on_canvas_release
        }
        
        self.view.set_callbacks(callbacks)
    
    def _create_new_image(self, width: int, height: int):
        """
        Create a new image.
        
        Args:
            width: Width of the image
            height: Height of the image
        """
        self.image = PGMImage(width, height)
        self.current_filepath = None
        self._update_view()
        self.view.set_status(f"Created new image ({width}x{height})")
    
    def _update_view(self):
        """Update the view with the current image data."""
        if self.image:
            self.view.update_image(self.image.data, self.image.width, self.image.height)
    
    def _on_new(self, width: int, height: int):
        """
        Handle the New action.
        
        Args:
            width: Width of the new image
            height: Height of the new image
        """
        self._create_new_image(width, height)
    
    def _on_open(self, filepath: str):
        """
        Handle the Open action.
        
        Args:
            filepath: Path to the file to open
        """
        try:
            self.image = PGMImage.from_file(filepath)
            self.current_filepath = filepath
            self._update_view()
            self.view.set_status(f"Opened {os.path.basename(filepath)}")
        except Exception as e:
            self.view.show_error("Error Opening File", str(e))
    
    def _on_save(self):
        """Handle the Save action."""
        if not self.image:
            return
        
        if self.current_filepath:
            self._save_to_file(self.current_filepath)
        else:
            # If no current filepath, use Save As
            self._on_save_as()
    
    def _on_save_as(self, filepath: Optional[str] = None, format_type: str = 'P5'):
        """
        Handle the Save As action.
        
        Args:
            filepath: Path where to save the file
            format_type: PGM format type ('P2' or 'P5')
        """
        if not self.image:
            return
        
        if filepath:
            self._save_to_file(filepath, format_type)
    
    def _save_to_file(self, filepath: str, format_type: str = 'P5'):
        """
        Save the image to a file.
        
        Args:
            filepath: Path where to save the file
            format_type: PGM format type ('P2' or 'P5')
        """
        try:
            self.image.save(filepath, format_type)
            self.current_filepath = filepath
            self.view.set_status(f"Saved to {os.path.basename(filepath)}")
        except Exception as e:
            self.view.show_error("Error Saving File", str(e))
    
    def _on_undo(self):
        """Handle the Undo action."""
        if self.image and self.image.undo():
            self._update_view()
            self.view.set_status("Undo")
    
    def _on_redo(self):
        """Handle the Redo action."""
        if self.image and self.image.redo():
            self._update_view()
            self.view.set_status("Redo")
    
    def _on_clear(self, value: int = 0):
        """
        Handle the Clear action.
        
        Args:
            value: Grayscale value to clear with
        """
        if self.image:
            self.image.clear(value)
            self._update_view()
            self.view.set_status(f"Cleared image with value {value}")
    
    def _on_tool_changed(self, tool_type: str, properties: Dict[str, Any]):
        """
        Handle tool selection change.
        
        Args:
            tool_type: New tool type
            properties: Tool properties
        """
        self.current_brush_type = tool_type
        self.current_brush_properties = properties
        self.brush = create_brush(tool_type, **properties)
        self.view.set_status(f"Selected {tool_type} tool")
    
    def _on_brush_property_changed(self, brush_type: str, properties: Dict[str, Any]):
        """
        Handle brush property change.
        
        Args:
            brush_type: Brush type
            properties: New brush properties
        """
        self.current_brush_type = brush_type
        self.current_brush_properties = properties
        self.brush.update_properties(**properties)
    
    def _on_canvas_click(self, x: int, y: int):
        """
        Handle canvas click event.
        
        Args:
            x, y: Click coordinates
        """
        if not self.image:
            return
        
        if self.current_brush_type == 'fill':
            # Flood fill
            self.image.flood_fill(
                x, y, 
                self.current_brush_properties['value'],
                self.current_brush_properties.get('tolerance', 0)
            )
            self._update_view()
        elif self.current_brush_type in ['line', 'rectangle', 'circle']:
            # Start shape drawing
            if hasattr(self.brush, 'start_shape'):
                self.brush.start_shape(x, y)
        else:
            # Pencil or eraser
            self.image.draw_point(
                x, y,
                self.current_brush_properties['value'],
                self.current_brush_properties['size'],
                self.current_brush_properties['shape'],
                self.current_brush_properties['hardness']
            )
            self._update_view()
    
    def _on_canvas_drag(self, x: int, y: int):
        """
        Handle canvas drag event.
        
        Args:
            x, y: Drag coordinates
        """
        if not self.image:
            return
        
        if self.current_brush_type in ['line', 'rectangle', 'circle']:
            # Update shape preview
            if hasattr(self.brush, 'update_shape'):
                self.brush.update_shape(x, y)
                # We could implement a preview here
        elif self.current_brush_type in ['pencil', 'eraser']:
            # Continue drawing
            self.image.draw_point(
                x, y,
                self.current_brush_properties['value'],
                self.current_brush_properties['size'],
                self.current_brush_properties['shape'],
                self.current_brush_properties['hardness']
            )
            self._update_view()
    
    def _on_canvas_release(self, x: int, y: int):
        """
        Handle canvas release event.
        
        Args:
            x, y: Release coordinates
        """
        if not self.image:
            return
        
        if self.current_brush_type == 'line':
            # Finish line
            if hasattr(self.brush, 'finish_shape'):
                start, end, props = self.brush.finish_shape()
                self.image.draw_line(
                    start[0], start[1], end[0], end[1],
                    props['value'], props['size'], props['shape'], props['hardness']
                )
                self._update_view()
        elif self.current_brush_type == 'rectangle':
            # Finish rectangle
            if hasattr(self.brush, 'finish_shape'):
                start, end, props = self.brush.finish_shape()
                self.image.draw_rectangle(
                    start[0], start[1], end[0], end[1],
                    props['value'], props['size'], props['shape'], props['hardness'],
                    props.get('fill', False)
                )
                self._update_view()
        elif self.current_brush_type == 'circle':
            # Finish circle
            if hasattr(self.brush, 'finish_shape'):
                start, end, props = self.brush.finish_shape()
                # Calculate radius
                dx = end[0] - start[0]
                dy = end[1] - start[1]
                radius = int((dx*dx + dy*dy) ** 0.5)
                self.image.draw_circle(
                    start[0], start[1], radius,
                    props['value'], props['size'], props['shape'], props['hardness'],
                    props.get('fill', False)
                )
                self._update_view()
    
    def run(self):
        """Run the application."""
        self.root.mainloop()
