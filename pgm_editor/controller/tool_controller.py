from typing import Dict, Any, Optional, Callable

from pgm_editor.model.brush import Brush, create_brush


class ToolController:
    """Controller for handling drawing tools and their interactions."""
    
    def __init__(self, on_image_update: Callable):
        """
        Initialize the tool controller.
        
        Args:
            on_image_update: Callback to update the image display
        """
        self.on_image_update = on_image_update
        
        # Current tool state
        self.current_tool_type = "pencil"
        self.current_tool_properties = {
            'size': 5,
            'value': 0,
            'shape': 'circle',
            'hardness': 1.0
        }
        
        # Create the initial brush
        self.current_brush = create_brush(self.current_tool_type, **self.current_tool_properties)
        
        # Drawing state
        self.is_drawing = False
        self.last_x = 0
        self.last_y = 0
        
        # Shape drawing state
        self.shape_start_point = None
    
    def set_tool(self, tool_type: str, properties: Dict[str, Any]):
        """
        Set the current tool.
        
        Args:
            tool_type: Type of tool to use
            properties: Tool properties
        """
        self.current_tool_type = tool_type
        self.current_tool_properties = properties
        self.current_brush = create_brush(tool_type, **properties)
    
    def update_tool_properties(self, properties: Dict[str, Any]):
        """
        Update the current tool properties.
        
        Args:
            properties: New tool properties
        """
        self.current_tool_properties.update(properties)
        self.current_brush.update_properties(**properties)
    
    def start_drawing(self, x: int, y: int, image):
        """
        Start a drawing operation.
        
        Args:
            x, y: Starting coordinates
            image: The image to draw on
        """
        self.is_drawing = True
        self.last_x = x
        self.last_y = y
        
        if self.current_tool_type == 'fill':
            # Flood fill
            image.flood_fill(
                x, y, 
                self.current_tool_properties['value'],
                self.current_tool_properties.get('tolerance', 0)
            )
            self.on_image_update()
        elif self.current_tool_type in ['line', 'rectangle', 'circle']:
            # Start shape drawing
            self.shape_start_point = (x, y)
        else:
            # Pencil or eraser
            image.draw_point(
                x, y,
                self.current_tool_properties['value'],
                self.current_tool_properties['size'],
                self.current_tool_properties['shape'],
                self.current_tool_properties['hardness']
            )
            self.on_image_update()
    
    def continue_drawing(self, x: int, y: int, image):
        """
        Continue a drawing operation.
        
        Args:
            x, y: Current coordinates
            image: The image to draw on
        """
        if not self.is_drawing:
            return
        
        if self.current_tool_type in ['pencil', 'eraser']:
            # Draw a line from last position to current position
            image.draw_line(
                self.last_x, self.last_y, x, y,
                self.current_tool_properties['value'],
                self.current_tool_properties['size'],
                self.current_tool_properties['shape'],
                self.current_tool_properties['hardness']
            )
            self.on_image_update()
        
        self.last_x = x
        self.last_y = y
    
    def finish_drawing(self, x: int, y: int, image):
        """
        Finish a drawing operation.
        
        Args:
            x, y: Ending coordinates
            image: The image to draw on
        """
        if not self.is_drawing:
            return
        
        self.is_drawing = False
        
        if self.current_tool_type == 'line' and self.shape_start_point:
            # Draw line
            start_x, start_y = self.shape_start_point
            image.draw_line(
                start_x, start_y, x, y,
                self.current_tool_properties['value'],
                self.current_tool_properties['size'],
                self.current_tool_properties['shape'],
                self.current_tool_properties['hardness']
            )
            self.on_image_update()
        elif self.current_tool_type == 'rectangle' and self.shape_start_point:
            # Draw rectangle
            start_x, start_y = self.shape_start_point
            image.draw_rectangle(
                start_x, start_y, x, y,
                self.current_tool_properties['value'],
                self.current_tool_properties['size'],
                self.current_tool_properties['shape'],
                self.current_tool_properties['hardness'],
                self.current_tool_properties.get('fill', False)
            )
            self.on_image_update()
        elif self.current_tool_type == 'circle' and self.shape_start_point:
            # Draw circle
            start_x, start_y = self.shape_start_point
            # Calculate radius
            dx = x - start_x
            dy = y - start_y
            radius = int((dx*dx + dy*dy) ** 0.5)
            image.draw_circle(
                start_x, start_y, radius,
                self.current_tool_properties['value'],
                self.current_tool_properties['size'],
                self.current_tool_properties['shape'],
                self.current_tool_properties['hardness'],
                self.current_tool_properties.get('fill', False)
            )
            self.on_image_update()
        
        self.shape_start_point = None
