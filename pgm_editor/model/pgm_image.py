import numpy as np
import os
from typing import Tuple, Optional, Union, List


class PGMImage:
    """
    Class representing a PGM (Portable Gray Map) image.
    Supports both P2 (ASCII) and P5 (binary) formats.
    """
    
    def __init__(self, width: int = 128, height: int = 128, max_val: int = 255, data: Optional[np.ndarray] = None):
        """
        Initialize a PGM image.
        
        Args:
            width: Width of the image in pixels
            height: Height of the image in pixels
            max_val: Maximum grayscale value (typically 255)
            data: Optional numpy array with image data
        """
        self.width = width
        self.height = height
        self.max_val = max_val
        
        if data is not None:
            if data.shape != (height, width):
                raise ValueError(f"Data shape {data.shape} does not match specified dimensions ({height}, {width})")
            
            # Check if data contains values greater than max_val
            if np.any(data > max_val):
                # If using uint8, NumPy will have already clipped values to 255
                if data.dtype == np.uint8 and max_val == 255:
                    self.data = data.copy()
                else:
                    # Clip values to max_val
                    self.data = np.clip(data, 0, max_val).astype(np.uint8)
            else:
                self.data = data.astype(np.uint8)
        else:
            # Initialize with zeros (black image)
            self.data = np.zeros((height, width), dtype=np.uint8)
        
        self._history = []  # For undo functionality
        self._redo_stack = []  # For redo functionality
        self._save_state()  # Save initial state
        
    def _save_state(self):
        """Save current state for undo functionality."""
        self._history.append(self.data.copy())
        self._redo_stack = []  # Clear redo stack when a new action is performed
        
        # Limit history size to prevent memory issues
        if len(self._history) > 20:
            self._history.pop(0)
    
    def undo(self) -> bool:
        """
        Undo the last action.
        
        Returns:
            bool: True if undo was successful, False otherwise
        """
        if len(self._history) <= 1:
            return False
        
        self._redo_stack.append(self._history.pop())  # Move current state to redo stack
        self.data = self._history[-1].copy()
        return True
    
    def redo(self) -> bool:
        """
        Redo the last undone action.
        
        Returns:
            bool: True if redo was successful, False otherwise
        """
        if not self._redo_stack:
            return False
        
        self._history.append(self._redo_stack.pop())
        self.data = self._history[-1].copy()
        return True
    
    @classmethod
    def from_file(cls, filepath: str) -> 'PGMImage':
        """
        Load a PGM image from a file.
        
        Args:
            filepath: Path to the PGM file
            
        Returns:
            PGMImage: A new PGMImage instance
            
        Raises:
            ValueError: If the file is not a valid PGM file
            FileNotFoundError: If the file does not exist
        """
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"File {filepath} not found")
        
        with open(filepath, 'rb') as f:
            # Read magic number
            magic = f.readline().decode('ascii').strip()
            if magic not in ['P2', 'P5']:
                raise ValueError(f"Not a valid PGM file: {magic}")
            
            # Skip comments
            line = f.readline()
            while line.startswith(b'#'):
                line = f.readline()
            
            # Read dimensions
            width, height = map(int, line.decode('ascii').strip().split())
            
            # Read max value
            max_val = int(f.readline().decode('ascii').strip())
            
            # Read data
            if magic == 'P2':  # ASCII format
                data = []
                for line in f:
                    data.extend(map(int, line.decode('ascii').strip().split()))
                data = np.array(data, dtype=np.uint8).reshape((height, width))
            else:  # P5, binary format
                data = np.frombuffer(f.read(), dtype=np.uint8).reshape((height, width))
            
            return cls(width, height, max_val, data)
    
    def save(self, filepath: str, format_type: str = 'P5'):
        """
        Save the PGM image to a file.
        
        Args:
            filepath: Path where to save the file
            format_type: 'P2' for ASCII format, 'P5' for binary format
            
        Raises:
            ValueError: If format_type is not 'P2' or 'P5'
        """
        if format_type not in ['P2', 'P5']:
            raise ValueError("Format type must be 'P2' or 'P5'")
        
        with open(filepath, 'wb') as f:
            # Write header
            f.write(f"{format_type}\n".encode('ascii'))
            f.write(f"# Created with PGM Editor\n".encode('ascii'))
            f.write(f"{self.width} {self.height}\n".encode('ascii'))
            f.write(f"{self.max_val}\n".encode('ascii'))
            
            # Write data
            if format_type == 'P2':  # ASCII format
                for row in self.data:
                    f.write(' '.join(map(str, row)).encode('ascii') + b'\n')
            else:  # P5, binary format
                f.write(self.data.tobytes())
    
    def draw_point(self, x: int, y: int, value: int, brush_size: int = 1, 
                  brush_shape: str = 'circle', hardness: float = 1.0):
        """
        Draw a point on the image.
        
        Args:
            x: X coordinate
            y: Y coordinate
            value: Grayscale value to draw with
            brush_size: Size of the brush
            brush_shape: Shape of the brush ('circle', 'square')
            hardness: Hardness of the brush (0.0 to 1.0)
        """
        # Create a copy of the current data for modification
        new_data = self.data.copy()
        
        # Calculate brush area
        radius = brush_size // 2
        min_x = max(0, x - radius)
        max_x = min(self.width, x + radius + 1)
        min_y = max(0, y - radius)
        max_y = min(self.height, y + radius + 1)
        
        # Apply brush based on shape
        for cy in range(min_y, max_y):
            for cx in range(min_x, max_x):
                if brush_shape == 'square':
                    # Square brush - all points within the square are affected
                    distance_factor = 1.0
                else:  # circle
                    # Calculate distance from center (normalized)
                    dx, dy = cx - x, cy - y
                    distance = np.sqrt(dx*dx + dy*dy)
                    if distance > radius:
                        continue  # Skip points outside the circle
                    
                    # Apply hardness - affects how the brush fades at the edges
                    if radius == 0:  # Avoid division by zero
                        distance_factor = 1.0
                    else:
                        distance_factor = 1.0 - (distance / radius) ** (1.0 / max(0.01, hardness))
                
                # Apply the brush effect
                if distance_factor > 0:
                    current_value = int(new_data[cy, cx])
                    new_value = int(current_value * (1 - distance_factor) + value * distance_factor)
                    new_data[cy, cx] = new_value
        
        self.data = new_data
        self._save_state()
    
    def draw_line(self, x1: int, y1: int, x2: int, y2: int, value: int, 
                 brush_size: int = 1, brush_shape: str = 'circle', hardness: float = 1.0):
        """
        Draw a line on the image using Bresenham's line algorithm.
        
        Args:
            x1, y1: Starting point coordinates
            x2, y2: Ending point coordinates
            value: Grayscale value to draw with
            brush_size: Size of the brush
            brush_shape: Shape of the brush ('circle', 'square')
            hardness: Hardness of the brush (0.0 to 1.0)
        """
        # Make a copy of the current state before drawing
        original_data = self.data.copy()
        
        # Ensure coordinates are within image bounds
        x1 = max(0, min(x1, self.width - 1))
        y1 = max(0, min(y1, self.height - 1))
        x2 = max(0, min(x2, self.width - 1))
        y2 = max(0, min(y2, self.height - 1))
        
        # Create a copy of the current data for modification
        new_data = self.data.copy()
        
        # Bresenham's line algorithm
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        err = dx - dy
        
        x, y = x1, y1  # Start at the first point
        
        while True:
            # Draw at the current point
            self.draw_point(x, y, value, brush_size, brush_shape, hardness)
            
            # Exit if we've reached the end point
            if x == x2 and y == y2:
                break
                
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        
        # Since draw_point saves state on each call, we need to consolidate the history
        if len(self._history) > 1:
            # Keep only the final state and the original state
            final_state = self._history[-1]
            self._history = [original_data, final_state]
    
    def draw_rectangle(self, x1: int, y1: int, x2: int, y2: int, value: int, 
                      brush_size: int = 1, brush_shape: str = 'circle', hardness: float = 1.0, 
                      fill: bool = False):
        """
        Draw a rectangle on the image.
        
        Args:
            x1, y1: Top-left corner coordinates
            x2, y2: Bottom-right corner coordinates
            value: Grayscale value to draw with
            brush_size: Size of the brush
            brush_shape: Shape of the brush ('circle', 'square')
            hardness: Hardness of the brush (0.0 to 1.0)
            fill: Whether to fill the rectangle
        """
        # Ensure x1,y1 is the top-left and x2,y2 is the bottom-right
        x1, x2 = min(x1, x2), max(x1, x2)
        y1, y2 = min(y1, y2), max(y1, y2)
        
        # Ensure coordinates are within image bounds
        x1 = max(0, min(x1, self.width - 1))
        y1 = max(0, min(y1, self.height - 1))
        x2 = max(0, min(x2, self.width - 1))
        y2 = max(0, min(y2, self.height - 1))
        
        # Make a copy of the current state before drawing
        original_data = self.data.copy()
        
        if fill:
            # Create a copy of the current data for modification
            new_data = self.data.copy()
            
            # Fill the rectangle
            for y in range(y1, y2 + 1):
                for x in range(x1, x2 + 1):
                    if 0 <= y < self.height and 0 <= x < self.width:
                        new_data[y, x] = value
            
            self.data = new_data
            self._save_state()
        else:
            # Draw the four sides of the rectangle
            self.draw_line(x1, y1, x2, y1, value, brush_size, brush_shape, hardness)  # Top
            self.draw_line(x2, y1, x2, y2, value, brush_size, brush_shape, hardness)  # Right
            self.draw_line(x2, y2, x1, y2, value, brush_size, brush_shape, hardness)  # Bottom
            self.draw_line(x1, y2, x1, y1, value, brush_size, brush_shape, hardness)  # Left
            
            # Consolidate history
            if len(self._history) > 1:
                # Keep only the final state and the original state
                final_state = self._history[-1]
                self._history = [original_data, final_state]
    
    def draw_circle(self, center_x: int, center_y: int, radius: int, value: int,
                   brush_size: int = 1, brush_shape: str = 'circle', hardness: float = 1.0,
                   fill: bool = False):
        """
        Draw a circle on the image.
        
        Args:
            center_x, center_y: Center coordinates of the circle
            radius: Radius of the circle
            value: Grayscale value to draw with
            brush_size: Size of the brush
            brush_shape: Shape of the brush ('circle', 'square')
            hardness: Hardness of the brush (0.0 to 1.0)
            fill: Whether to fill the circle
        """
        # Make a copy of the current state before drawing
        original_data = self.data.copy()
        
        if fill:
            # Create a copy of the current data for modification
            new_data = self.data.copy()
            
            # Calculate bounds
            min_x = max(0, center_x - radius)
            max_x = min(self.width, center_x + radius + 1)
            min_y = max(0, center_y - radius)
            max_y = min(self.height, center_y + radius + 1)
            
            # Fill the circle
            for y in range(min_y, max_y):
                for x in range(min_x, max_x):
                    dx, dy = x - center_x, y - center_y
                    if dx*dx + dy*dy <= radius*radius:
                        new_data[y, x] = value
            
            self.data = new_data
            self._save_state()
        else:
            # Midpoint circle algorithm
            x = radius
            y = 0
            decision = 1 - radius
            
            while x >= y:
                # Draw 8 points for each step
                points = [
                    (center_x + x, center_y + y),
                    (center_x - x, center_y + y),
                    (center_x + x, center_y - y),
                    (center_x - x, center_y - y),
                    (center_x + y, center_y + x),
                    (center_x - y, center_y + x),
                    (center_x + y, center_y - x),
                    (center_x - y, center_y - x)
                ]
                
                for px, py in points:
                    if 0 <= px < self.width and 0 <= py < self.height:
                        # Draw directly on the data array for better performance
                        self.data[py, px] = value
                
                y += 1
                if decision <= 0:
                    decision += 2 * y + 1
                else:
                    x -= 1
                    decision += 2 * (y - x) + 1
            
            # Save the state after drawing the circle
            self._save_state()
        
        # Consolidate history
        if len(self._history) > 1:
            # Keep only the final state and the original state
            final_state = self._history[-1]
            self._history = [original_data, final_state]
    
    def flood_fill(self, x: int, y: int, value: int, tolerance: int = 0):
        """
        Fill an area with a specific value using flood fill algorithm.
        
        Args:
            x, y: Starting point coordinates
            value: Grayscale value to fill with
            tolerance: How much variation in grayscale value is allowed
        """
        if not (0 <= x < self.width and 0 <= y < self.height):
            return
        
        # Create a copy of the current data for modification
        new_data = self.data.copy()
        
        # Get the target value (the value we're replacing)
        target_value = int(self.data[y, x])
        
        # Don't do anything if the target is already the fill value
        if target_value == value:
            return
        
        # Define the valid range for replacement
        min_val = max(0, target_value - tolerance)
        max_val = min(255, target_value + tolerance)
        
        # Use a queue for the flood fill to avoid recursion stack overflow
        queue = [(x, y)]
        visited = set()
        
        while queue:
            cx, cy = queue.pop(0)
            
            if (cx, cy) in visited:
                continue
                
            visited.add((cx, cy))
            
            # Check if this pixel should be filled
            if not (0 <= cx < self.width and 0 <= cy < self.height):
                continue
                
            current_value = int(new_data[cy, cx])
            if not (min_val <= current_value <= max_val):
                continue
                
            # Fill this pixel
            new_data[cy, cx] = value
            
            # Add neighbors to the queue
            queue.append((cx + 1, cy))
            queue.append((cx - 1, cy))
            queue.append((cx, cy + 1))
            queue.append((cx, cy - 1))
        
        self.data = new_data
        self._save_state()
    
    def clear(self, value: int = 0):
        """
        Clear the image with a specific value.
        
        Args:
            value: Grayscale value to clear with (default: 0, black)
        """
        self.data.fill(value)
        self._save_state()
    
    def get_histogram(self) -> List[int]:
        """
        Get the histogram of the image.
        
        Returns:
            List[int]: Histogram data (counts for each grayscale value 0-255)
        """
        hist, _ = np.histogram(self.data, bins=256, range=(0, 256))
        return hist.tolist()
    
    def resize(self, new_width: int, new_height: int):
        """
        Resize the image using nearest neighbor interpolation.
        
        Args:
            new_width: New width of the image
            new_height: New height of the image
        """
        if new_width <= 0 or new_height <= 0:
            raise ValueError("Width and height must be positive")
            
        # Simple nearest-neighbor implementation
        x_ratio = float(self.width) / new_width
        y_ratio = float(self.height) / new_height
        
        new_data = np.zeros((new_height, new_width), dtype=np.uint8)
        
        for y in range(new_height):
            for x in range(new_width):
                px = min(self.width - 1, int(x * x_ratio))
                py = min(self.height - 1, int(y * y_ratio))
                new_data[y, x] = self.data[py, px]
        
        self.width = new_width
        self.height = new_height
        self.data = new_data
        self._save_state()
