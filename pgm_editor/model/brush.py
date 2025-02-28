import numpy as np
from typing import Tuple, Dict, Any, Optional


class Brush:
    """Base class for all brushes in the PGM editor."""
    
    def __init__(self, size: int = 1, value: int = 0, shape: str = 'circle', hardness: float = 1.0):
        """
        Initialize a brush.
        
        Args:
            size: Size of the brush in pixels
            value: Grayscale value (0-255)
            shape: Shape of the brush ('circle', 'square')
            hardness: Hardness of the brush (0.0 to 1.0)
        """
        self.size = max(1, size)
        self.value = max(0, min(255, value))
        self.shape = shape if shape in ['circle', 'square'] else 'circle'
        self.hardness = max(0.0, min(1.0, hardness))
        
        # Generate the brush mask
        self._generate_mask()
    
    def _generate_mask(self):
        """Generate the brush mask based on current properties."""
        radius = self.size // 2
        diameter = self.size
        
        # Create a square mask
        self.mask = np.zeros((diameter, diameter), dtype=np.float32)
        
        # Fill the mask based on the shape
        center = radius
        for y in range(diameter):
            for x in range(diameter):
                if self.shape == 'square':
                    # Square brush - all points within the square have full intensity
                    self.mask[y, x] = 1.0
                else:  # circle
                    # Calculate distance from center (normalized)
                    dx, dy = x - center, y - center
                    distance = np.sqrt(dx*dx + dy*dy)
                    
                    if distance <= radius:
                        # Apply hardness - affects how the brush fades at the edges
                        if radius == 0:  # Avoid division by zero
                            intensity = 1.0
                        else:
                            intensity = 1.0 - (distance / radius) ** (1.0 / max(0.01, self.hardness))
                        self.mask[y, x] = max(0.0, intensity)
    
    def update_properties(self, **kwargs):
        """
        Update brush properties.
        
        Args:
            **kwargs: Properties to update (size, value, shape, hardness)
        """
        changed = False
        
        if 'size' in kwargs and kwargs['size'] != self.size:
            self.size = max(1, kwargs['size'])
            changed = True
            
        if 'value' in kwargs and kwargs['value'] != self.value:
            self.value = max(0, min(255, kwargs['value']))
            
        if 'shape' in kwargs and kwargs['shape'] != self.shape:
            if kwargs['shape'] in ['circle', 'square']:
                self.shape = kwargs['shape']
                changed = True
                
        if 'hardness' in kwargs and kwargs['hardness'] != self.hardness:
            self.hardness = max(0.0, min(1.0, kwargs['hardness']))
            changed = True
        
        # Regenerate mask if necessary
        if changed:
            self._generate_mask()
    
    def get_properties(self) -> Dict[str, Any]:
        """
        Get the current brush properties.
        
        Returns:
            Dict[str, Any]: Dictionary of brush properties
        """
        return {
            'size': self.size,
            'value': self.value,
            'shape': self.shape,
            'hardness': self.hardness
        }
    
    def apply_at_point(self, image_data: np.ndarray, x: int, y: int) -> np.ndarray:
        """
        Apply the brush at a specific point on the image.
        
        Args:
            image_data: The image data to modify
            x, y: Coordinates where to apply the brush
            
        Returns:
            np.ndarray: Modified image data
        """
        # Create a copy of the image data
        result = image_data.copy()
        
        # Calculate the area to modify
        radius = self.size // 2
        min_x = max(0, x - radius)
        max_x = min(image_data.shape[1], x + radius + 1)
        min_y = max(0, y - radius)
        max_y = min(image_data.shape[0], y + radius + 1)
        
        # Calculate offsets into the brush mask
        brush_min_x = max(0, radius - x)
        brush_min_y = max(0, radius - y)
        
        # Apply the brush
        for cy in range(min_y, max_y):
            brush_y = brush_min_y + (cy - min_y)
            for cx in range(min_x, max_x):
                brush_x = brush_min_x + (cx - min_x)
                
                if 0 <= brush_y < self.size and 0 <= brush_x < self.size:
                    intensity = self.mask[brush_y, brush_x]
                    if intensity > 0:
                        current_value = result[cy, cx]
                        new_value = int(current_value * (1 - intensity) + self.value * intensity)
                        result[cy, cx] = new_value
        
        return result


class PencilBrush(Brush):
    """Pencil brush for drawing lines and freehand shapes."""
    
    def __init__(self, size: int = 1, value: int = 0, shape: str = 'circle', hardness: float = 1.0):
        super().__init__(size, value, shape, hardness)


class EraserBrush(Brush):
    """Eraser brush that sets pixels to the background value."""
    
    def __init__(self, size: int = 10, background_value: int = 255, shape: str = 'circle', hardness: float = 0.5):
        super().__init__(size, background_value, shape, hardness)
        self.background_value = background_value
    
    def update_properties(self, **kwargs):
        """Update eraser properties."""
        if 'background_value' in kwargs:
            self.background_value = max(0, min(255, kwargs['background_value']))
            self.value = self.background_value
        
        super().update_properties(**kwargs)


class ShapeBrush(Brush):
    """Base class for shape brushes (line, rectangle, circle)."""
    
    def __init__(self, size: int = 1, value: int = 0, shape: str = 'circle', hardness: float = 1.0, fill: bool = False):
        """
        Initialize a shape brush.
        
        Args:
            size: Size of the brush in pixels
            value: Grayscale value (0-255)
            shape: Shape of the brush ('circle', 'square')
            hardness: Hardness of the brush (0.0 to 1.0)
            fill: Whether to fill the shape
        """
        super().__init__(size, value, shape, hardness)
        self.start_point: Optional[Tuple[int, int]] = None
        self.end_point: Optional[Tuple[int, int]] = None
        self.fill = fill
    
    def start_shape(self, x: int, y: int):
        """
        Start drawing a shape.
        
        Args:
            x, y: Starting coordinates
        """
        self.start_point = (x, y)
        self.end_point = (x, y)
    
    def update_shape(self, x: int, y: int):
        """
        Update the end point of the shape.
        
        Args:
            x, y: New end coordinates
        """
        self.end_point = (x, y)
    
    def finish_shape(self):
        """Finish drawing the shape."""
        result = (self.start_point, self.end_point, self.get_properties())
        self.start_point = None
        self.end_point = None
        return result
    
    def update_properties(self, **kwargs):
        """Update shape brush properties."""
        if 'fill' in kwargs:
            self.fill = bool(kwargs['fill'])
        
        super().update_properties(**kwargs)
    
    def get_properties(self) -> Dict[str, Any]:
        """Get shape brush properties."""
        props = super().get_properties()
        props['fill'] = self.fill
        return props


class LineBrush(ShapeBrush):
    """Brush for drawing straight lines."""
    pass


class RectangleBrush(ShapeBrush):
    """Brush for drawing rectangles."""
    pass


class CircleBrush(ShapeBrush):
    """Brush for drawing circles."""
    pass


class FillBrush(Brush):
    """Brush for flood filling areas."""
    
    def __init__(self, value: int = 0, tolerance: int = 0, size: int = 1, shape: str = 'circle', hardness: float = 1.0):
        # FillBrush always uses size=1, but we accept the parameter for compatibility
        super().__init__(1, value, 'circle', 1.0)
        self.tolerance = max(0, min(255, tolerance))
    
    def update_properties(self, **kwargs):
        """Update fill brush properties."""
        if 'tolerance' in kwargs:
            self.tolerance = max(0, min(255, kwargs['tolerance']))
        
        # Only update value from kwargs, ignore other properties
        if 'value' in kwargs:
            self.value = max(0, min(255, kwargs['value']))
    
    def get_properties(self) -> Dict[str, Any]:
        """Get fill brush properties."""
        props = super().get_properties()
        props['tolerance'] = self.tolerance
        return props


# Factory function to create brushes
def create_brush(brush_type: str, **kwargs) -> Brush:
    """
    Create a brush of the specified type.
    
    Args:
        brush_type: Type of brush to create ('pencil', 'eraser', 'line', 'rectangle', 'circle', 'fill')
        **kwargs: Brush properties
        
    Returns:
        Brush: A brush instance
        
    Raises:
        ValueError: If brush_type is not recognized
    """
    brush_classes = {
        'pencil': PencilBrush,
        'point': PencilBrush,  # Map 'point' to PencilBrush
        'eraser': EraserBrush,
        'line': LineBrush,
        'rectangle': RectangleBrush,
        'circle': CircleBrush,
        'fill': FillBrush
    }
    
    if brush_type not in brush_classes:
        raise ValueError(f"Unknown brush type: {brush_type}")
    
    # Filter kwargs based on brush type
    filtered_kwargs = kwargs.copy()
    
    # FillBrush only accepts 'value' and 'tolerance'
    if brush_type == 'fill':
        allowed_params = {'value', 'tolerance'}
        filtered_kwargs = {k: v for k, v in kwargs.items() if k in allowed_params}
    
    # EraserBrush accepts 'size', 'background_value', 'shape', 'hardness'
    elif brush_type == 'eraser':
        allowed_params = {'size', 'background_value', 'shape', 'hardness'}
        filtered_kwargs = {k: v for k, v in kwargs.items() if k in allowed_params}
    
    # Shape brushes accept 'size', 'value', 'shape', 'hardness', 'fill'
    elif brush_type in ['line', 'rectangle', 'circle']:
        allowed_params = {'size', 'value', 'shape', 'hardness', 'fill'}
        filtered_kwargs = {k: v for k, v in kwargs.items() if k in allowed_params}
    
    # Other brushes accept 'size', 'value', 'shape', 'hardness'
    else:
        allowed_params = {'size', 'value', 'shape', 'hardness'}
        filtered_kwargs = {k: v for k, v in kwargs.items() if k in allowed_params}
    
    return brush_classes[brush_type](**filtered_kwargs)
