import numpy as np
import pytest

from pgm_editor.model.brush import (
    Brush, PencilBrush, EraserBrush, LineBrush, 
    RectangleBrush, CircleBrush, FillBrush, create_brush
)


class TestBrush:
    """Tests for the Brush class and its subclasses."""
    
    def test_base_brush_init(self):
        """Test base brush initialization."""
        brush = Brush(size=10, value=200, shape='circle', hardness=0.5)
        
        assert brush.size == 10
        assert brush.value == 200
        assert brush.shape == 'circle'
        assert brush.hardness == 0.5
        
        # Check that the mask was generated
        assert brush.mask.shape == (10, 10)
        assert brush.mask.dtype == np.float32
    
    def test_brush_init_defaults(self):
        """Test brush initialization with default values."""
        brush = Brush()
        
        assert brush.size == 1
        assert brush.value == 0
        assert brush.shape == 'circle'
        assert brush.hardness == 1.0
        
        # Check that the mask was generated
        assert brush.mask.shape == (1, 1)
        assert brush.mask.dtype == np.float32
    
    def test_brush_init_invalid_values(self):
        """Test brush initialization with invalid values."""
        # Size should be at least 1
        brush = Brush(size=0)
        assert brush.size == 1
        
        # Value should be between 0 and 255
        brush = Brush(value=-10)
        assert brush.value == 0
        
        brush = Brush(value=300)
        assert brush.value == 255
        
        # Shape should be 'circle' or 'square'
        brush = Brush(shape='invalid')
        assert brush.shape == 'circle'
        
        # Hardness should be between 0 and 1
        brush = Brush(hardness=-0.5)
        assert brush.hardness == 0.0
        
        brush = Brush(hardness=1.5)
        assert brush.hardness == 1.0
    
    def test_brush_update_properties(self):
        """Test updating brush properties."""
        brush = Brush(size=5, value=100)
        
        # Update size
        brush.update_properties(size=10)
        assert brush.size == 10
        assert brush.mask.shape == (10, 10)
        
        # Update value
        brush.update_properties(value=200)
        assert brush.value == 200
        
        # Update shape
        brush.update_properties(shape='square')
        assert brush.shape == 'square'
        
        # Update hardness
        brush.update_properties(hardness=0.5)
        assert brush.hardness == 0.5
        
        # Update multiple properties
        brush.update_properties(size=15, value=150, shape='circle', hardness=0.8)
        assert brush.size == 15
        assert brush.value == 150
        assert brush.shape == 'circle'
        assert brush.hardness == 0.8
        assert brush.mask.shape == (15, 15)
    
    def test_brush_get_properties(self):
        """Test getting brush properties."""
        brush = Brush(size=10, value=200, shape='square', hardness=0.5)
        
        props = brush.get_properties()
        assert props['size'] == 10
        assert props['value'] == 200
        assert props['shape'] == 'square'
        assert props['hardness'] == 0.5
    
    def test_brush_apply_at_point(self):
        """Test applying a brush at a point."""
        # Create a test image
        image_data = np.zeros((20, 20), dtype=np.uint8)
        
        # Create a brush
        brush = Brush(size=5, value=200, shape='circle', hardness=1.0)
        
        # Apply the brush
        result = brush.apply_at_point(image_data, 10, 10)
        
        # Check that the result is a copy
        assert result is not image_data
        
        # Check that the center pixel is set
        assert result[10, 10] == 200
        
        # Check that nearby pixels are also set (within brush radius)
        assert result[10, 11] > 0
        assert result[11, 10] > 0
        
        # Check that distant pixels are not set
        assert result[0, 0] == 0
        assert result[19, 19] == 0
    
    def test_pencil_brush(self):
        """Test pencil brush."""
        brush = PencilBrush(size=5, value=200)
        
        assert isinstance(brush, Brush)
        assert brush.size == 5
        assert brush.value == 200
    
    def test_eraser_brush(self):
        """Test eraser brush."""
        brush = EraserBrush(size=10, background_value=255)
        
        assert isinstance(brush, Brush)
        assert brush.size == 10
        assert brush.value == 255
        assert brush.background_value == 255
        
        # Update background value
        brush.update_properties(background_value=200)
        assert brush.background_value == 200
        assert brush.value == 200  # Value should match background_value
    
    def test_shape_brush(self):
        """Test shape brush base class."""
        brush = LineBrush(size=5, value=200)
        
        assert hasattr(brush, 'start_point')
        assert hasattr(brush, 'end_point')
        assert hasattr(brush, 'fill')
        
        # Test shape drawing methods
        brush.start_shape(10, 10)
        assert brush.start_point == (10, 10)
        assert brush.end_point == (10, 10)
        
        brush.update_shape(20, 20)
        assert brush.start_point == (10, 10)
        assert brush.end_point == (20, 20)
        
        result = brush.finish_shape()
        assert result == ((10, 10), (20, 20), brush.get_properties())
        assert brush.start_point is None
        assert brush.end_point is None
    
    def test_line_brush(self):
        """Test line brush."""
        brush = LineBrush(size=5, value=200)
        
        assert isinstance(brush, Brush)
        assert hasattr(brush, 'start_shape')
        assert hasattr(brush, 'update_shape')
        assert hasattr(brush, 'finish_shape')
    
    def test_rectangle_brush(self):
        """Test rectangle brush."""
        brush = RectangleBrush(size=5, value=200)
        
        assert isinstance(brush, Brush)
        assert hasattr(brush, 'start_shape')
        assert hasattr(brush, 'update_shape')
        assert hasattr(brush, 'finish_shape')
        
        # Test fill property
        assert brush.fill is False
        
        brush.update_properties(fill=True)
        assert brush.fill is True
        
        props = brush.get_properties()
        assert props['fill'] is True
    
    def test_circle_brush(self):
        """Test circle brush."""
        brush = CircleBrush(size=5, value=200)
        
        assert isinstance(brush, Brush)
        assert hasattr(brush, 'start_shape')
        assert hasattr(brush, 'update_shape')
        assert hasattr(brush, 'finish_shape')
        
        # Test fill property
        assert brush.fill is False
        
        brush.update_properties(fill=True)
        assert brush.fill is True
        
        props = brush.get_properties()
        assert props['fill'] is True
    
    def test_fill_brush(self):
        """Test fill brush."""
        brush = FillBrush(value=200, tolerance=10)
        
        assert isinstance(brush, Brush)
        assert brush.value == 200
        assert brush.tolerance == 10
        
        # Update tolerance
        brush.update_properties(tolerance=20)
        assert brush.tolerance == 20
        
        props = brush.get_properties()
        assert props['tolerance'] == 20
    
    def test_create_brush(self):
        """Test brush factory function."""
        # Test creating different brush types
        pencil = create_brush('pencil', size=5, value=200)
        assert isinstance(pencil, PencilBrush)
        
        eraser = create_brush('eraser', size=10, background_value=255)
        assert isinstance(eraser, EraserBrush)
        
        line = create_brush('line', size=3, value=150)
        assert isinstance(line, LineBrush)
        
        rectangle = create_brush('rectangle', size=3, value=150, fill=True)
        assert isinstance(rectangle, RectangleBrush)
        assert rectangle.fill is True
        
        circle = create_brush('circle', size=3, value=150)
        assert isinstance(circle, CircleBrush)
        
        fill = create_brush('fill', value=200, tolerance=10)
        assert isinstance(fill, FillBrush)
        assert fill.tolerance == 10
        
        # Test with invalid brush type
        with pytest.raises(ValueError):
            create_brush('invalid')
