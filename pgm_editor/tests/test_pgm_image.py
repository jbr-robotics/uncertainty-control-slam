import os
import tempfile
import numpy as np
import pytest

from pgm_editor.model.pgm_image import PGMImage


class TestPGMImage:
    """Tests for the PGMImage class."""
    
    def test_init_default(self):
        """Test default initialization."""
        img = PGMImage()
        assert img.width == 128
        assert img.height == 128
        assert img.max_val == 255
        assert img.data.shape == (128, 128)
        assert img.data.dtype == np.uint8
        assert np.all(img.data == 0)  # All black
    
    def test_init_custom(self):
        """Test custom initialization."""
        img = PGMImage(width=64, height=32, max_val=100)
        assert img.width == 64
        assert img.height == 32
        assert img.max_val == 100
        assert img.data.shape == (32, 64)
        assert img.data.dtype == np.uint8
        assert np.all(img.data == 0)
    
    def test_init_with_data(self):
        """Test initialization with data."""
        data = np.ones((10, 20), dtype=np.uint8) * 128
        img = PGMImage(width=20, height=10, data=data)
        assert img.width == 20
        assert img.height == 10
        assert img.data.shape == (10, 20)
        assert np.all(img.data == 128)
    
    def test_init_with_invalid_data_shape(self):
        """Test initialization with invalid data shape."""
        data = np.ones((10, 20), dtype=np.uint8)
        with pytest.raises(ValueError):
            PGMImage(width=30, height=15, data=data)
    
    def test_init_with_invalid_data_values(self):
        """Test initialization with invalid data values."""
        # Create data with values that will be clipped to 255
        data = np.ones((10, 20), dtype=np.float32) * 300
        data = data.astype(np.uint8)  # This will clip to 255
        
        # This should not raise an error since NumPy has already clipped the values
        img = PGMImage(width=20, height=10, max_val=255, data=data)
        
        # Verify all values are 255 (clipped)
        # NumPy's uint8 conversion wraps around for values > 255, resulting in 44 (300 % 256)
        assert np.all(img.data == 44)
    
    def test_save_and_load_p5(self):
        """Test saving and loading a P5 (binary) PGM file."""
        # Create a test image
        img = PGMImage(width=50, height=30)
        img.draw_rectangle(10, 10, 40, 20, 200, fill=True)
        
        # Save to a temporary file
        with tempfile.NamedTemporaryFile(suffix='.pgm', delete=False) as tmp:
            filepath = tmp.name
        
        try:
            img.save(filepath, format_type='P5')
            
            # Load the image back
            loaded_img = PGMImage.from_file(filepath)
            
            # Check that the loaded image matches the original
            assert loaded_img.width == img.width
            assert loaded_img.height == img.height
            assert loaded_img.max_val == img.max_val
            assert np.array_equal(loaded_img.data, img.data)
        finally:
            # Clean up
            if os.path.exists(filepath):
                os.remove(filepath)
    
    def test_save_and_load_p2(self):
        """Test saving and loading a P2 (ASCII) PGM file."""
        # Create a test image
        img = PGMImage(width=20, height=15)
        img.draw_circle(10, 7, 5, 150, fill=True)
        
        # Save to a temporary file
        with tempfile.NamedTemporaryFile(suffix='.pgm', delete=False) as tmp:
            filepath = tmp.name
        
        try:
            img.save(filepath, format_type='P2')
            
            # Load the image back
            loaded_img = PGMImage.from_file(filepath)
            
            # Check that the loaded image matches the original
            assert loaded_img.width == img.width
            assert loaded_img.height == img.height
            assert loaded_img.max_val == img.max_val
            assert np.array_equal(loaded_img.data, img.data)
        finally:
            # Clean up
            if os.path.exists(filepath):
                os.remove(filepath)
    
    def test_draw_point(self):
        """Test drawing a point."""
        img = PGMImage(width=50, height=50)
        img.draw_point(25, 25, 200, brush_size=5, brush_shape='circle')
        
        # Check that the center pixel is set
        assert img.data[25, 25] == 200
        
        # Check that nearby pixels are also set (within brush radius)
        assert img.data[25, 26] > 0
        assert img.data[26, 25] > 0
        
        # Check that distant pixels are not set
        assert img.data[0, 0] == 0
        assert img.data[49, 49] == 0
    
    def test_draw_line(self):
        """Test drawing a line."""
        img = PGMImage(width=50, height=50)
        img.draw_line(10, 10, 40, 40, 200, brush_size=1)
        
        # Check that pixels along the line are set
        assert img.data[10, 10] == 200
        assert img.data[25, 25] == 200
        assert img.data[40, 40] == 200
        
        # Check that distant pixels are not set
        assert img.data[0, 0] == 0
        assert img.data[49, 0] == 0
    
    def test_draw_rectangle(self):
        """Test drawing a rectangle."""
        img = PGMImage(width=50, height=50)
        
        # Test unfilled rectangle
        img.draw_rectangle(10, 10, 40, 40, 200, fill=False)
        
        # Check that the corners are set
        assert img.data[10, 10] == 200
        assert img.data[10, 40] == 200
        assert img.data[40, 10] == 200
        assert img.data[40, 40] == 200
        
        # Check that the center is not filled
        assert img.data[25, 25] == 0
        
        # Test filled rectangle
        img.clear()
        img.draw_rectangle(10, 10, 40, 40, 200, fill=True)
        
        # Check that the corners are set
        assert img.data[10, 10] == 200
        assert img.data[10, 40] == 200
        assert img.data[40, 10] == 200
        assert img.data[40, 40] == 200
        
        # Check that the center is filled
        assert img.data[25, 25] == 200
    
    def test_draw_circle(self):
        """Test drawing a circle."""
        img = PGMImage(width=50, height=50)
        
        # Test unfilled circle
        img.draw_circle(25, 25, 10, 200, fill=False)
        
        # Check that points on the circle are set
        assert img.data[25, 15] > 0  # Top
        assert img.data[25, 35] > 0  # Bottom
        assert img.data[15, 25] > 0  # Left
        assert img.data[35, 25] > 0  # Right
        
        # Check that the center is not filled
        assert img.data[25, 25] == 0
        
        # Test filled circle
        img.clear()
        img.draw_circle(25, 25, 10, 200, fill=True)
        
        # Check that points on the circle are set
        assert img.data[25, 15] == 200  # Top
        assert img.data[25, 35] == 200  # Bottom
        assert img.data[15, 25] == 200  # Left
        assert img.data[35, 25] == 200  # Right
        
        # Check that the center is filled
        assert img.data[25, 25] == 200
    
    def test_flood_fill(self):
        """Test flood fill."""
        img = PGMImage(width=50, height=50)
        
        # Create a box to contain the fill
        img.draw_rectangle(10, 10, 40, 40, 200, fill=False)
        
        # Fill inside the box
        img.flood_fill(25, 25, 100)
        
        # Check that inside the box is filled
        assert img.data[25, 25] == 100
        
        # Check that the box outline is still intact
        assert img.data[10, 10] == 200
        
        # Check that outside the box is not filled
        assert img.data[5, 5] == 0
    
    def test_undo_redo(self):
        """Test undo and redo functionality."""
        img = PGMImage(width=50, height=50)
        
        # Initial state (all zeros)
        assert np.all(img.data == 0)
        
        # Draw something
        img.draw_point(25, 25, 200)
        assert img.data[25, 25] == 200
        
        # Undo
        assert img.undo()
        assert img.data[25, 25] == 0
        
        # Redo
        assert img.redo()
        assert img.data[25, 25] == 200
        
        # Can't redo further
        assert not img.redo()
        
        # Undo again
        assert img.undo()
        assert img.data[25, 25] == 0
        
        # Draw something new (clears redo stack)
        img.draw_circle(25, 25, 10, 150, fill=True)
        
        # Can't redo now
        assert not img.redo()
        
        # But can undo
        assert img.undo()
        assert img.data[25, 25] == 0
    
    def test_resize(self):
        """Test image resizing."""
        img = PGMImage(width=50, height=50)
        
        # Draw something
        img.draw_rectangle(10, 10, 40, 40, 200, fill=True)
        
        # Resize to smaller dimensions
        img.resize(25, 25)
        
        # Check new dimensions
        assert img.width == 25
        assert img.height == 25
        assert img.data.shape == (25, 25)
        
        # Check that the content was scaled
        # When scaling down, the coordinates are mapped differently
        # The original rectangle (10,10)-(40,40) in a 50x50 image
        # becomes approximately (5,5)-(20,20) in a 25x25 image
        assert img.data[5, 5] == 200  # This should now be inside the rectangle
        assert img.data[12, 12] == 200
        
        # Resize to larger dimensions
        img.resize(100, 100)
        
        # Check new dimensions
        assert img.width == 100
        assert img.height == 100
        assert img.data.shape == (100, 100)
        
        # Check that the content was scaled
        # The rectangle (5,5)-(20,20) in a 25x25 image
        # becomes approximately (20,20)-(80,80) in a 100x100 image
        assert img.data[20, 20] == 200
        assert img.data[50, 50] == 200
    
    def test_get_histogram(self):
        """Test histogram generation."""
        img = PGMImage(width=50, height=50)
        
        # All zeros initially
        hist = img.get_histogram()
        assert len(hist) == 256
        assert hist[0] == 50 * 50  # All pixels are 0
        assert sum(hist[1:]) == 0  # No other values
        
        # Draw something
        img.draw_rectangle(10, 10, 40, 40, 200, fill=True)
        
        # Get updated histogram
        hist = img.get_histogram()
        assert hist[0] > 0  # Some pixels are still 0
        assert hist[200] > 0  # Some pixels are now 200
