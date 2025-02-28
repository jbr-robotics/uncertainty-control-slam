import os
import tempfile
import tkinter as tk
import numpy as np
import pytest
from unittest.mock import MagicMock, patch

from pgm_editor.model.pgm_image import PGMImage
from pgm_editor.model.brush import create_brush, PencilBrush, EraserBrush, LineBrush, RectangleBrush, CircleBrush, FillBrush
from pgm_editor.controller.app_controller import AppController
from pgm_editor.controller.tool_controller import ToolController


class TestIntegration:
    """Integration tests for the PGM editor application."""
    
    @pytest.fixture
    def setup_app(self):
        """Set up a test application with a mock view."""
        root = tk.Tk()
        
        # Create a controller with a mocked view
        with patch('tkinter.Tk'):
            with patch('tkinter.ttk.Frame'):
                with patch('tkinter.Canvas'):
                    app = AppController(root)
                    
                    # Mock the view methods
                    app.view.update_image = MagicMock()
                    app.view.set_status = MagicMock()
                    app.view.show_error = MagicMock()
                    app.view.show_info = MagicMock()
                    
                    yield app
    
    def test_create_new_image(self, setup_app):
        """Test creating a new image."""
        app = setup_app
        
        # Create a new image
        app._on_new(200, 150)
        
        # Check that the image was created with the correct dimensions
        assert app.image.width == 200
        assert app.image.height == 150
        
        # Check that the view was updated
        app.view.update_image.assert_called_once()
        app.view.set_status.assert_called_once()
    
    def test_drawing_operations(self, setup_app):
        """Test various drawing operations."""
        app = setup_app
        
        # Create a new image
        app._create_new_image(100, 100)
        
        # Draw a point
        app._on_tool_changed('pencil', {'size': 1, 'value': 100, 'shape': 'circle', 'hardness': 1.0})
        app._on_canvas_click(50, 50)
        assert app.image.data[50, 50] == 100  # Point should be drawn with value 100
        
        # Change the brush value and draw another point
        app._on_brush_property_changed('pencil', {'size': 5, 'value': 200, 'shape': 'circle', 'hardness': 1.0})
        app._on_canvas_click(60, 60)
        assert app.image.data[60, 60] == 200  # Point should be drawn with value 200
        
        # Test undo
        app._on_undo()
        assert app.image.data[60, 60] != 200  # Point should be gone
        
        # Test redo
        app._on_redo()
        assert app.image.data[60, 60] == 200  # Point should be back
    
    def test_save_and_load(self, setup_app):
        """Test saving and loading images."""
        app = setup_app
        
        # Create a new image
        app._create_new_image(50, 50)
        
        # Draw a point
        app._on_tool_changed('pencil', {'size': 5, 'value': 200, 'shape': 'circle', 'hardness': 1.0})
        app._on_canvas_click(25, 25)
        
        # Verify the point was drawn
        assert app.image.data[25, 25] == 200
        
        # Save to a temporary file
        with tempfile.NamedTemporaryFile(suffix='.pgm', delete=False) as tmp:
            filepath = tmp.name
        
        try:
            # Save the image
            app._save_to_file(filepath, 'P5')
            
            # Create a new image (to clear the current one)
            app._create_new_image(100, 100)
            assert app.image.width == 100
            assert app.image.data[25, 25] == 0  # Should be cleared
            
            # Load the saved image
            app._on_open(filepath)
            
            # Check that the loaded image has the correct dimensions
            assert app.image.width == 50
            assert app.image.height == 50
            
            # Check that the point was loaded correctly
            assert app.image.data[25, 25] == 200
        finally:
            # Clean up
            if os.path.exists(filepath):
                os.remove(filepath)
    
    def test_tool_controller(self):
        """Test the tool controller."""
        # Create a mock image update callback
        update_callback = MagicMock()
        
        # Create a tool controller
        tool_controller = ToolController(update_callback)
        
        # Create a test image
        image = PGMImage(width=100, height=100)
        
        # Test pencil tool
        tool_controller.set_tool('pencil', {'size': 5, 'value': 200, 'shape': 'circle', 'hardness': 1.0})
        tool_controller.start_drawing(50, 50, image)
        assert image.data[50, 50] == 200
        update_callback.assert_called_once()
        
        # Reset the mock
        update_callback.reset_mock()
        
        # Test line tool
        tool_controller.set_tool('line', {'size': 1, 'value': 150, 'shape': 'circle', 'hardness': 1.0})
        tool_controller.start_drawing(10, 10, image)
        tool_controller.continue_drawing(20, 20, image)  # Should not draw yet
        update_callback.assert_not_called()
        tool_controller.finish_drawing(90, 90, image)  # Now it should draw
        assert image.data[10, 10] == 150
        assert image.data[50, 50] == 150
        assert image.data[90, 90] == 150
        update_callback.assert_called_once()
        
        # Reset the mock
        update_callback.reset_mock()
        
        # Test rectangle tool
        tool_controller.set_tool('rectangle', {'size': 1, 'value': 100, 'shape': 'circle', 'hardness': 1.0, 'fill': True})
        tool_controller.start_drawing(20, 20, image)
        tool_controller.finish_drawing(40, 40, image)
        assert image.data[20, 20] == 100
        assert image.data[30, 30] == 100
        assert image.data[40, 40] == 100
        update_callback.assert_called_once()
        
        # Reset the mock
        update_callback.reset_mock()
        
        # Test circle tool
        tool_controller.set_tool('circle', {'size': 1, 'value': 50, 'shape': 'circle', 'hardness': 1.0, 'fill': True})
        tool_controller.start_drawing(50, 50, image)
        tool_controller.finish_drawing(60, 50, image)  # Radius = 10
        assert image.data[50, 50] == 50  # Center
        assert image.data[50, 45] == 50  # Inside circle
        update_callback.assert_called_once()
        
        # Reset the mock
        update_callback.reset_mock()
        
        # Test fill tool
        tool_controller.set_tool('fill', {'value': 255, 'tolerance': 10})
        tool_controller.start_drawing(5, 5, image)  # Fill the background
        assert image.data[0, 0] == 255
        update_callback.assert_called_once()
    
    def test_brush_factory(self):
        """Test the brush factory with different brush types."""
        # Test creating different brush types
        brushes = [
            ('pencil', PencilBrush),
            ('eraser', EraserBrush),
            ('line', LineBrush),
            ('rectangle', RectangleBrush),
            ('circle', CircleBrush),
            ('fill', FillBrush)
        ]
        
        for brush_type, brush_class in brushes:
            brush = create_brush(brush_type)
            assert isinstance(brush, brush_class)
        
        # Test with invalid brush type
        with pytest.raises(ValueError):
            create_brush('invalid')

    def test_undo_redo(self, setup_app):
        """Test undo and redo functionality."""
        app = setup_app
        
        # Create a new image
        app._create_new_image(50, 50)
        
        # Initial state - all pixels should be 0
        assert app.image.data[10, 10] == 0
        
        # Draw a point
        app._on_tool_changed('point', {'size': 3, 'value': 200, 'shape': 'circle', 'hardness': 1.0})
        app._on_canvas_click(10, 10)
        
        # Verify the point was drawn
        assert app.image.data[10, 10] == 200
        
        # Draw another point
        app._on_canvas_click(20, 20)
        assert app.image.data[20, 20] == 200
        
        # Undo the second point
        app._on_undo()
        assert app.image.data[20, 20] == 0  # Second point should be gone
        assert app.image.data[10, 10] == 200  # First point should still be there
        
        # Undo the first point
        app._on_undo()
        assert app.image.data[10, 10] == 0  # First point should be gone
        
        # Redo the first point
        app._on_redo()
        assert app.image.data[10, 10] == 200  # First point should be back
        assert app.image.data[20, 20] == 0  # Second point should still be gone
        
        # Redo the second point
        app._on_redo()
        assert app.image.data[20, 20] == 200  # Second point should be back
