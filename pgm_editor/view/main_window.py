import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import os
from typing import Callable, Dict, Any, Optional

from .canvas import ImageCanvas
from .dialogs import NewImageDialog, BrushPropertiesDialog


class MainWindow:
    """Main window of the PGM editor application."""
    
    def __init__(self, root: tk.Tk):
        """
        Initialize the main window.
        
        Args:
            root: The root Tkinter window
        """
        self.root = root
        self.root.title("PGM Editor")
        self.root.geometry("1000x700")
        
        # Set up callbacks dictionary
        self.callbacks: Dict[str, Callable] = {}
        
        # Create the main UI components
        self._create_menu()
        self._create_toolbar()
        self._create_statusbar()
        self._create_main_area()
        
        # Configure grid weights
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_rowconfigure(1, weight=1)
        
        # Set up the current brush info
        self.current_brush_type = "pencil"
        self.current_brush_properties = {
            'size': 5,
            'value': 0,
            'shape': 'circle',
            'hardness': 1.0
        }
    
    def _create_menu(self):
        """Create the application menu bar."""
        self.menu_bar = tk.Menu(self.root)
        
        # File menu
        file_menu = tk.Menu(self.menu_bar, tearoff=0)
        file_menu.add_command(label="New", command=self._on_new)
        file_menu.add_command(label="Open", command=self._on_open)
        file_menu.add_command(label="Save", command=self._on_save)
        file_menu.add_command(label="Save As", command=self._on_save_as)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.root.quit)
        self.menu_bar.add_cascade(label="File", menu=file_menu)
        
        # Edit menu
        edit_menu = tk.Menu(self.menu_bar, tearoff=0)
        edit_menu.add_command(label="Undo", command=self._on_undo)
        edit_menu.add_command(label="Redo", command=self._on_redo)
        edit_menu.add_separator()
        edit_menu.add_command(label="Clear", command=self._on_clear)
        self.menu_bar.add_cascade(label="Edit", menu=edit_menu)
        
        # View menu
        view_menu = tk.Menu(self.menu_bar, tearoff=0)
        view_menu.add_command(label="Zoom In", command=self._on_zoom_in)
        view_menu.add_command(label="Zoom Out", command=self._on_zoom_out)
        view_menu.add_command(label="Reset Zoom", command=self._on_reset_zoom)
        view_menu.add_separator()
        
        # View options submenu
        self.show_grid_var = tk.BooleanVar(value=False)
        self.show_values_var = tk.BooleanVar(value=False)
        view_menu.add_checkbutton(label="Show Grid", variable=self.show_grid_var, 
                                 command=self._on_toggle_grid)
        view_menu.add_checkbutton(label="Show Values", variable=self.show_values_var,
                                 command=self._on_toggle_values)
        self.menu_bar.add_cascade(label="View", menu=view_menu)
        
        # Help menu
        help_menu = tk.Menu(self.menu_bar, tearoff=0)
        help_menu.add_command(label="About", command=self._on_about)
        self.menu_bar.add_cascade(label="Help", menu=help_menu)
        
        self.root.config(menu=self.menu_bar)
    
    def _create_toolbar(self):
        """Create the toolbar with drawing tools and options."""
        self.toolbar_frame = ttk.Frame(self.root)
        self.toolbar_frame.grid(row=0, column=0, sticky="ew", padx=5, pady=5)
        
        # Tool selection
        ttk.Label(self.toolbar_frame, text="Tool:").pack(side=tk.LEFT, padx=5)
        
        self.tool_var = tk.StringVar(value="pencil")
        tools = [
            ("Pencil", "pencil"),
            ("Eraser", "eraser"),
            ("Line", "line"),
            ("Rectangle", "rectangle"),
            ("Circle", "circle"),
            ("Fill", "fill")
        ]
        
        for text, value in tools:
            ttk.Radiobutton(
                self.toolbar_frame, 
                text=text, 
                value=value, 
                variable=self.tool_var,
                command=self._on_tool_changed
            ).pack(side=tk.LEFT, padx=2)
        
        # Brush properties button
        ttk.Button(
            self.toolbar_frame, 
            text="Brush Properties", 
            command=self._on_brush_properties
        ).pack(side=tk.LEFT, padx=10)
        
        # Value slider
        ttk.Label(self.toolbar_frame, text="Value:").pack(side=tk.LEFT, padx=5)
        self.value_var = tk.IntVar(value=0)
        self.value_slider = ttk.Scale(
            self.toolbar_frame,
            from_=0,
            to=255,
            orient=tk.HORIZONTAL,
            variable=self.value_var,
            length=100,
            command=self._on_value_changed
        )
        self.value_slider.pack(side=tk.LEFT, padx=5)
        
        # Value display
        self.value_label = ttk.Label(self.toolbar_frame, text="0")
        self.value_label.pack(side=tk.LEFT, padx=5)
        
        # Size slider
        ttk.Label(self.toolbar_frame, text="Size:").pack(side=tk.LEFT, padx=5)
        self.size_var = tk.IntVar(value=5)
        self.size_slider = ttk.Scale(
            self.toolbar_frame,
            from_=1,
            to=50,
            orient=tk.HORIZONTAL,
            variable=self.size_var,
            length=100,
            command=self._on_size_changed
        )
        self.size_slider.pack(side=tk.LEFT, padx=5)
        
        # Size display
        self.size_label = ttk.Label(self.toolbar_frame, text="5")
        self.size_label.pack(side=tk.LEFT, padx=5)
    
    def _create_statusbar(self):
        """Create the status bar at the bottom of the window."""
        self.status_frame = ttk.Frame(self.root)
        self.status_frame.grid(row=2, column=0, sticky="ew")
        
        self.status_label = ttk.Label(self.status_frame, text="Ready")
        self.status_label.pack(side=tk.LEFT, padx=5)
        
        self.position_label = ttk.Label(self.status_frame, text="")
        self.position_label.pack(side=tk.RIGHT, padx=5)
        
        self.image_info_label = ttk.Label(self.status_frame, text="No image")
        self.image_info_label.pack(side=tk.RIGHT, padx=5)
    
    def _create_main_area(self):
        """Create the main area with the canvas and scrollbars."""
        self.main_frame = ttk.Frame(self.root)
        self.main_frame.grid(row=1, column=0, sticky="nsew", padx=5, pady=5)
        
        # Configure grid weights for the main frame
        self.main_frame.grid_columnconfigure(0, weight=1)
        self.main_frame.grid_rowconfigure(0, weight=1)
        
        # Create canvas with scrollbars
        self.canvas_frame = ttk.Frame(self.main_frame)
        self.canvas_frame.grid(row=0, column=0, sticky="nsew")
        
        # Create the image canvas
        self.image_canvas = ImageCanvas(self.canvas_frame)
        self.image_canvas.pack(fill=tk.BOTH, expand=True)
        
        # Set up canvas callbacks
        self.image_canvas.set_mouse_position_callback(self._update_position_info)
    
    def set_callbacks(self, callbacks: Dict[str, Callable]):
        """
        Set the callbacks for various actions.
        
        Args:
            callbacks: Dictionary of callback functions
        """
        self.callbacks = callbacks
        
        # Connect canvas events to controller callbacks
        if 'on_canvas_click' in callbacks:
            self.image_canvas.set_click_callback(callbacks['on_canvas_click'])
        
        if 'on_canvas_drag' in callbacks:
            self.image_canvas.set_drag_callback(callbacks['on_canvas_drag'])
        
        if 'on_canvas_release' in callbacks:
            self.image_canvas.set_release_callback(callbacks['on_canvas_release'])
    
    def update_image(self, image_data, width: int, height: int):
        """
        Update the displayed image.
        
        Args:
            image_data: NumPy array with image data
            width: Width of the image
            height: Height of the image
        """
        self.image_canvas.update_image(image_data)
        self.image_info_label.config(text=f"{width}x{height}")
    
    def _update_position_info(self, x: int, y: int, value: Optional[int] = None):
        """
        Update the position information in the status bar.
        
        Args:
            x, y: Mouse coordinates
            value: Optional grayscale value at the position
        """
        if value is not None:
            self.position_label.config(text=f"X: {x}, Y: {y}, Value: {value}")
        else:
            self.position_label.config(text=f"X: {x}, Y: {y}")
    
    def set_status(self, message: str):
        """
        Set the status message.
        
        Args:
            message: Status message to display
        """
        self.status_label.config(text=message)
    
    def _on_new(self):
        """Handle the New menu action."""
        dialog = NewImageDialog(self.root)
        if dialog.result:
            width, height = dialog.result
            if 'on_new' in self.callbacks:
                self.callbacks['on_new'](width, height)
    
    def _on_open(self):
        """Handle the Open menu action."""
        filepath = filedialog.askopenfilename(
            title="Open PGM File",
            filetypes=[("PGM files", "*.pgm"), ("All files", "*.*")]
        )
        
        if filepath:
            if 'on_open' in self.callbacks:
                self.callbacks['on_open'](filepath)
    
    def _on_save(self):
        """Handle the Save menu action."""
        if 'on_save' in self.callbacks:
            self.callbacks['on_save']()
    
    def _on_save_as(self):
        """Handle the Save As menu action."""
        filepath = filedialog.asksaveasfilename(
            title="Save PGM File",
            defaultextension=".pgm",
            filetypes=[("PGM files", "*.pgm"), ("All files", "*.*")]
        )
        
        if filepath:
            # Ask for format type
            format_type = tk.StringVar(value="P5")
            dialog = tk.Toplevel(self.root)
            dialog.title("PGM Format")
            dialog.geometry("300x150")
            dialog.resizable(False, False)
            dialog.transient(self.root)
            dialog.grab_set()
            
            ttk.Label(dialog, text="Select PGM format:").pack(pady=10)
            
            ttk.Radiobutton(dialog, text="P2 (ASCII)", variable=format_type, value="P2").pack(anchor=tk.W, padx=20)
            ttk.Radiobutton(dialog, text="P5 (Binary)", variable=format_type, value="P5").pack(anchor=tk.W, padx=20)
            
            def on_ok():
                if 'on_save_as' in self.callbacks:
                    self.callbacks['on_save_as'](filepath, format_type.get())
                dialog.destroy()
            
            ttk.Button(dialog, text="OK", command=on_ok).pack(pady=10)
            
            dialog.wait_window()
    
    def _on_undo(self):
        """Handle the Undo menu action."""
        if 'on_undo' in self.callbacks:
            self.callbacks['on_undo']()
    
    def _on_redo(self):
        """Handle the Redo menu action."""
        if 'on_redo' in self.callbacks:
            self.callbacks['on_redo']()
    
    def _on_clear(self):
        """Handle the Clear menu action."""
        if messagebox.askyesno("Clear Image", "Are you sure you want to clear the image?"):
            if 'on_clear' in self.callbacks:
                self.callbacks['on_clear'](self.value_var.get())
    
    def _on_zoom_in(self):
        """Handle the Zoom In menu action."""
        self.image_canvas.zoom_in()
    
    def _on_zoom_out(self):
        """Handle the Zoom Out menu action."""
        self.image_canvas.zoom_out()
    
    def _on_reset_zoom(self):
        """Handle the Reset Zoom menu action."""
        self.image_canvas.reset_zoom()
    
    def _on_toggle_grid(self):
        """Handle the Show Grid toggle."""
        self.image_canvas.set_show_grid(self.show_grid_var.get())
    
    def _on_toggle_values(self):
        """Handle the Show Values toggle."""
        self.image_canvas.set_show_values(self.show_values_var.get())
    
    def _on_about(self):
        """Handle the About menu action."""
        messagebox.showinfo(
            "About PGM Editor",
            "PGM Editor\n\n"
            "A simple editor for PGM (Portable Gray Map) images.\n\n"
            "Created with Python and Tkinter."
        )
    
    def _on_tool_changed(self):
        """Handle tool selection change."""
        tool_type = self.tool_var.get()
        self.current_brush_type = tool_type
        
        if 'on_tool_changed' in self.callbacks:
            self.callbacks['on_tool_changed'](tool_type, self.current_brush_properties)
    
    def _on_value_changed(self, value):
        """
        Handle value slider change.
        
        Args:
            value: New value from the slider
        """
        value = int(float(value))
        self.value_label.config(text=str(value))
        self.current_brush_properties['value'] = value
        
        if 'on_brush_property_changed' in self.callbacks:
            self.callbacks['on_brush_property_changed'](self.current_brush_type, self.current_brush_properties)
    
    def _on_size_changed(self, size):
        """
        Handle size slider change.
        
        Args:
            size: New size from the slider
        """
        size = int(float(size))
        self.size_label.config(text=str(size))
        self.current_brush_properties['size'] = size
        
        if 'on_brush_property_changed' in self.callbacks:
            self.callbacks['on_brush_property_changed'](self.current_brush_type, self.current_brush_properties)
    
    def _on_brush_properties(self):
        """Handle the Brush Properties button click."""
        dialog = BrushPropertiesDialog(self.root, self.current_brush_type, self.current_brush_properties)
        
        if dialog.result:
            self.current_brush_properties.update(dialog.result)
            
            # Update sliders to match
            if 'size' in dialog.result:
                self.size_var.set(dialog.result['size'])
                self.size_label.config(text=str(dialog.result['size']))
            
            if 'value' in dialog.result:
                self.value_var.set(dialog.result['value'])
                self.value_label.config(text=str(dialog.result['value']))
            
            if 'on_brush_property_changed' in self.callbacks:
                self.callbacks['on_brush_property_changed'](self.current_brush_type, self.current_brush_properties)
    
    def show_error(self, title: str, message: str):
        """
        Show an error message.
        
        Args:
            title: Error title
            message: Error message
        """
        messagebox.showerror(title, message)
    
    def show_info(self, title: str, message: str):
        """
        Show an information message.
        
        Args:
            title: Info title
            message: Info message
        """
        messagebox.showinfo(title, message)
