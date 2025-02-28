import tkinter as tk
from tkinter import ttk
from typing import Dict, Any, Optional, Tuple


class NewImageDialog:
    """Dialog for creating a new image."""
    
    def __init__(self, parent):
        """
        Initialize the dialog.
        
        Args:
            parent: Parent window
        """
        self.result = None
        
        # Create dialog window
        self.dialog = tk.Toplevel(parent)
        self.dialog.title("New Image")
        self.dialog.geometry("300x200")
        self.dialog.resizable(False, False)
        self.dialog.transient(parent)
        self.dialog.grab_set()
        
        # Center the dialog on the parent window
        parent_x = parent.winfo_x()
        parent_y = parent.winfo_y()
        parent_width = parent.winfo_width()
        parent_height = parent.winfo_height()
        
        x = parent_x + (parent_width - 300) // 2
        y = parent_y + (parent_height - 200) // 2
        
        self.dialog.geometry(f"+{x}+{y}")
        
        # Create widgets
        ttk.Label(self.dialog, text="Create a new PGM image:").pack(pady=10)
        
        # Width input
        width_frame = ttk.Frame(self.dialog)
        width_frame.pack(fill=tk.X, padx=20, pady=5)
        
        ttk.Label(width_frame, text="Width:").pack(side=tk.LEFT)
        self.width_var = tk.StringVar(value="128")
        ttk.Entry(width_frame, textvariable=self.width_var, width=10).pack(side=tk.RIGHT)
        
        # Height input
        height_frame = ttk.Frame(self.dialog)
        height_frame.pack(fill=tk.X, padx=20, pady=5)
        
        ttk.Label(height_frame, text="Height:").pack(side=tk.LEFT)
        self.height_var = tk.StringVar(value="128")
        ttk.Entry(height_frame, textvariable=self.height_var, width=10).pack(side=tk.RIGHT)
        
        # Buttons
        button_frame = ttk.Frame(self.dialog)
        button_frame.pack(fill=tk.X, padx=20, pady=20)
        
        ttk.Button(button_frame, text="Cancel", command=self._on_cancel).pack(side=tk.RIGHT)
        ttk.Button(button_frame, text="Create", command=self._on_create).pack(side=tk.RIGHT, padx=10)
        
        # Wait for the dialog to close
        self.dialog.wait_window()
    
    def _on_create(self):
        """Handle the Create button click."""
        try:
            width = int(self.width_var.get())
            height = int(self.height_var.get())
            
            if width <= 0 or height <= 0:
                raise ValueError("Width and height must be positive")
            
            self.result = (width, height)
            self.dialog.destroy()
        except ValueError as e:
            tk.messagebox.showerror("Invalid Input", str(e))
    
    def _on_cancel(self):
        """Handle the Cancel button click."""
        self.dialog.destroy()


class BrushPropertiesDialog:
    """Dialog for adjusting brush properties."""
    
    def __init__(self, parent, brush_type: str, current_properties: Dict[str, Any]):
        """
        Initialize the dialog.
        
        Args:
            parent: Parent window
            brush_type: Type of brush ('pencil', 'eraser', 'line', 'rectangle', 'circle', 'fill')
            current_properties: Current brush properties
        """
        self.result = None
        self.brush_type = brush_type
        self.current_properties = current_properties.copy()
        
        # Create dialog window
        self.dialog = tk.Toplevel(parent)
        self.dialog.title(f"{brush_type.capitalize()} Properties")
        self.dialog.geometry("350x350")
        self.dialog.resizable(False, False)
        self.dialog.transient(parent)
        self.dialog.grab_set()
        
        # Center the dialog on the parent window
        parent_x = parent.winfo_x()
        parent_y = parent.winfo_y()
        parent_width = parent.winfo_width()
        parent_height = parent.winfo_height()
        
        x = parent_x + (parent_width - 350) // 2
        y = parent_y + (parent_height - 350) // 2
        
        self.dialog.geometry(f"+{x}+{y}")
        
        # Create widgets
        ttk.Label(self.dialog, text=f"{brush_type.capitalize()} Properties:").pack(pady=10)
        
        # Create property controls based on brush type
        self._create_common_properties()
        
        if brush_type in ['line', 'rectangle', 'circle']:
            self._create_shape_properties()
        elif brush_type == 'fill':
            self._create_fill_properties()
        elif brush_type == 'eraser':
            self._create_eraser_properties()
        
        # Buttons
        button_frame = ttk.Frame(self.dialog)
        button_frame.pack(fill=tk.X, padx=20, pady=20)
        
        ttk.Button(button_frame, text="Cancel", command=self._on_cancel).pack(side=tk.RIGHT)
        ttk.Button(button_frame, text="Apply", command=self._on_apply).pack(side=tk.RIGHT, padx=10)
        
        # Wait for the dialog to close
        self.dialog.wait_window()
    
    def _create_common_properties(self):
        """Create controls for common brush properties."""
        # Size control
        size_frame = ttk.LabelFrame(self.dialog, text="Size")
        size_frame.pack(fill=tk.X, padx=20, pady=5)
        
        self.size_var = tk.IntVar(value=self.current_properties.get('size', 5))
        size_scale = ttk.Scale(
            size_frame,
            from_=1,
            to=50,
            orient=tk.HORIZONTAL,
            variable=self.size_var,
            length=250
        )
        size_scale.pack(side=tk.LEFT, padx=5, pady=5)
        
        size_label = ttk.Label(size_frame, textvariable=self.size_var, width=3)
        size_label.pack(side=tk.LEFT, padx=5)
        
        # Value control
        value_frame = ttk.LabelFrame(self.dialog, text="Value (0-255)")
        value_frame.pack(fill=tk.X, padx=20, pady=5)
        
        self.value_var = tk.IntVar(value=self.current_properties.get('value', 0))
        value_scale = ttk.Scale(
            value_frame,
            from_=0,
            to=255,
            orient=tk.HORIZONTAL,
            variable=self.value_var,
            length=250
        )
        value_scale.pack(side=tk.LEFT, padx=5, pady=5)
        
        value_label = ttk.Label(value_frame, textvariable=self.value_var, width=3)
        value_label.pack(side=tk.LEFT, padx=5)
        
        # Shape control
        if self.brush_type not in ['fill']:
            shape_frame = ttk.LabelFrame(self.dialog, text="Shape")
            shape_frame.pack(fill=tk.X, padx=20, pady=5)
            
            self.shape_var = tk.StringVar(value=self.current_properties.get('shape', 'circle'))
            ttk.Radiobutton(
                shape_frame, 
                text="Circle", 
                value="circle", 
                variable=self.shape_var
            ).pack(anchor=tk.W, padx=5, pady=2)
            
            ttk.Radiobutton(
                shape_frame, 
                text="Square", 
                value="square", 
                variable=self.shape_var
            ).pack(anchor=tk.W, padx=5, pady=2)
            
            # Hardness control
            hardness_frame = ttk.LabelFrame(self.dialog, text="Hardness")
            hardness_frame.pack(fill=tk.X, padx=20, pady=5)
            
            self.hardness_var = tk.DoubleVar(value=self.current_properties.get('hardness', 1.0))
            hardness_scale = ttk.Scale(
                hardness_frame,
                from_=0.0,
                to=1.0,
                orient=tk.HORIZONTAL,
                variable=self.hardness_var,
                length=250
            )
            hardness_scale.pack(side=tk.LEFT, padx=5, pady=5)
            
            hardness_label = ttk.Label(hardness_frame, text=f"{self.hardness_var.get():.1f}", width=3)
            hardness_label.pack(side=tk.LEFT, padx=5)
            
            # Update hardness label when the scale changes
            def update_hardness_label(*args):
                hardness_label.config(text=f"{self.hardness_var.get():.1f}")
            
            self.hardness_var.trace_add("write", update_hardness_label)
    
    def _create_shape_properties(self):
        """Create controls for shape brush properties."""
        # Fill option
        fill_frame = ttk.Frame(self.dialog)
        fill_frame.pack(fill=tk.X, padx=20, pady=5)
        
        self.fill_var = tk.BooleanVar(value=self.current_properties.get('fill', False))
        ttk.Checkbutton(
            fill_frame, 
            text="Fill Shape", 
            variable=self.fill_var
        ).pack(anchor=tk.W)
    
    def _create_fill_properties(self):
        """Create controls for fill brush properties."""
        # Tolerance control
        tolerance_frame = ttk.LabelFrame(self.dialog, text="Tolerance")
        tolerance_frame.pack(fill=tk.X, padx=20, pady=5)
        
        self.tolerance_var = tk.IntVar(value=self.current_properties.get('tolerance', 0))
        tolerance_scale = ttk.Scale(
            tolerance_frame,
            from_=0,
            to=255,
            orient=tk.HORIZONTAL,
            variable=self.tolerance_var,
            length=250
        )
        tolerance_scale.pack(side=tk.LEFT, padx=5, pady=5)
        
        tolerance_label = ttk.Label(tolerance_frame, textvariable=self.tolerance_var, width=3)
        tolerance_label.pack(side=tk.LEFT, padx=5)
    
    def _create_eraser_properties(self):
        """Create controls for eraser brush properties."""
        # Background value control
        bg_frame = ttk.LabelFrame(self.dialog, text="Background Value")
        bg_frame.pack(fill=tk.X, padx=20, pady=5)
        
        self.bg_value_var = tk.IntVar(value=self.current_properties.get('background_value', 255))
        bg_scale = ttk.Scale(
            bg_frame,
            from_=0,
            to=255,
            orient=tk.HORIZONTAL,
            variable=self.bg_value_var,
            length=250
        )
        bg_scale.pack(side=tk.LEFT, padx=5, pady=5)
        
        bg_label = ttk.Label(bg_frame, textvariable=self.bg_value_var, width=3)
        bg_label.pack(side=tk.LEFT, padx=5)
    
    def _on_apply(self):
        """Handle the Apply button click."""
        # Collect properties based on brush type
        properties = {}
        
        # Common properties
        properties['size'] = self.size_var.get()
        properties['value'] = self.value_var.get()
        
        if self.brush_type != 'fill':
            properties['shape'] = self.shape_var.get()
            properties['hardness'] = self.hardness_var.get()
        
        # Brush-specific properties
        if self.brush_type in ['line', 'rectangle', 'circle']:
            properties['fill'] = self.fill_var.get()
        elif self.brush_type == 'fill':
            properties['tolerance'] = self.tolerance_var.get()
        elif self.brush_type == 'eraser':
            properties['background_value'] = self.bg_value_var.get()
        
        self.result = properties
        self.dialog.destroy()
    
    def _on_cancel(self):
        """Handle the Cancel button click."""
        self.dialog.destroy()
