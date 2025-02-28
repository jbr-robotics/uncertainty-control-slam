#!/usr/bin/env python3
"""
PGM Editor - A simple editor for PGM (Portable Gray Map) images.
"""

import tkinter as tk
import sys
import os

from pgm_editor.controller.app_controller import AppController


def main():
    """Main entry point for the application."""
    # Create the root window
    root = tk.Tk()
    root.title("PGM Editor")
    
    # Create and run the application controller
    app = AppController(root)
    
    # Handle command line arguments
    if len(sys.argv) > 1:
        filepath = sys.argv[1]
        if os.path.exists(filepath) and filepath.lower().endswith('.pgm'):
            app._on_open(filepath)
    
    # Start the application
    app.run()


if __name__ == "__main__":
    main()
