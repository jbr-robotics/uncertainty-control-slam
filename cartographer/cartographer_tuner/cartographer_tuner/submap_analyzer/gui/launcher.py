#!/usr/bin/env python3

import sys
import signal
import os
import threading
import importlib.util
import streamlit as st
import subprocess
import atexit

from .app import run_streamlit_app

def is_streamlit_installed():
    """Check if streamlit is installed."""
    try:
        importlib.util.find_spec("streamlit")
        return True
    except ImportError:
        return False

def launch_gui():
    """
    Main entry point for launching the GUI.
    
    This function:
    1. Initializes the ROS handler
    2. Sets up signal handling for clean shutdown when appropriate
    3. Runs the Streamlit app with the ROS storage
    """
    # Check if streamlit is installed
    if not is_streamlit_installed():
        print("Error: Streamlit is not installed. Please install it with 'pip install streamlit'")
        sys.exit(1)
    
    # Check if we're being run directly by Streamlit
    is_streamlit_context = 'STREAMLIT_RUNTIME' in os.environ
    is_main_thread = threading.current_thread() is threading.main_thread()
    
    try:
        if is_streamlit_context:
            # If we're already in a Streamlit context, just run the app
            run_streamlit_app()
        else:
            # If we're not in a Streamlit context, launch Streamlit as a subprocess
            script_path = os.path.abspath(__file__)
            script_dir = os.path.dirname(script_path)
            entry_point = os.path.join(script_dir, "streamlit_entry_point.py")
            
            # Make sure the entry point exists
            if not os.path.exists(entry_point):
                print(f"Error: Streamlit entry point not found at {entry_point}")
                sys.exit(1)
                
            # Launch streamlit
            print(f"Launching Streamlit with {entry_point}")
            subprocess.run(["streamlit", "run", entry_point, '--server.headless', 'true'])
    except Exception as e:
        print(f"Error running Streamlit app: {e}")


if __name__ == "__main__":
    launch_gui() 