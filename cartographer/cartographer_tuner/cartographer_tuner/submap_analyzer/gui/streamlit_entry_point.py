#!/usr/bin/env python3

import streamlit as st
import sys
import os

# Add parent directories to path for imports
parent_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

from cartographer_tuner.submap_analyzer.gui.ros_handler import RosHandler
from cartographer_tuner.submap_analyzer.gui.app import run_streamlit_app

# Main entry point when run directly by Streamlit
if __name__ == "__main__":
    # Set up the page configuration
    st.set_page_config(
        page_title="Cartographer Submap Analyzer",
        page_icon="🗺️",
        layout="centered",
        initial_sidebar_state="expanded"
    )
    
    try:
        # Get the ROS handler instance and start it if needed
        ros_handler = RosHandler.get_instance()
        ros_handler.start()
        
        # Directly run the Streamlit app with the storage from RosHandler
        run_streamlit_app(ros_handler.storage)
        
    except Exception as e:
        st.error(f"Error starting ROS: {e}")
        st.error("Please check if ROS is running and try again.")
        
        # Provide help for debugging
        with st.expander("Troubleshooting"):
            st.markdown("""
            ### Common Issues:
            - ROS not initialized
            - ROS topics not available
            - Missing dependencies
            
            Try running from the command line:
            ```bash
            python -m cartographer_tuner.cli.submap_analyzer_gui
            ```
            """) 