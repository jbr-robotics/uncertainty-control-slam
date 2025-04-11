#!/usr/bin/env python3

import os
os.environ["STREAMLIT_HEADLESS"] = "true"
os.environ["STREAMLIT_BROWSER_GATHER_USAGE_STATS"] = "false"

import streamlit as st
import sys
import os

# Add parent directories to path for imports
parent_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

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
        run_streamlit_app()
    except Exception as e:
        st.error(f"Error starting ROS: {e}")