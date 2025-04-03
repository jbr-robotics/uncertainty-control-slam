import streamlit as st
import numpy as np
import threading
import time
from typing import Optional

from ..submap_storage import SubmapStorage
from .state import SubmapAnalyzerState
from .components import SubmapListComponent, ControlPanelComponent, VersionSliderComponent
from .visualizer import display_submap

class SubmapAnalyzerApp:
    """Main Streamlit application for the Submap Analyzer GUI"""
    
    def __init__(self, storage: SubmapStorage):
        self.storage = storage
        
        # Don't set page config here, it should be done once before creating the app
        # to avoid "StreamlitAPIException: set_page_config() can only be called once per app"
        
        # Initialize components
        self.submap_list = SubmapListComponent(storage)
        self.control_panel = ControlPanelComponent(storage)
        self.version_slider = VersionSliderComponent(storage)
    
    def run(self):
        """Run the Streamlit application"""
        # Initialize the state
        SubmapAnalyzerState.initialize()
        
        # App title
        st.title("Cartographer Submap Analyzer")
        
        # Render sidebar components
        self.control_panel.render()
        st.sidebar.markdown("---")
        self.submap_list.render()
        
        # Render main content
        self._render_main_content()
    
    def _render_main_content(self):
        """Render the main content area"""
        # Render version slider which returns the selected version and history
        slider_result = self.version_slider.render()
        if not slider_result:
            return
        
        version_index, submap_history = slider_result
        
        # Get the selected submap data for the chosen version
        selected_submap = SubmapAnalyzerState.get_selected_submap()
        if not selected_submap:
            return
        
        # Ensure the version index is valid
        if 0 <= version_index < len(submap_history):
            submap_data = submap_history[version_index]
            
            # Display the submap
            trajectory_id, submap_index = selected_submap
            title = f"Submap ({trajectory_id}, {submap_index}) - Version {version_index}"
            display_submap(submap_data, title)
            
            # Space for future metrics
            st.markdown("---")
            st.markdown("## Metrics")
            st.info("Metrics visualization will be implemented in future updates.")
            
            # Add placeholder for future features
            with st.expander("Future Features"):
                st.write("""
                - Time series plots of submap metrics
                - Comparison between submap versions
                - Uncertainty visualization
                - Export functionality
                """)

def run_streamlit_app(storage: SubmapStorage):
    """
    Function to run the Streamlit app with the given SubmapStorage.
    This is the main entry point for the GUI.
    
    Args:
        storage: A SubmapStorage instance containing the submap data
    """
    try:
        # Handle case where storage might be None
        if storage is None:
            st.error("No submap storage provided. Cannot display submaps.")
            return
            
        # Create and run the app
        app = SubmapAnalyzerApp(storage)
        app.run()
    except Exception as e:
        st.error(f"Error running the app: {e}")
        st.exception(e)  # This will show a detailed traceback in the Streamlit UI 