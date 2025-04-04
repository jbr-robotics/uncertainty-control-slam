import streamlit as st

from cartographer_tuner.submap_analyzer.gui.state import SubmapAnalyzerState

class VersionSliderComponent:
    def __init__(self):
        pass
    
    def render(self):
        selected_submap = SubmapAnalyzerState.get_selected_submap()
        
        if not selected_submap:
            st.warning("No submap selected. Please select a submap from the sidebar.")
            return
        
        trajectory_id, submap_index = selected_submap
        
        st.markdown(f"## Submap ({trajectory_id}, {submap_index})")
        
        if SubmapAnalyzerState.is_data_outdated():
            st.warning("⚠️ Click 'Update Submap Data' in the sidebar to load data for this submap")
            return None
        
        version_count = SubmapAnalyzerState.get_version_count()
        
        if version_count <= 0:
            st.warning("No data available. Click 'Update Submap Data' to load versions.")
            return None
        
        st.markdown(f"### Version History: {version_count} versions")
        
        current_version = SubmapAnalyzerState.get_selected_version()
        
        current_version = min(current_version, version_count - 1)
        
        if version_count <= 1:
            st.text("Only one version available (version 0)")
            new_version = 0
        else:
            new_version = st.slider(
                "Select Version", 
                min_value=0, 
                max_value=version_count-1,
                value=current_version,
                key="version_slider"
            )
        
        if new_version != current_version:
            SubmapAnalyzerState.select_version(new_version)
        
        submap_history = SubmapAnalyzerState.get_current_submap_history()
        
        # Return the selected version and submap history
        if submap_history and 0 <= new_version < len(submap_history):
            return new_version, submap_history
        
        return None 