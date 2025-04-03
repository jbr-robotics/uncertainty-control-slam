#!/usr/bin/env python3

import streamlit as st
import numpy as np
from typing import List, Tuple, Callable, Optional

from ..submap_storage import SubmapStorage
from .state import SubmapAnalyzerState

class SubmapListComponent:
    """Component for displaying and selecting from available submaps"""
    
    def __init__(self, storage: SubmapStorage):
        self.storage = storage
    
    def render(self):
        """Render the submap list component"""
        st.sidebar.markdown("## Submap List")
        
        # Toggle button for hiding/showing the list
        show_hide_text = "Hide List" if SubmapAnalyzerState.should_show_submap_list() else "Show List"
        if st.sidebar.button(show_hide_text):
            SubmapAnalyzerState.toggle_submap_list()
        
        # Update button
        if st.sidebar.button("🔄 Refresh List"):
            submaps = self.storage.get_all_submap_keys()
            SubmapAnalyzerState.update_available_submaps(submaps)
            st.sidebar.success(f"Found {len(submaps)} submaps")
        
        # Only show the list if not hidden
        if SubmapAnalyzerState.should_show_submap_list():
            available_submaps = SubmapAnalyzerState.get_available_submaps()
            selected_submap = SubmapAnalyzerState.get_selected_submap()
            
            if not available_submaps:
                st.sidebar.info("No submaps available. Click 'Refresh List' to update.")
                return
            
            st.sidebar.markdown("### Available Submaps")
            
            # Group submaps by trajectory ID for better organization
            trajectories = {}
            for traj_id, submap_idx in available_submaps:
                if traj_id not in trajectories:
                    trajectories[traj_id] = []
                trajectories[traj_id].append(submap_idx)
            
            # Display submaps organized by trajectory
            for traj_id in sorted(trajectories.keys()):
                st.sidebar.markdown(f"**Trajectory {traj_id}**")
                
                # Sort submap indices
                submap_indices = sorted(trajectories[traj_id])
                
                # Create a selection grid
                cols = st.sidebar.columns(3)
                for i, submap_idx in enumerate(submap_indices):
                    col_idx = i % 3
                    is_selected = selected_submap == (traj_id, submap_idx)
                    button_label = f"{submap_idx}"
                    button_type = "primary" if is_selected else "secondary"
                    
                    if cols[col_idx].button(button_label, key=f"submap_{traj_id}_{submap_idx}", 
                                          type=button_type):
                        # Just select the submap without loading data
                        SubmapAnalyzerState.select_submap(traj_id, submap_idx)
                        # Reset the version display to indicate data needs refresh
                        SubmapAnalyzerState.set_version_count(0)
                        SubmapAnalyzerState.mark_data_outdated()

class ControlPanelComponent:
    """Component for control panel with buttons and actions"""
    
    def __init__(self, storage: SubmapStorage):
        self.storage = storage
    
    def render(self):
        """Render the control panel component"""
        st.sidebar.markdown("## Controls")
        
        # Add update button for refreshing the current submap data
        selected_submap = SubmapAnalyzerState.get_selected_submap()
        update_button_disabled = selected_submap is None
        
        if st.sidebar.button("🔄 Update Submap Data", disabled=update_button_disabled):
            st.sidebar.info("Updating submap data...")
            
            if selected_submap:
                trajectory_id, submap_index = selected_submap
                
                # Force a refresh by getting the latest map list
                latest_list = self.storage.get_latest_submap_list()
                
                # Get the submap history after update
                submap_history = self.storage.get_submap_history(trajectory_id, submap_index)
                
                if submap_history:
                    # Update version count in state
                    SubmapAnalyzerState.set_version_count(len(submap_history))
                    SubmapAnalyzerState.mark_data_current()
                    st.sidebar.success(f"Submap data updated - {len(submap_history)} versions available")
                else:
                    st.sidebar.warning(f"No data available for submap ({trajectory_id}, {submap_index})")
            else:
                st.sidebar.warning("No submap selected")

class VersionSliderComponent:
    """Component for selecting the version of a submap using a slider"""
    
    def __init__(self, storage: SubmapStorage):
        self.storage = storage
    
    def render(self):
        """Render the version slider component"""
        selected_submap = SubmapAnalyzerState.get_selected_submap()
        
        if not selected_submap:
            st.warning("No submap selected. Please select a submap from the sidebar.")
            return
        
        trajectory_id, submap_index = selected_submap
        
        # Display submap identifier
        st.markdown(f"## Submap ({trajectory_id}, {submap_index})")
        
        # Check if data is outdated or needs refresh
        if SubmapAnalyzerState.is_data_outdated():
            st.warning("⚠️ Click 'Update Submap Data' in the sidebar to load data for this submap")
            return None
        
        # Get version count from state (populated by Update button)
        version_count = SubmapAnalyzerState.get_version_count()
        
        if version_count <= 0:
            st.warning("No data available. Click 'Update Submap Data' to load versions.")
            return None
        
        st.markdown(f"### Version History: {version_count} versions")
        
        # Get current selected version
        current_version = SubmapAnalyzerState.get_selected_version()
        
        # Ensure current version is valid
        current_version = min(current_version, version_count - 1)
        
        # If there's only one version, display text instead of a slider
        if version_count <= 1:
            st.text("Only one version available (version 0)")
            new_version = 0
        else:
            # Add the slider (doesn't trigger data loading)
            new_version = st.slider(
                "Select Version", 
                min_value=0, 
                max_value=version_count-1,
                value=current_version,
                key="version_slider"
            )
        
        # Update selected version in state if changed
        if new_version != current_version:
            SubmapAnalyzerState.select_version(new_version)
        
        # Get the submap history only for display purposes
        submap_history = self.storage.get_submap_history(trajectory_id, submap_index)
        
        # Return the selected version and submap history
        if submap_history and 0 <= new_version < len(submap_history):
            return new_version, submap_history
        
        return None 