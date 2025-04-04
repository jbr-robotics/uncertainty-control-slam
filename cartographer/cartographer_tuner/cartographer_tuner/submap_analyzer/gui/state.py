#!/usr/bin/env python3

import streamlit as st
from typing import Dict, Tuple, Optional, List
import numpy as np
from dataclasses import dataclass

class SubmapAnalyzerState:
    """
    Class to manage the state of the Submap Analyzer GUI.
    Uses Streamlit's session_state to persist data between reruns.
    """
    @staticmethod
    def initialize():
        """Initialize the application state if not already done"""
        if 'initialized' not in st.session_state:
            st.session_state.selected_submap = None  # (trajectory_id, submap_index)
            st.session_state.selected_version = 0    # Index into the history array
            st.session_state.show_submap_list = True # Whether to display the submap list
            st.session_state.available_submaps = []  # List of submap keys
            st.session_state.version_count = 0       # Number of versions available for the current submap
            st.session_state.data_outdated = True    # Whether the data needs refreshing
            st.session_state.initialized = True
            st.session_state.current_submap_history = []
            st.session_state.intensity_metrics = {}
            st.session_state.alpha_metrics = {}
    
    @staticmethod
    def select_submap(trajectory_id: int, submap_index: int):
        """Select a submap for visualization"""
        st.session_state.selected_submap = (trajectory_id, submap_index)
        st.session_state.selected_version = 0  # Reset to the first version
    
    @staticmethod
    def select_version(version_index: int):
        """Select a version of the current submap"""
        st.session_state.selected_version = version_index
    
    @staticmethod
    def toggle_submap_list():
        """Toggle the visibility of the submap list"""
        st.session_state.show_submap_list = not st.session_state.show_submap_list
    
    @staticmethod
    def update_available_submaps(submaps: List[Tuple[int, int]]):
        """Update the list of available submaps"""
        st.session_state.available_submaps = submaps
        
        # If no submap is selected and we have submaps, select the first one
        if st.session_state.selected_submap is None and submaps:
            st.session_state.selected_submap = submaps[0]
            st.session_state.data_outdated = True
    
    @staticmethod
    def get_selected_submap() -> Optional[Tuple[int, int]]:
        """Get the currently selected submap"""
        return st.session_state.selected_submap
    
    @staticmethod
    def get_selected_version() -> int:
        """Get the currently selected version index"""
        return st.session_state.selected_version
    
    @staticmethod
    def get_available_submaps() -> List[Tuple[int, int]]:
        """Get the list of available submaps"""
        return st.session_state.available_submaps
    
    @staticmethod
    def should_show_submap_list() -> bool:
        """Check if the submap list should be shown"""
        return st.session_state.show_submap_list
    
    @staticmethod
    def set_version_count(count: int):
        """Set the number of versions available for the current submap"""
        st.session_state.version_count = count
    
    @staticmethod
    def get_version_count() -> int:
        """Get the number of versions available for the current submap"""
        return st.session_state.version_count
    
    @staticmethod
    def mark_data_outdated():
        """Mark the data as needing refresh"""
        st.session_state.data_outdated = True
    
    @staticmethod
    def mark_data_current():
        """Mark the data as up-to-date"""
        st.session_state.data_outdated = False
    
    @staticmethod
    def is_data_outdated() -> bool:
        """Check if the data needs refreshing"""
        return st.session_state.data_outdated 
    
    @staticmethod
    def get_current_submap_history() -> List[Tuple[np.ndarray, np.ndarray]]:
        """Get the current submap history"""
        return st.session_state.current_submap_history
    
    
    @staticmethod
    def set_current_submap_history(history: List[Tuple[np.ndarray, np.ndarray]]):
        """Set the current submap history"""
        st.session_state.current_submap_history = history

    @staticmethod
    def get_intensity_metrics() -> Dict[str, Dict[str, float]]:
        """Get the intensity metrics for the current submap"""
        return st.session_state.intensity_metrics
    
    @staticmethod
    def get_alpha_metrics() -> Dict[str, Dict[str, float]]:
        """Get the alpha metrics for the current submap"""
        return st.session_state.alpha_metrics
    
    @staticmethod
    def set_intensity_metrics(metrics: Dict[str, Dict[str, float]]):
        """Set the intensity metrics for the current submap"""
        st.session_state.intensity_metrics = metrics
    
    @staticmethod
    def set_alpha_metrics(metrics: Dict[str, Dict[str, float]]):
        """Set the alpha metrics for the current submap"""
        st.session_state.alpha_metrics = metrics
