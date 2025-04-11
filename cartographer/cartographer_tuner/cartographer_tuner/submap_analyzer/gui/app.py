import streamlit as st
import numpy as np
import threading
import time
from typing import Optional
from pathlib import Path

from cartographer_tuner.submap_analyzer.gui.state import SubmapAnalyzerState
from cartographer_tuner.submap_analyzer.gui.visualizer import display_bitmap
from cartographer_tuner.submap_analyzer.gui.components.individual_submap_analyzer import IndividualSubmapAnalyzer
from cartographer_tuner.submap_analyzer.gui.components.submap_sequence_analyzer import SubmapSequenceAnalyzer
from cartographer_tuner.submap_analyzer.gui.components.submap_multi_sequence_analyzer import SubmapMultiSequenceAnalyzer

class SubmapAnalyzerApp:
    
    def __init__(self):
        self.individual_submap_analyzer = IndividualSubmapAnalyzer()
        self.submap_sequence_analyzer = SubmapSequenceAnalyzer()
        self.submap_multi_sequence_analyzer = SubmapMultiSequenceAnalyzer()

    def run(self):
        SubmapAnalyzerState.initialize()

        self._submap_path = Path(st.text_input("Enter folder path"))

        SubmapAnalyzerState.set_working_path(self._submap_path)

        if self._submap_path is None or not self._submap_path:
            st.error("No submap path selected. Please select a submap path from the sidebar.")
        elif SubmapAnalyzerApp._is_individual_submap(self._submap_path):
            self.individual_submap_analyzer.render()
        elif SubmapAnalyzerApp._is_submap_sequence(self._submap_path):
            self.submap_sequence_analyzer.render()
        elif SubmapAnalyzerApp._is_sequence_of_sequences(self._submap_path):
            self.submap_multi_sequence_analyzer.render()
        elif not self._submap_path.is_dir():
            st.error("Please select a valid folder path.")
        else:
            st.error("Something went wrong. Please try again.")

    @staticmethod
    def _is_individual_submap(submap_path: Path) -> bool:
        return submap_path.suffix == ".pkl"

    @staticmethod
    def _is_submap_sequence(submap_path: Path) -> bool:
        return submap_path.is_dir() and all(f.name.startswith("submap_") and f.name.endswith(".pkl") and f.is_file() for f in submap_path.iterdir())

    @staticmethod
    def _is_sequence_of_sequences(submap_path: Path) -> bool:
        return submap_path.is_dir() and all(SubmapAnalyzerApp._is_submap_sequence(f) for f in submap_path.iterdir())

    def _render_summary(self):
        self.submaps_summary.render()
    
    def _render_individual(self):
            self.control_panel.render()
            st.sidebar.markdown("---")
            self.submap_list.render()
            self._render_main_content()

    def _render_main_content(self):
        slider_result = self.version_slider.render()
        if not slider_result:
            return
        # Add a checkbox for normalization control
        normalize_images = st.checkbox(
            "Normalize Images", 
            value=True, 
            help="When checked, images will be normalized to improve visibility"
        )
        col1, col2 = st.columns(2)

        version_index, submap_history = slider_result
        
        selected_submap = SubmapAnalyzerState.get_selected_submap()
        if not selected_submap:
            return
        
        if 0 <= version_index < len(submap_history):
            submap_data = submap_history[version_index]
            intencity = submap_data[0]
            alpha = submap_data[1]
            trajectory_id, submap_index = selected_submap

            for col, (title, data) in zip([col1, col2], [("Intensity", intencity), ("Alpha", alpha)]):
                with col:
                    title = f"{title} ({trajectory_id}, {submap_index}) - Version {version_index}"
                    display_bitmap(data, title, normalize=normalize_images)
        
        self.metrics_display.render()


def run_streamlit_app():
    try:
        app = SubmapAnalyzerApp()
        app.run()
    except Exception as e:
        st.error(f"Error running the app: {e}")
        st.exception(e)