import streamlit as st
import numpy as np
import threading
import time
from typing import Optional

from cartographer_tuner.submap_analyzer.submap_storage import SubmapStorage
from cartographer_tuner.submap_analyzer.gui.state import SubmapAnalyzerState
from cartographer_tuner.submap_analyzer.gui.components import SubmapListComponent, ControlPanelComponent, VersionSliderComponent, MetricsDisplayComponent, SubmapsSummaryComponent
from cartographer_tuner.submap_analyzer.gui.visualizer import display_bitmap

class SubmapAnalyzerApp:
    
    def __init__(self, storage: SubmapStorage):
        self.storage = storage
        
        self.submap_list = SubmapListComponent(storage)
        self.control_panel = ControlPanelComponent(storage)
        self.version_slider = VersionSliderComponent()
        self.metrics_display = MetricsDisplayComponent()
        self.submaps_summary = SubmapsSummaryComponent(storage)
    def run(self):
        SubmapAnalyzerState.initialize()
        

        st.title("Cartographer Submap Analyzer")
        tab_summary, tab_individual = st.tabs(["Summary", "Individual"])

        with tab_summary:
            self._render_summary()

        with tab_individual:
            self._render_individual()

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


def run_streamlit_app(storage: SubmapStorage):
    assert storage is not None, "No submap storage provided. Cannot display submaps."
    try:
        app = SubmapAnalyzerApp(storage)
        app.run()
    except Exception as e:
        st.error(f"Error running the app: {e}")
        st.exception(e)