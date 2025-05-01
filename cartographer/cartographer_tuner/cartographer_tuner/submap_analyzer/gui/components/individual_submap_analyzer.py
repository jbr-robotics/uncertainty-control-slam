import streamlit as st
from pathlib import Path
import numpy as np
import plotly.express as px
import plotly.graph_objects as go
import cv2

from cartographer_tuner.submap_analyzer.gui.state import SubmapAnalyzerState
from cartographer_tuner.submap_analyzer.submap import Submap
from cartographer_tuner.metrics.calculators.pgm import (
    CornerCountCalculator,
    EnclosedAreasCalculator,
    OccupiedProportionCalculator,
    UnsureAreaProportionCalculator
)

class IndividualSubmapAnalyzer:
    def __init__(self):
        pass

    def render(self):
        st.markdown("## Individual submap analyzer")

        submap_path = SubmapAnalyzerState.get_working_path()
        if not isinstance(submap_path, Path): 
            st.error(f"{submap_path} of type {type(submap_path)} is not a Path")
            return

        try: 
            submap = Submap.load(submap_path)
        except Exception as e:
            st.error(e)
            return

        st.markdown("### Submap Layers and Metrics")

        col1, col2, col3 = st.columns(3)

        with col1:
            self.analyze_layer("Alpha", submap.alpha)

        with col2:
            self.analyze_layer("Intensity", submap.intensity)

        with col3:
            self.analyze_layer("Map", submap.map)

    def analyze_layer(self, name: str, data: np.ndarray):
        st.markdown(f"**{name}**")
        self.display_plotly_image(data, title=f"{name} Bitmap")

        # --- Corner Count ---
        corner_calc = CornerCountCalculator(map_data=data, debug=False)
        corner_metrics = corner_calc.calculate()
        st.write(f"Corner count: {corner_metrics['corner_count'].value}")
        self.display_debug_image(corner_calc.debug_image(), f"{name} Corners")

        # --- Enclosed Areas ---
        enclosed_calc = EnclosedAreasCalculator(map_data=data, debug=False)
        enclosed_metrics = enclosed_calc.calculate()
        st.write(f"Enclosed areas: {enclosed_metrics['enclosed_areas_count'].value}")
        self.display_debug_image(enclosed_calc.debug_image(), f"{name} Enclosed Areas")

        # --- Occupied Proportion ---
        occupied_calc = OccupiedProportionCalculator(map_data=data, debug=False)
        occupied_metrics = occupied_calc.calculate()
        st.write(f"Occupied proportion: {occupied_metrics['occupied_proportion'].value:.3f}")
        self.display_debug_image(occupied_calc.debug_image(), f"{name} Occupied Proportion")

        # --- Unsure Area Proportion ---
        unsure_calc = UnsureAreaProportionCalculator(map_data=data, debug=False)
        unsure_metrics = unsure_calc.calculate()
        st.write(f"Unsure area proportion: {unsure_metrics['unsure_area_proportion'].value:.3f}")
        self.display_debug_image(unsure_calc.debug_image(), f"{name} Unsure Area Proportion")

    def display_plotly_image(self, image: np.ndarray, title=""):
        # Normalize image for better contrast if needed
        norm_img = image.astype(np.float32)
        norm_img -= norm_img.min()
        if norm_img.max() > 0:
            norm_img /= norm_img.max()

        fig = px.imshow(norm_img, color_continuous_scale="gray", origin="lower")
        fig.update_layout(title=title, coloraxis_showscale=False, margin=dict(l=0, r=0, t=30, b=0))
        st.plotly_chart(fig, use_container_width=True)

    def display_debug_image(self, img_rgb: np.ndarray, title: str):
        fig = px.imshow(img_rgb, title=title, origin="lower")
        fig.update_layout(margin=dict(l=0, r=0, t=30, b=0))
        chart_key = f"plotly_chart_{title.lower().replace(' ', '_')}" 
        st.plotly_chart(fig, use_container_width=True, key=chart_key) 
