import streamlit as st
from pathlib import Path
from cartographer_tuner.submap_analyzer.gui.state import SubmapAnalyzerState
from cartographer_tuner.submap_analyzer.submap import Submap
from cartographer_tuner.submap_analyzer.gui.visualizer import display_bitmap
from cartographer_tuner.metrics.calculators.pgm import CornerCountCalculator, EnclosedAreasCalculator, OccupiedProportionCalculator
import cv2

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

    def analyze_layer(self, name: str, data):
        st.markdown(f"**{name}**")
        display_bitmap(data, name, normalize=True, inverse=False)

        # --- Corner Count ---
        corner_calc = CornerCountCalculator(map_data=data, debug=False)
        corner_metrics = corner_calc.calculate()
        st.write(f"Corner count: {corner_metrics['corner_count'].value}")
        st.image(cv2.cvtColor(corner_calc.debug_image(), cv2.COLOR_BGR2RGB), caption="Corners", channels="RGB")

        # --- Enclosed Areas ---
        enclosed_calc = EnclosedAreasCalculator(map_data=data, debug=False)
        enclosed_metrics = enclosed_calc.calculate()
        st.write(f"Enclosed areas: {enclosed_metrics['enclosed_areas_count'].value}")
        st.image(cv2.cvtColor(enclosed_calc.debug_image(), cv2.COLOR_BGR2RGB), caption="Enclosed Areas", channels="RGB")

        # --- Occupied Proportion ---
        occupied_calc = OccupiedProportionCalculator(map_data=data, debug=False)
        occupied_metrics = occupied_calc.calculate()
        st.write(f"Occupied proportion: {occupied_metrics['occupied_proportion'].value:.3f}")
        st.image(cv2.cvtColor(occupied_calc.debug_image(), cv2.COLOR_BGR2RGB), caption="Occupied", channels="RGB")