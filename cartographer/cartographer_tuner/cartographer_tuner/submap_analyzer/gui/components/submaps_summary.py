from cartographer_tuner.submap_analyzer.gui.state import SubmapAnalyzerState
from cartographer_tuner.submap_analyzer.submap_storage import SubmapStorage
import streamlit as st
import matplotlib.pyplot as plt
import numpy as np
from collections import defaultdict
from cartographer_tuner.metrics.calculators.pgm import CornerCountCalculator, EnclosedAreasCalculator, OccupiedProportionCalculator

class SubmapsSummaryComponent:
    def __init__(self, storage: SubmapStorage):
        self.storage = storage
    
    def render(self):
        
        available_submaps = self.storage.get_all_submap_keys()
        
        if not available_submaps:
            st.warning("No submaps available. Please refresh the submap list.")
            return
        
        st.markdown("## Submaps Summary")
        st.markdown("This view shows metrics distribution across all submaps (using their latest versions).")
        
        intensity_metrics = defaultdict(list)
        alpha_metrics = defaultdict(list)
        for trajectory_id, submap_index in available_submaps:
            submap_data = self.storage.get_latest_submap(trajectory_id, submap_index)
            
            intensity_data = submap_data[0]
            alpha_data = submap_data[1]
            for metric in [CornerCountCalculator, EnclosedAreasCalculator, OccupiedProportionCalculator]:
                i_metrics = metric(intensity_data).calculate()
                a_metrics = metric(alpha_data).calculate() 
                for key, value in i_metrics.items():    
                    intensity_metrics[key].append(value.value)
                for key, value in a_metrics.items():
                    alpha_metrics[key].append(value.value)
        
        def plot_metric(metrics: dict):
            for metric_name, metric_values in metrics.items():
                st.subheader(metric_name)
                st.markdown(f"**Mean**: {np.mean(metric_values)}")
                st.markdown(f"**Std Dev**: {np.std(metric_values)}")
                st.markdown(f"**Min**: {np.min(metric_values)}")
                st.markdown(f"**25%**: {np.percentile(metric_values, 25)}")
                st.markdown(f"**Median**: {np.median(metric_values)}")
                st.markdown(f"**75%**: {np.percentile(metric_values, 75)}")
                st.markdown(f"**Max**: {np.max(metric_values)}")
                fig, ax = plt.subplots()
                ax.hist(metric_values, bins=100)
                st.pyplot(fig)
        
        col1, col2 = st.columns(2)
        with col1:
            st.subheader("Intensity Metrics")
            plot_metric(intensity_metrics)
        with col2:
            st.subheader("Alpha Metrics")
            plot_metric(alpha_metrics)
