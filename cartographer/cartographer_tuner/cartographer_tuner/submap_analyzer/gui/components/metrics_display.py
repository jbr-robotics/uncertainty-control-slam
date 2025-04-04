import streamlit as st
import matplotlib.pyplot as plt

from cartographer_tuner.submap_analyzer.gui.state import SubmapAnalyzerState


class MetricsDisplayComponent:
    def __init__(self):
        pass
    
    def render(self):
        selected_submap = SubmapAnalyzerState.get_selected_submap()

        if not selected_submap:
            st.warning("No submap selected. Please select a submap from the sidebar.")
            return
        
        trajectory_id, submap_index = selected_submap

        submap_history = SubmapAnalyzerState.get_current_submap_history()

        if submap_history is None:
            st.warning("No submap history available. Please select a submap from the sidebar.")
            return
        
        version_count = SubmapAnalyzerState.get_version_count()
        
        if version_count <= 0:
            st.warning("No versions available for this submap.")
            return
        
        st.markdown("## Submap Metrics")
        
        col_intensity, col_alpha = st.columns(2)

        def plot_metric(metrics: dict):
            for metric_name, metric_values in metrics.items():
                st.subheader(metric_name)
                fig, ax = plt.subplots()
                versions = list(range(version_count))
                ax.plot(versions, metric_values, label=metric_name)
                ax.set_xlabel('Version')
                ax.set_ylabel('Value')
                ax.legend()
                ax.grid(True)
                st.pyplot(fig)
        
        with col_intensity:
            st.markdown("### Intensity Channel Metrics")
            plot_metric(SubmapAnalyzerState.get_intensity_metrics())
        
        with col_alpha:
            st.markdown("### Alpha Channel Metrics")
            plot_metric(SubmapAnalyzerState.get_alpha_metrics())
                    
