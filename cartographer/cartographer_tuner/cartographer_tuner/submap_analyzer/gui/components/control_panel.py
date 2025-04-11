import streamlit as st
from collections import defaultdict
from cartographer_tuner.submap_analyzer.gui.state import SubmapAnalyzerState

from cartographer_tuner.metrics.calculators.pgm import CornerCountCalculator, EnclosedAreasCalculator, OccupiedProportionCalculator

# class ControlPanelComponent:
#     """Component for control panel with buttons and actions"""
    
#     def __init__(self, storage):
#         self.storage = storage
    
#     def render(self):
#         """Render the control panel component"""
#         st.sidebar.markdown("## Controls")
        
#         # Add update button for refreshing the current submap data
#         selected_submap = SubmapAnalyzerState.get_selected_submap()
#         update_button_disabled = selected_submap is None
        
#         if st.sidebar.button("🔄 Update Submap Data", disabled=update_button_disabled):
            
#             if selected_submap:
#                 updating_placeholder = st.sidebar.info("Updating submap data...")
#                 trajectory_id, submap_index = selected_submap
                
#                 submap_history = self.storage.get_submap_history(trajectory_id, submap_index)
                
#                 SubmapAnalyzerState.set_current_submap_history(submap_history)
#                 if submap_history:
#                     SubmapAnalyzerState.set_version_count(len(submap_history))
#                     SubmapAnalyzerState.mark_data_current()
#                     st.sidebar.success(f"Submap data updated - {len(submap_history)} versions available")
#                 else:
#                     st.sidebar.warning(f"No data available for submap ({trajectory_id}, {submap_index})")
#                 updating_placeholder.empty()
#                 calculating_metrics_placeholder = st.sidebar.info("Calculating metrics...")
#                 self._calculate_metrics()
#                 calculating_metrics_placeholder.empty()
#             else:
#                 st.sidebar.warning("No submap selected")

#     def _calculate_metrics(self):
#         """Calculate metrics for the current submap"""
#         selected_submap = SubmapAnalyzerState.get_selected_submap()
#         if not selected_submap:
#             st.warning("No submap selected. Please select a submap from the sidebar.")
#             return
        
#         submap_history = SubmapAnalyzerState.get_current_submap_history()
#         if not submap_history:
#             st.warning("No submap history available. Please select a submap from the sidebar.")
#             return
        
#         intensity_metrics = defaultdict(list)
#         alpha_metrics = defaultdict(list)
#         for data in submap_history:
#             intensity_data = data[0]
#             alpha_data = data[1]
#             for metric in [CornerCountCalculator, EnclosedAreasCalculator, OccupiedProportionCalculator]:
#                 i_metrics = metric(intensity_data).calculate()
#                 a_metrics = metric(alpha_data).calculate() 
#                 for key, value in i_metrics.items():    
#                     intensity_metrics[key].append(value.value)
#                 for key, value in a_metrics.items():
#                     alpha_metrics[key].append(value.value)
            
#         SubmapAnalyzerState.set_intensity_metrics(intensity_metrics)
#         SubmapAnalyzerState.set_alpha_metrics(alpha_metrics)
            
        
        
