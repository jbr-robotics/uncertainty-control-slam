import streamlit as st

from cartographer_tuner.submap_analyzer.submap_storage import SubmapStorage
from cartographer_tuner.submap_analyzer.gui.state import SubmapAnalyzerState

class SubmapListComponent:
    def __init__(self, storage: SubmapStorage):
        self.storage = storage
    
    def render(self):
        st.sidebar.markdown("## Submap List")
        
        if st.sidebar.button("🔄 Refresh List"):
            submaps = self.storage.get_all_submap_keys()
            SubmapAnalyzerState.update_available_submaps(submaps)
            st.sidebar.success(f"Found {len(submaps)} submaps")
        
        if SubmapAnalyzerState.should_show_submap_list():
            available_submaps = SubmapAnalyzerState.get_available_submaps()
            selected_submap = SubmapAnalyzerState.get_selected_submap()
            
            if not available_submaps:
                st.sidebar.info("No submaps available. Click 'Refresh List' to update.")
                return
            
            st.sidebar.markdown("### Available Submaps")
            
            trajectories = {}
            for traj_id, submap_idx in available_submaps:
                if traj_id not in trajectories:
                    trajectories[traj_id] = []
                trajectories[traj_id].append(submap_idx)
            
            for traj_id in sorted(trajectories.keys()):
                st.sidebar.markdown(f"**Trajectory {traj_id}**")
                
                submap_indices = sorted(trajectories[traj_id])
                
                cols = st.sidebar.columns(3)
                for i, submap_idx in enumerate(submap_indices):
                    col_idx = i % 3
                    is_selected = selected_submap == (traj_id, submap_idx)
                    button_label = f"{submap_idx}"
                    button_type = "primary" if is_selected else "secondary"
                    
                    if cols[col_idx].button(button_label, key=f"submap_{traj_id}_{submap_idx}", 
                                          type=button_type):
                        SubmapAnalyzerState.select_submap(traj_id, submap_idx)
                        SubmapAnalyzerState.set_version_count(0)
                        SubmapAnalyzerState.mark_data_outdated()