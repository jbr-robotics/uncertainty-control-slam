import re
import numpy as np
import pandas as pd
from pathlib import Path
import streamlit as st
import plotly.express as px
import plotly.graph_objects as go

from cartographer_tuner.submap_analyzer.submap import Submap
from cartographer_tuner.submap_analyzer.gui.state import SubmapAnalyzerState
from cartographer_tuner.metrics.calculators.pgm.corner_count_calculator import CornerCountCalculator
from cartographer_tuner.metrics.calculators.pgm.enclosed_areas_calculator import EnclosedAreasCalculator
from cartographer_tuner.metrics.calculators.pgm.occupied_proportion_calculator import OccupiedProportionCalculator

# Helper functions
METRIC_CALCULATORS = {
    "corner_count": CornerCountCalculator,
    "enclosed_areas_count": EnclosedAreasCalculator,
    "occupied_proportion": OccupiedProportionCalculator,
}

STAT_FUNCS = {
    "mean": np.mean,
    "median": np.median,
    "std": np.std,
    "min": np.min,
    "max": np.max,
}

def get_metric_values(seq_dir, map_type, metric_type):
    values = []
    for f in sorted(seq_dir.glob("submap_*.pkl"), key=lambda f: int(re.search(r"submap_(\d+)", f.stem).group(1)) if re.search(r"submap_(\d+)", f.stem) else -1):
        try:
            submap = Submap.load(f)
            layer = getattr(submap, map_type)
            calculator = METRIC_CALCULATORS[metric_type](layer)
            val = calculator.calculate()[metric_type].value
            if not np.isnan(val):
                values.append(val)
        except Exception as e:
            st.warning(f"Failed to process {f.name} in {seq_dir.name}: {e}")
    return values

def get_range(min, max):
    padding = (max - min) * 0.05
    return [min - padding, max + padding]

class SubmapMultiSequenceAnalyzer:
    def render_single_metric_summary_input(self):
        col1, col2 = st.columns(2)
        with col1:
            self.map_type = st.selectbox("Map type", ["intensity", "map", "alpha"])
            self.min_filter = st.number_input("Min value filter", value=0.0, step=0.1)
        with col2:
            self.metric_type = st.selectbox("Metric", list(METRIC_CALCULATORS))
            self.max_filter = st.number_input("Max value filter", value=1000.0, step=0.1)
        self.stat_type = st.selectbox("Statistic to compute per sequence", list(STAT_FUNCS))

    def create_raw_metrics_dataframe(self, root_dir, map_type="intensity"):
        data = []
        sequence_dirs = sorted([d for d in root_dir.iterdir() if d.is_dir()])
        
        for seq_dir in sequence_dirs:
            try:
                param_value = int(seq_dir.name)
            except ValueError:
                param_value = None
                
            for f in sorted(seq_dir.glob("submap_*.pkl"), key=lambda f: int(re.search(r"submap_(\d+)", f.stem).group(1)) if re.search(r"submap_(\d+)", f.stem) else -1):
                try:
                    submap_id = int(re.search(r"submap_(\d+)", f.stem).group(1))
                    submap = Submap.load(f)
                    layer = getattr(submap, map_type)
                    
                    # Calculate corner count
                    corner_calculator = METRIC_CALCULATORS["corner_count"](layer)
                    corner_count = corner_calculator.calculate()["corner_count"].value
                    
                    # Calculate enclosed areas
                    areas_calculator = METRIC_CALCULATORS["enclosed_areas_count"](layer)
                    enclosed_areas = areas_calculator.calculate()["enclosed_areas_count"].value
                    
                    if not np.isnan(corner_count) and not np.isnan(enclosed_areas):
                        data.append({
                            "map_id": submap_id,
                            "param_value": param_value,
                            "corner_count": corner_count,
                            "enclosed_areas": enclosed_areas
                        })
                except Exception as e:
                    st.warning(f"Failed to process {f.name} in {seq_dir.name}: {e}")
        
        return pd.DataFrame(data)

    def render_single_metric_summary(self, root_dir):
        self.render_single_metric_summary_input()
        sequence_dirs = sorted([d for d in root_dir.iterdir() if d.is_dir()])
        if not sequence_dirs:
            st.warning("No sequence folders found.")
            return

        sequence_stats, sequence_names, all_values = [], [], []

        for seq_dir in sequence_dirs:
            values = get_metric_values(seq_dir, self.map_type, self.metric_type)
            if values:
                arr = np.array(values)
                value = STAT_FUNCS[self.stat_type](arr)
                if self.min_filter <= value <= self.max_filter:
                    sequence_stats.append(value)
                    sequence_names.append(seq_dir.name)
                    all_values.append(arr)

        if not sequence_stats:
            st.warning("No valid metrics found.")
            return

        st.markdown(f"### {self.stat_type.capitalize()} of `{self.metric_type}` per sequence")
        df_bar = pd.DataFrame({"Sequence": sequence_names, "Value": sequence_stats})
        fig_bar = px.bar(df_bar, x="Sequence", y="Value",
                        title=f"{self.metric_type.replace('_', ' ').capitalize()} ({self.stat_type}) per Sequence",
                        labels={"Value": f"{self.stat_type.capitalize()} {self.metric_type.replace('_', ' ')}"},
                        range_y=get_range(df_bar["Value"].min(), df_bar["Value"].max()))

        st.plotly_chart(fig_bar, use_container_width=True)

        # Boxplot of per-sequence stats
        st.markdown(f"### Distribution of {self.stat_type} across sequences (Boxplot)")
        fig_stat_box = go.Figure()
        fig_stat_box.add_trace(go.Box(x=sequence_stats, boxmean=True, orientation='h'))
        fig_stat_box.update_layout(
            xaxis_title=f"{self.stat_type.capitalize()} {self.metric_type.replace('_', ' ')}",
            title=f"Boxplot of {self.stat_type} across sequences"
        )
        st.plotly_chart(fig_stat_box, use_container_width=True)

        # Boxplot of per-submap metric values by sequence with histogram background
        st.markdown(f"### Boxplot of per-submap `{self.metric_type}` values by sequence")

        # Prepare histogram data (count of submaps per sequence)
        submap_counts = [len(arr) for arr in all_values]

        fig_box = go.Figure()

        # Add histogram bars (background)
        fig_box.add_trace(go.Bar(
            x=sequence_names,
            y=submap_counts,
            marker=dict(color='lightgray'),
            opacity=0.4,
            yaxis='y2',
            name='Submap Count'
        ))

        # Add boxplots on top
        for name, values in zip(sequence_names, all_values):
            fig_box.add_trace(go.Box(
                y=values,
                name=name,
                boxmean=True,
                marker_color='steelblue',
                line=dict(width=1)
            ))

        # Dual Y-axes
        fig_box.update_layout(
            title=f"{self.metric_type.replace('_', ' ').capitalize()} per Sequence",
            yaxis=dict(title=self.metric_type.replace('_', ' ').capitalize()),
            yaxis2=dict(overlaying='y', side='right', showgrid=False, visible=False),
            barmode="overlay"
        )

        st.plotly_chart(fig_box, use_container_width=True)

    def render(self):
        st.markdown("## Multi-Sequence Metric Summary")

        root_dir = SubmapAnalyzerState.get_working_path()
        if not isinstance(root_dir, Path) or not root_dir.is_dir():
            st.error("Working path must be a directory containing sequence folders.")
            return
        
        # Option to show raw metrics data
        if st.checkbox("Show raw metrics data"):
            st.write("Creating raw metrics dataframe...")
            map_type = st.selectbox("Map type for raw metrics", ["intensity", "map", "alpha"])
            df_raw = self.create_raw_metrics_dataframe(root_dir, map_type=map_type)
            st.dataframe(df_raw)
            
            # Option to download as CSV
            if not df_raw.empty:
                csv = df_raw.to_csv(index=False)
                st.download_button(
                    label="Download raw metrics as CSV",
                    data=csv,
                    file_name="submap_metrics_raw.csv",
                    mime="text/csv",
                )
        
        self.render_single_metric_summary(root_dir)
