import re
import numpy as np
import pandas as pd
import streamlit as st
import plotly.express as px
import plotly.graph_objects as go
from pathlib import Path

from cartographer_tuner.submap_analyzer.submap import Submap
from cartographer_tuner.submap_analyzer.gui.state import SubmapAnalyzerState
from cartographer_tuner.metrics.calculators.pgm import (
    CornerCountCalculator,
    EnclosedAreasCalculator,
    OccupiedProportionCalculator,
    UnsureAreaProportionCalculator
)

class SubmapSequenceAnalyzer:
    def __init__(self):
        self.calculators = {
            "corner_count": CornerCountCalculator,
            "enclosed_areas_count": EnclosedAreasCalculator,
            "occupied_proportion": OccupiedProportionCalculator,
            "unsure_area_proportion": UnsureAreaProportionCalculator
        }

    def calculate_metric(self, submap, map_type, metric_type):
        layer = getattr(submap, map_type)
        return self.calculators[metric_type](layer).calculate()[metric_type].value

    def render(self):
        st.markdown("## Submap Sequence Analyzer")

        sequence_dir = SubmapAnalyzerState.get_working_path()
        if not isinstance(sequence_dir, Path) or not sequence_dir.is_dir():
            st.error("Working path must be a valid directory.")
            return

        map_type = st.selectbox("Select map type", ["map", "alpha", "intensity"])
        metric_type = st.selectbox("Select metric", list(self.calculators.keys()))

        submap_files = sorted(sequence_dir.glob("submap_*.pkl"),
                              key=lambda f: int(re.search(r"submap_(\d+)", f.stem).group(1)))
        if not submap_files:
            st.warning("No submap_*.pkl files found in the directory.")
            return

        values, paths = [], []

        for f in submap_files:
            try:
                submap = Submap.load(f)
                val = self.calculate_metric(submap, map_type, metric_type)
                values.append(val if not np.isnan(val) else np.nan)
                paths.append(str(f))
            except Exception as e:
                st.warning(f"Failed to process {f.name}: {e}")
                values.append(np.nan)
                paths.append(str(f))

        # --- Interactive Line Plot ---
        st.markdown(f"### Metric: `{metric_type}` over time (Interactive)")
        df = pd.DataFrame({"Index": range(len(values)), "Value": values, "Path": paths})
        fig = px.line(df, x="Index", y="Value", hover_name="Path",
                      title=f"{metric_type.replace('_', ' ').capitalize()} over time",
                      labels={"Value": metric_type.replace("_", " ").capitalize()},
                      markers=True)
        st.plotly_chart(fig, use_container_width=True)

        # --- Stats ---
        valid_values = [v for v in values if not np.isnan(v)]
        if valid_values:
            stats = {
                "Count": len(valid_values),
                "Mean": np.mean(valid_values),
                "Median": np.median(valid_values),
                "Std Dev": np.std(valid_values),
                "Min": np.min(valid_values),
                "Max": np.max(valid_values),
                "NaNs": len(values) - len(valid_values)
            }
            st.markdown("### Summary Statistics")
            st.table(pd.DataFrame(stats, index=["Value"]).T)
        else:
            st.warning("No valid data to compute statistics.")

        # --- Metric Comparison Section ---
        st.markdown("---")
        st.markdown("## Compare Two Metrics (Binned Boxplot + Histogram)")

        col1, col2 = st.columns(2)
        with col1:
            map_type_1 = st.selectbox("Map type 1", ["map", "alpha", "intensity"], key="map1")
            metric_type_1 = st.selectbox("Metric 1", list(self.calculators), key="metric1")
        with col2:
            map_type_2 = st.selectbox("Map type 2", ["map", "alpha", "intensity"], key="map2")
            metric_type_2 = st.selectbox("Metric 2", list(self.calculators), key="metric2")

        bins = st.number_input("Number of bins", min_value=2, max_value=100, value=10)

        if (map_type_1 == map_type_2 and metric_type_1 == metric_type_2):
            st.warning("Please select two different map+metric combinations to compare.")
            return

        m1_vals, m2_vals = [], []

        for f in submap_files:
            try:
                submap = Submap.load(f)
                val1 = self.calculate_metric(submap, map_type_1, metric_type_1)
                val2 = self.calculate_metric(submap, map_type_2, metric_type_2)
                if not (np.isnan(val1) or np.isnan(val2)):
                    m1_vals.append(val1)
                    m2_vals.append(val2)
            except Exception as e:
                st.warning(f"Error loading {f.name}: {e}")

        if m1_vals:
            m1 = np.array(m1_vals)
            m2 = np.array(m2_vals)

            bin_edges = np.histogram_bin_edges(m1, bins=bins)
            bin_counts = np.zeros(len(bin_edges) - 1)
            binned_values = []
            binned_labels = []

            for i in range(1, len(bin_edges)):
                in_bin = (m1 >= bin_edges[i-1]) & (
                    m1 < bin_edges[i] if i < len(bin_edges) - 1 else m1 <= bin_edges[i]
                )
                values_in_bin = m2[in_bin]
                bin_counts[i-1] = np.sum(in_bin)
                if values_in_bin.size > 0:
                    binned_values.append(values_in_bin)
                    binned_labels.append(f"[{bin_edges[i-1]:.2f} – {bin_edges[i]:.2f}]")

            if binned_values:
                st.markdown(f"### Boxplot: `{metric_type_2}` in bins of `{metric_type_1}`")

                fig = go.Figure()

                # Gray histogram bars
                fig.add_trace(go.Bar(
                    x=binned_labels,
                    y=bin_counts,
                    marker=dict(color="lightgray"),
                    opacity=0.4,
                    yaxis="y2",
                    name="Count"
                ))

                # Boxplot per bin
                for label, values in zip(binned_labels, binned_values):
                    fig.add_trace(go.Box(
                        y=values,
                        name=label,
                        boxmean=True,
                        marker_color='steelblue',
                        line=dict(width=1)
                    ))

                fig.update_layout(
                    title=f"{metric_type_2.replace('_', ' ').capitalize()} vs. {metric_type_1.replace('_', ' ').capitalize()}",
                    xaxis_title=f"{metric_type_1.replace('_', ' ').capitalize()} bins",
                    yaxis=dict(title=metric_type_2.replace('_', ' ').capitalize()),
                    yaxis2=dict(overlaying="y", side="right", showgrid=False, visible=False),
                    barmode="overlay",
                    bargap=0.3
                )

                st.plotly_chart(fig, use_container_width=True)
            else:
                st.warning("No data available for boxplot comparison.")
        else:
            st.warning("Not enough valid data for comparison.")