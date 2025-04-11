import streamlit as st
import os
import re
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

from cartographer_tuner.submap_analyzer.submap import Submap
from cartographer_tuner.submap_analyzer.gui.state import SubmapAnalyzerState

from cartographer_tuner.metrics.calculators.pgm.corner_count_calculator import CornerCountCalculator
from cartographer_tuner.metrics.calculators.pgm.enclosed_areas_calculator import EnclosedAreasCalculator
from cartographer_tuner.metrics.calculators.pgm.occupied_proportion_calculator import OccupiedProportionCalculator


class SubmapSequenceAnalyzer:
    def __init__(self):
        pass

    def render(self):
        st.markdown("## Submap Sequence Analyzer")

        sequence_dir = SubmapAnalyzerState.get_working_path()
        if not isinstance(sequence_dir, Path):
            st.error("Working path must be a directory.")
            return

        if not sequence_dir.is_dir():
            st.error(f"{sequence_dir} is not a directory.")
            return

        # --- Select map type and metric ---
        map_type = st.selectbox("Select map type", ["map", "alpha", "intensity"])
        metric_type = st.selectbox("Select metric", [
            "corner_count", 
            "enclosed_areas_count", 
            "occupied_proportion"
        ])

        # --- Load all submap files ---
        submap_files = sorted(
            [f for f in sequence_dir.glob("submap_*.pkl")],
            key=lambda f: int(re.search(r"submap_(\d+)\.pkl", f.name).group(1))
        )

        if not submap_files:
            st.warning("No submap_*.pkl files found in the directory.")
            return

        values = []

        for f in submap_files:
            try:
                submap = Submap.load(f)
                layer = getattr(submap, map_type)

                if metric_type == "corner_count":
                    calc = CornerCountCalculator(layer)
                elif metric_type == "enclosed_areas_count":
                    calc = EnclosedAreasCalculator(layer)
                elif metric_type == "occupied_proportion":
                    calc = OccupiedProportionCalculator(layer)
                else:
                    st.error(f"Unknown metric: {metric_type}")
                    return

                metric = calc.calculate()[metric_type]
                values.append(metric.value)
            except Exception as e:
                st.warning(f"Failed to process {f.name}: {e}")
                values.append(np.nan)

        # --- Plot the metric over time ---
        st.markdown(f"### Metric: `{metric_type}` over time")
        fig, ax = plt.subplots()
        ax.plot(range(len(values)), values, marker='o')
        ax.set_xlabel("Submap index")
        ax.set_ylabel(metric_type.replace("_", " ").capitalize())
        ax.set_title(f"{metric_type.replace('_', ' ').capitalize()} over time")
        ax.grid(True)
        st.pyplot(fig)

        # --- Show basic statistics ---
        valid_values = [v for v in values if v is not None and not np.isnan(v)]
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
        st.markdown("## Compare Two Metrics (Binned Histogram)")

        map_type_1 = st.selectbox("Map type 1", ["map", "alpha", "intensity"], key="map1")
        metric_type_1 = st.selectbox("Metric 1", [
            "corner_count", 
            "enclosed_areas_count", 
            "occupied_proportion"
        ], key="metric1")

        map_type_2 = st.selectbox("Map type 2", ["map", "alpha", "intensity"], key="map2")
        metric_type_2 = st.selectbox("Metric 2", [
            "corner_count", 
            "enclosed_areas_count", 
            "occupied_proportion"
        ], key="metric2")

        bins = st.number_input("Number of bins", min_value=2, max_value=100, value=10)

        if map_type_1 == map_type_2 and metric_type_1 == metric_type_2:
            st.warning("Please select two different map+metric combinations to compare.")
        else:
            metric1_values = []
            metric2_values = []

            for f in submap_files:
                try:
                    submap = Submap.load(f)
                    layer1 = getattr(submap, map_type_1)
                    layer2 = getattr(submap, map_type_2)

                    if metric_type_1 == "corner_count":
                        calc1 = CornerCountCalculator(layer1)
                    elif metric_type_1 == "enclosed_areas_count":
                        calc1 = EnclosedAreasCalculator(layer1)
                    else:
                        calc1 = OccupiedProportionCalculator(layer1)

                    if metric_type_2 == "corner_count":
                        calc2 = CornerCountCalculator(layer2)
                    elif metric_type_2 == "enclosed_areas_count":
                        calc2 = EnclosedAreasCalculator(layer2)
                    else:
                        calc2 = OccupiedProportionCalculator(layer2)

                    val1 = calc1.calculate()[metric_type_1].value
                    val2 = calc2.calculate()[metric_type_2].value

                    if not (np.isnan(val1) or np.isnan(val2)):
                        metric1_values.append(val1)
                        metric2_values.append(val2)

                except Exception as e:
                    st.warning(f"Error loading {f.name}: {e}")

            if len(metric1_values) > 0:
                metric1_array = np.array(metric1_values)
                metric2_array = np.array(metric2_values)

                bin_edges = np.histogram_bin_edges(metric1_array, bins=bins)
                bin_indices = np.digitize(metric1_array, bins=bin_edges, right=False)

                binned_means = []
                binned_labels = []

                for i in range(1, len(bin_edges)):
                    # Include values in [left, right) except for the last bin which is [left, right]
                    in_bin = (metric1_array >= bin_edges[i-1]) & (metric1_array < bin_edges[i]) if i < len(bin_edges) - 1 else (metric1_array >= bin_edges[i-1]) & (metric1_array <= bin_edges[i])
                    bin_values = metric2_array[in_bin]

                    if bin_values.size > 0:
                        binned_means.append(np.mean(bin_values))
                    else:
                        binned_means.append(np.nan)

                    # Label with bin range
                    label = f"[{bin_edges[i-1]:.2f} – {bin_edges[i]:.2f}]"
                    binned_labels.append(label)

                st.markdown(f"### Histogram: Average `{metric_type_2}` by `{metric_type_1}` bins")

                fig2, ax2 = plt.subplots()
                ax2.bar(binned_labels, binned_means)
                ax2.set_ylabel(f"Avg {metric_type_2}")
                ax2.set_xlabel(f"{metric_type_1} bins")
                ax2.set_title(f"{metric_type_2} vs {metric_type_1}")
                plt.xticks(rotation=45)
                st.pyplot(fig2)
            else:
                st.warning("Not enough valid data for comparison.")
