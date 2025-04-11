import re
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import streamlit as st

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

class SubmapMultiSequenceAnalyzer():

    def __init__(self):
        pass

    def render(self):
        st.markdown("## Multi-Sequence Metric Summary")

        root_dir = SubmapAnalyzerState.get_working_path()
        if not isinstance(root_dir, Path) or not root_dir.is_dir():
            st.error("Working path must be a directory containing sequence folders.")
            return

        map_type = st.selectbox("Map type", ["map", "alpha", "intensity"])
        metric_type = st.selectbox("Metric", list(METRIC_CALCULATORS))
        stat_type = st.selectbox("Statistic to compute per sequence", list(STAT_FUNCS))

        sequence_dirs = sorted([d for d in root_dir.iterdir() if d.is_dir()])
        if not sequence_dirs:
            st.warning("No sequence folders found.")
            return

        sequence_stats, sequence_names, all_values = [], [], []

        for seq_dir in sequence_dirs:
            values = get_metric_values(seq_dir, map_type, metric_type)
            if values:
                arr = np.array(values)
                sequence_stats.append(STAT_FUNCS[stat_type](arr))
                sequence_names.append(seq_dir.name)
                all_values.append(arr)

        if not sequence_stats:
            st.warning("No valid metrics found.")
            return

        # Bar chart
        st.markdown(f"### {stat_type.capitalize()} of `{metric_type}` per sequence")
        fig, ax = plt.subplots()
        ax.bar(sequence_names, sequence_stats)
        ax.set_title(f"{metric_type.replace('_', ' ').capitalize()} ({stat_type}) per Sequence")
        ax.set_ylabel(f"{stat_type.capitalize()} {metric_type.replace('_', ' ')}")
        ax.set_xlabel("Sequence")
        plt.xticks(rotation=45)
        st.pyplot(fig)

        # Boxplot of per-sequence stats
        st.markdown(f"### Distribution of {stat_type} across sequences (Boxplot)")
        fig_stat_box, ax_stat_box = plt.subplots()
        ax_stat_box.boxplot(sequence_stats, vert=False, showmeans=True)
        ax_stat_box.set_xlabel(f"{stat_type.capitalize()} {metric_type.replace('_', ' ')}")
        ax_stat_box.set_yticks([1])
        ax_stat_box.set_yticklabels([""])
        st.pyplot(fig_stat_box)

        # Boxplot of per-submap metric values by sequence
        st.markdown(f"### Boxplot of per-submap `{metric_type}` values by sequence")
        fig_box, ax_box = plt.subplots()
        ax_box.boxplot(all_values, labels=sequence_names, showmeans=True)
        ax_box.set_ylabel(metric_type.replace('_', ' ').capitalize())
        ax_box.set_title(f"{metric_type.replace('_', ' ').capitalize()} per Sequence")
        plt.xticks(rotation=45)
        st.pyplot(fig_box)

        # --- Metric Comparison Section ---
        st.markdown("---")
        st.markdown("## Compare Two Metrics (per-sequence, binned boxplot)")

        col1, col2 = st.columns(2)
        with col1:
            map_type_1 = st.selectbox("Map type 1", ["map", "alpha", "intensity"], key="map1")
            metric_type_1 = st.selectbox("Metric 1", list(METRIC_CALCULATORS), key="metric1")
            stat_type_1 = st.selectbox("Statistic 1", list(STAT_FUNCS), key="stat1")
        with col2:
            map_type_2 = st.selectbox("Map type 2", ["map", "alpha", "intensity"], key="map2")
            metric_type_2 = st.selectbox("Metric 2", list(METRIC_CALCULATORS), key="metric2")
            stat_type_2 = st.selectbox("Statistic 2", list(STAT_FUNCS), key="stat2")

        bins = st.number_input("Number of bins", min_value=2, max_value=100, value=10)

        if (map_type_1 == map_type_2 and metric_type_1 == metric_type_2 and stat_type_1 == stat_type_2):
            st.warning("Please choose two different combinations.")
            return

        stat1_values, stat2_values = [], []

        for seq_dir in sequence_dirs:
            v1 = get_metric_values(seq_dir, map_type_1, metric_type_1)
            v2 = get_metric_values(seq_dir, map_type_2, metric_type_2)

            if v1 and v2:
                a1, a2 = np.array(v1), np.array(v2)
                stat1_values.append(STAT_FUNCS[stat_type_1](a1))
                stat2_values.append(STAT_FUNCS[stat_type_2](a2))

        if stat1_values and stat2_values:
            stat1_array = np.array(stat1_values)
            stat2_array = np.array(stat2_values)

            bin_edges = np.histogram_bin_edges(stat1_array, bins=bins)

            bin_values_grouped = []
            binned_labels = []

            for i in range(1, len(bin_edges)):
                in_bin = (stat1_array >= bin_edges[i-1]) & (
                    stat1_array < bin_edges[i] if i < len(bin_edges) - 1 else stat1_array <= bin_edges[i]
                )
                values_in_bin = stat2_array[in_bin]
                if values_in_bin.size > 0:
                    bin_values_grouped.append(values_in_bin)
                    binned_labels.append(f"[{bin_edges[i-1]:.2f} – {bin_edges[i]:.2f}]")

            if bin_values_grouped:
                st.markdown(
                    f"### Boxplot: `{stat_type_2} {metric_type_2}` in bins of `{stat_type_1} {metric_type_1}`"
                )
                fig_box_cmp, ax_box_cmp = plt.subplots()
                ax_box_cmp.boxplot(bin_values_grouped, labels=binned_labels, showmeans=True)
                ax_box_cmp.set_ylabel(f"{stat_type_2} {metric_type_2}")
                ax_box_cmp.set_xlabel(f"{stat_type_1} {metric_type_1} bins")
                ax_box_cmp.set_title(f"{stat_type_2.capitalize()} {metric_type_2} vs. {stat_type_1.capitalize()} {metric_type_1}")
                plt.xticks(rotation=45)
                st.pyplot(fig_box_cmp)
            else:
                st.warning("No data available in any bin for boxplot comparison.")
        else:
            st.warning("Not enough valid data for comparison.")