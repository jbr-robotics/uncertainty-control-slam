import re
import numpy as np
import pandas as pd
from pathlib import Path
import streamlit as st
import plotly.express as px
import plotly.graph_objects as go
from dataclasses import dataclass

from cartographer_tuner.submap_analyzer.submap import Submap
from cartographer_tuner.submap_analyzer.gui.state import SubmapAnalyzerState
from cartographer_tuner.metrics.calculators.pgm.corner_count_calculator import CornerCountCalculator
from cartographer_tuner.metrics.calculators.pgm.enclosed_areas_calculator import EnclosedAreasCalculator
from cartographer_tuner.metrics.calculators.pgm.occupied_proportion_calculator import OccupiedProportionCalculator


@dataclass
class AnalysisConfig:
    """Configuration for submap analysis."""
    map_type: str = "intensity"
    metric_type: str = "corner_count"
    stat_type: str = "mean"
    min_filter: float = 0.0
    max_filter: float = 1000.0
    normalize: bool = False


def get_plot_range(min_val, max_val):
    """Calculate padding for plot ranges."""
    padding = (max_val - min_val) * 0.05
    return [min_val - padding, max_val + padding]


class DataManager:
    # Constants moved to class level
    MAP_TYPES = ["intensity", "map", "alpha"]
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
    
    @classmethod
    def create_dataframe(cls, root_dir, config):
        """Create raw dataframe of metrics from all submaps across parameter sets.
        
        Args:
            root_dir: Root directory containing parameter set folders
            config: Analysis configuration
            
        Returns:
            DataFrame with extracted metrics data and errors
        """
        data = []
        errors = []
        param_dirs = sorted([d for d in root_dir.iterdir() if d.is_dir()])
        
        for param_dir in param_dirs:
            try:
                param_value = int(param_dir.name)
            except ValueError:
                param_value = None
                
            for f in sorted(param_dir.glob("submap_*.pkl"), key=lambda f: int(re.search(r"submap_(\d+)", f.stem).group(1)) if re.search(r"submap_(\d+)", f.stem) else -1):
                try:
                    submap_id = int(re.search(r"submap_(\d+)", f.stem).group(1))
                    submap = Submap.load(f)
                    layer = getattr(submap, config.map_type)
                    
                    calculator = cls.METRIC_CALCULATORS[config.metric_type](layer)
                    metric_value = calculator.calculate()[config.metric_type].value
                    
                    data.append({
                        "map_id": submap_id,
                        "param_value": param_value,
                        "param_set": param_dir.name,
                        config.metric_type: metric_value,
                    })
                except Exception as e:
                    errors.append(f"Failed to process {f.name} in {param_dir.name}: {str(e)}")
        
        df = pd.DataFrame(data)
        
        # Add errors as metadata to the DataFrame
        df.attrs['errors'] = errors
        
        # Apply normalization if requested
        if config.normalize and not df.empty and config.metric_type in df.columns:
            # Calculate mean metric value per map_id across all parameter sets
            metric_means = df.groupby('map_id')[config.metric_type].transform('mean')
            
            # Add normalized metric column (avoid division by zero)
            df[f"{config.metric_type}_normalized"] = np.where(
                metric_means != 0,
                df[config.metric_type] / metric_means,
                df[config.metric_type]
            )
        
        return df
    
    @classmethod
    def get_parameter_statistics(cls, df_raw, config):
        """Calculate statistics for each parameter set based on the raw data.
        
        Args:
            df_raw: DataFrame with raw metrics
            config: Analysis configuration
            
        Returns:
            DataFrame with parameter set statistics and raw values
        """
        if df_raw.empty:
            return pd.DataFrame(columns=['param_set', 'statistic', 'values'])
            
        result_data = []
        
        # Use normalized values if normalization is enabled
        metric_col = f"{config.metric_type}_normalized" if config.normalize else config.metric_type
        
        for param_set, group in df_raw.groupby('param_set'):
            metric_values = group[metric_col]
            if len(metric_values) > 0:
                value = cls.STAT_FUNCS[config.stat_type](metric_values.values)
                
                if config.min_filter <= value <= config.max_filter:
                    result_data.append({
                        'param_set': param_set,
                        'statistic': value,
                        'values': metric_values.values
                    })
        
        return pd.DataFrame(result_data)
    
    @staticmethod
    def prepare_heatmap_data(params_df):
        """Prepare data for a heatmap/scatter plot visualization.
        
        Args:
            params_df: DataFrame from get_parameter_statistics
            
        Returns:
            DataFrame for heatmap or empty DataFrame if format is invalid
        """
        if params_df.empty:
            return pd.DataFrame(columns=["x_param", "y_param", "metric_value"])
            
        x_values = []
        y_values = []
        param_sets = params_df['param_set'].tolist()
        stats = params_df['statistic'].tolist()
        
        for name in param_sets:
            try:
                x, y = map(float, name.split('_'))
                x_values.append(x)
                y_values.append(y)
            except ValueError:
                return pd.DataFrame(columns=["x_param", "y_param", "metric_value"])
        
        df_heatmap = pd.DataFrame({
            "x_param": x_values,
            "y_param": y_values,
            "metric_value": stats
        })
        
        return df_heatmap
    
    @staticmethod
    def prepare_bar_data(params_df):
        """Prepare data for a bar chart visualization."""
        if params_df.empty:
            return pd.DataFrame(columns=["ParamSet", "MetricValue"])
            
        return pd.DataFrame({
            "ParamSet": params_df['param_set'].tolist(), 
            "MetricValue": params_df['statistic'].tolist()
        })
    
    @staticmethod
    def prepare_boxplot_data(params_df):
        """Prepare data for boxplot visualization.
        
        Returns:
            DataFrame with param_set, values and count columns
        """
        if params_df.empty:
            return pd.DataFrame(columns=["param_set", "values", "submap_count"])
        
        boxplot_data = []
        
        for _, row in params_df.iterrows():
            boxplot_data.append({
                'param_set': row['param_set'],
                'values': row['values'],
                'submap_count': len(row['values'])
            })
            
        return pd.DataFrame(boxplot_data)


class SubmapParamSetsAnalyzer:
    # UI constants
    SCALE_OPTIONS = ["Uniform", "Log X", "Log Y", "Log X and Y"]
    
    def __init__(self):
        self.config = AnalysisConfig()
    
    def render_config_input(self):
        """Render input controls for metric analysis."""
        col1, col2 = st.columns(2)
        with col1:
            self.config.map_type = st.selectbox("Map type", DataManager.MAP_TYPES)
            self.config.min_filter = st.number_input("Min value filter", value=0.0, step=0.1)
            self.config.normalize = st.checkbox("Normalize by map_id mean", value=False, 
                                               help="Divide metric values by mean calculated across all parameter sets with same map_id")
        with col2:
            self.config.metric_type = st.selectbox("Metric", list(DataManager.METRIC_CALCULATORS))
            self.config.max_filter = st.number_input("Max value filter", value=1000.0, step=0.1)
        self.config.stat_type = st.selectbox("Statistic to compute per parameter set", list(DataManager.STAT_FUNCS))
    
    def render_heatmap(self, params_df):
        """Render heatmap visualization for x_y formatted parameter set names."""
        metric_name = f"{self.config.metric_type} (normalized)" if self.config.normalize else self.config.metric_type
        st.markdown(f"### Heatmap of `{metric_name}` ({self.config.stat_type})")
        
        # Get processed data from DataManager
        df_heatmap = DataManager.prepare_heatmap_data(params_df)
        
        if df_heatmap.empty:
            st.warning("One or more parameter set names don't follow the expected 'x_y' format")
            return
        
        # Create a scatter plot instead of a heatmap to show all values explicitly
        fig = px.scatter(
            df_heatmap, 
            x="x_param", 
            y="y_param", 
            color="metric_value",
            size="metric_value",
            size_max=25,
            hover_data=["x_param", "y_param", "metric_value"],
            title=f"{metric_name.replace('_', ' ').capitalize()} ({self.config.stat_type})",
            labels={"metric_value": f"{self.config.stat_type.capitalize()} {metric_name.replace('_', ' ')}"}
        )
        
        # Improve appearance
        fig.update_layout(
            xaxis_title="X Parameter",
            yaxis_title="Y Parameter",
            coloraxis_colorbar_title=f"{self.config.stat_type.capitalize()} Value"
        )
        
        # Add text labels to show exact values
        fig.update_traces(
            textposition='top center',
            texttemplate='%{z:.2f}',
            marker=dict(line=dict(width=1, color='DarkSlateGrey'))
        )
        
        # Add scale toggle options
        scale_option = st.selectbox(
            "Axis Scale",
            self.SCALE_OPTIONS,
            key=f"scale_toggle_{self.config.metric_type}_{self.config.stat_type}"
        )
        
        # Apply scale based on selection
        if scale_option == "Log X" or scale_option == "Log X and Y":
            fig.update_xaxes(type="log")
        if scale_option == "Log Y" or scale_option == "Log X and Y":
            fig.update_yaxes(type="log")
        st.plotly_chart(fig, use_container_width=True)

    def render_bars(self, params_df):
        """Render bar chart visualization for parameter set names."""
        metric_name = f"{self.config.metric_type} (normalized)" if self.config.normalize else self.config.metric_type
        st.markdown(f"### {self.config.stat_type.capitalize()} of `{metric_name}` per parameter set")

        # Get processed data from DataManager
        df_bar = DataManager.prepare_bar_data(params_df)
        
        if df_bar.empty:
            st.warning("No data available for bar chart")
            return
        
        fig_bar = px.bar(df_bar, x="ParamSet", y="MetricValue",
                        title=f"{metric_name.replace('_', ' ').capitalize()} ({self.config.stat_type}) per Parameter Set",
                        labels={"MetricValue": f"{self.config.stat_type.capitalize()} {metric_name.replace('_', ' ')}"},
                        range_y=get_plot_range(df_bar["MetricValue"].min(), df_bar["MetricValue"].max()))

        st.plotly_chart(fig_bar, use_container_width=True)

    def render_boxplots(self, params_df):
        """Render boxplots of per-submap metric values by parameter set."""
        metric_name = f"{self.config.metric_type} (normalized)" if self.config.normalize else self.config.metric_type
        st.markdown(f"### Boxplot of per-submap `{metric_name}` values by parameter set")

        # Get processed data from DataManager
        df_boxplot = DataManager.prepare_boxplot_data(params_df)
        
        if df_boxplot.empty:
            st.warning("No data available for boxplot")
            return
            
        fig_box = go.Figure()

        # Add histogram bars (background)
        fig_box.add_trace(go.Bar(
            x=df_boxplot['param_set'].tolist(),
            y=df_boxplot['submap_count'].tolist(),
            marker=dict(color='lightgray'),
            opacity=0.4,
            yaxis='y2',
            name='Submap Count'
        ))

        # Add boxplots on top
        for _, row in df_boxplot.iterrows():
            fig_box.add_trace(go.Box(
                y=row['values'],
                name=row['param_set'],
                boxmean=True,
                marker_color='steelblue',
                line=dict(width=1)
            ))

        # Dual Y-axes
        fig_box.update_layout(
            title=f"{metric_name.replace('_', ' ').capitalize()} per Parameter Set",
            yaxis=dict(title=metric_name.replace('_', ' ').capitalize()),
            yaxis2=dict(overlaying='y', side='right', showgrid=False, visible=False),
            barmode="overlay"
        )

        st.plotly_chart(fig_box, use_container_width=True)

    def render_metric_summary(self, root_dir):
        """Render summary visualizations for a single metric across parameter sets."""
        self.render_config_input()
        
        # Get raw data using the DataManager
        df_raw = DataManager.create_dataframe(root_dir, self.config)
        
        # Display any errors that occurred during processing
        if 'errors' in df_raw.attrs and df_raw.attrs['errors']:
            for error in df_raw.attrs['errors']:
                st.warning(error)
        
        if df_raw.empty:
            st.warning("No valid metrics found.")
            return

        # Get parameter set statistics from the DataManager
        params_df = DataManager.get_parameter_statistics(df_raw, self.config)
        
        if params_df.empty:
            st.warning("No valid metrics found within filter range.")
            return

        # Render appropriate visualizations based on parameter set name format
        if all(map(lambda x: x.count('_') == 1, params_df['param_set'].tolist())):
            self.render_heatmap(params_df)
        else:
            self.render_bars(params_df)

        self.render_boxplots(params_df)

    def render_raw_data_view(self, root_dir):
        """Render the raw metrics data view."""
        st.write("Creating raw metrics dataframe...")
        
        raw_config = AnalysisConfig()
        raw_config.map_type = st.selectbox("Map type for raw metrics", DataManager.MAP_TYPES)
        raw_config.metric_type = st.selectbox("Metric type for raw metrics", list(DataManager.METRIC_CALCULATORS))
        raw_config.normalize = st.checkbox("Show normalized metrics", value=False)
        
        # Get raw data using the DataManager
        df_raw = DataManager.create_dataframe(root_dir, raw_config)
        
        # Display any errors that occurred during processing
        if 'errors' in df_raw.attrs and df_raw.attrs['errors']:
            for error in df_raw.attrs['errors']:
                st.warning(error)
            
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

    def render(self):
        """Main entry point for rendering the component."""
        st.markdown("## Multi-Parameter Metric Summary")

        root_dir = SubmapAnalyzerState.get_working_path()
        if not isinstance(root_dir, Path) or not root_dir.is_dir():
            st.error("Working path must be a directory containing parameter set folders.")
            return
        
        # Option to show raw metrics data
        if st.checkbox("Show raw metrics data"):
            self.render_raw_data_view(root_dir)
        
        # Render metric summary visualizations
        self.render_metric_summary(root_dir)
