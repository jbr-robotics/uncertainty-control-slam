import re
import numpy as np
import pandas as pd
from pathlib import Path
import streamlit as st
import plotly.express as px
import plotly.graph_objects as go
from dataclasses import dataclass
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression
from sklearn.pipeline import make_pipeline
from scipy.spatial import cKDTree
from scipy.interpolate import griddata

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
    # 3D plot configuration
    bucket_count: int = 5


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
            for f in sorted(param_dir.glob("submap_*.pkl"), key=lambda f: int(re.search(r"submap_(\d+)", f.stem).group(1)) if re.search(r"submap_(\d+)", f.stem) else -1):
                try:
                    submap_id = int(re.search(r"submap_(\d+)", f.stem).group(1))
                    submap = Submap.load(f)
                    layer = getattr(submap, config.map_type)
                    
                    calculator = cls.METRIC_CALCULATORS[config.metric_type](layer)
                    metric_value = calculator.calculate()[config.metric_type].value
                    
                    data.append({
                        "map_id": submap_id,
                        "param_set": param_dir.name,
                        config.metric_type: metric_value,
                    })
                except Exception as e:
                    errors.append(f"Failed to process {f.name} in {param_dir.name}: {str(e)}")
        
        df = pd.DataFrame(data)

        # Try to convert param_set to numeric types if possible
        if not df.empty and 'param_set' in df.columns:
            # First check if all param_set values can be cast to int
            try:
                # Check if all values can be converted to int without data loss
                if all(float(x).is_integer() for x in df['param_set']):
                    df['param_set'] = df['param_set'].astype(int)
                # If not all are integers, try converting to float
                else:
                    df['param_set'] = df['param_set'].astype(float)
            except (ValueError, TypeError):
                # If conversion fails, keep as string
                pass
        
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
        
    @classmethod
    def prepare_3d_histogram_data(cls, df_raw, config):
        """Prepare data for 3D histogram visualization based on alternative design.
        
        Args:
            df_raw: DataFrame with raw metrics
            config: Analysis configuration
            
        Returns:
            DataFrame with x (num_range_data), y (corner_count_bucket), z (metric_value), and variation
        """
        if df_raw.empty:
            return pd.DataFrame(columns=["num_range_data", "corner_count_bucket", "metric_value", "variation"])
            
        # Assume param_set is num_range_data
        try:
            df_raw['num_range_data'] = pd.to_numeric(df_raw['param_set'])
        except ValueError:
            # If conversion fails, return empty DataFrame
            return pd.DataFrame(columns=["num_range_data", "corner_count_bucket", "metric_value", "variation"])
        
        # Use normalized values if normalization is enabled
        metric_col = f"{config.metric_type}_normalized" if config.normalize else config.metric_type
        
        # Create buckets for corner_count
        if config.metric_type in df_raw.columns:
            min_val = df_raw[config.metric_type].min()
            max_val = df_raw[config.metric_type].max()
            bucket_edges = np.linspace(min_val, max_val, config.bucket_count + 1)
            
            df_raw['corner_count_bucket'] = pd.cut(
                df_raw[config.metric_type], 
                bins=bucket_edges,
                labels=[f"{bucket_edges[i]:.1f}-{bucket_edges[i+1]:.1f}" for i in range(len(bucket_edges)-1)]
            )
            
            # Group by num_range_data and corner_count_bucket
            grouped = df_raw.groupby(['num_range_data', 'corner_count_bucket'])
            
            # Calculate statistics
            result_data = []
            for (num_range_data, bucket), group in grouped:
                if not group.empty:
                    metric_values = group[metric_col]
                    
                    # Calculate mean and variation
                    mean_val = cls.STAT_FUNCS[config.stat_type](metric_values)
                    variation = np.std(metric_values) / np.mean(metric_values) if np.mean(metric_values) != 0 else 0
                    
                    result_data.append({
                        'num_range_data': num_range_data,
                        'corner_count_bucket': bucket,
                        'metric_value': mean_val,
                        'variation': variation,
                        'count': len(group)
                    })
            
            return pd.DataFrame(result_data)
        
        return pd.DataFrame(columns=["num_range_data", "corner_count_bucket", "metric_value", "variation"])


class SubmapParamSetsAnalyzer:
    # UI constants
    SCALE_OPTIONS = ["Uniform", "Log X", "Log Y", "Log X and Y"]
    
    def __init__(self):
        self.config = AnalysisConfig()
    
    def render_config_input(self):
        """Render input controls for metric analysis."""
        col1, col2 = st.columns(2)
        with col1:
            self.config.map_type = st.selectbox("Map type", DataManager.MAP_TYPES, key="map_type_config")
            self.config.min_filter = st.number_input("Min value filter", value=0.0, step=0.1, key="min_filter_config")
            self.config.normalize = st.checkbox("Normalize by map_id mean", value=False, 
                                               help="Divide metric values by mean calculated across all parameter sets with same map_id",
                                               key="normalize_config")
        with col2:
            self.config.metric_type = st.selectbox("Metric", list(DataManager.METRIC_CALCULATORS), key="metric_type_config")
            self.config.max_filter = st.number_input("Max value filter", value=1000.0, step=0.1, key="max_filter_config")
        self.config.stat_type = st.selectbox("Statistic to compute per parameter set", list(DataManager.STAT_FUNCS), key="stat_type_config")
    
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
            key=f"scale_toggle_{self.config.metric_type}_{self.config.stat_type}_heatmap"
        )
        
        # Apply scale based on selection
        if scale_option == "Log X" or scale_option == "Log X and Y":
            fig.update_xaxes(type="log")
        if scale_option == "Log Y" or scale_option == "Log X and Y":
            fig.update_yaxes(type="log")
        st.plotly_chart(fig, use_container_width=True)

    def render_bars(self, params_df):
        """Render bar chart visualization for parameter set names with smoothed trend line."""
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
                        range_y=get_plot_range(df_bar["MetricValue"].min(), df_bar["MetricValue"].max()),
                        )

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

    def render_3d_histogram(self, root_dir):
        """Render 3D histogram visualization based on alternative experiment design."""
        st.markdown("## 3D Visualization of Alternative Experiment Design")
        st.markdown("""
        This visualization shows data according to the alternative experimental design, where:
        - X-axis: `num_range_data` values
        - Y-axis: `corner_count` buckets
        - Z-axis: Mean value of map quality metric
        """)
        
        # Use a different instance of render_config_input
        col1, col2 = st.columns(2)
        with col1:
            self.config.map_type = st.selectbox("Map type", DataManager.MAP_TYPES, key="map_type_3d")
            self.config.min_filter = st.number_input("Min value filter", value=0.0, step=0.1, key="min_filter_3d")
            self.config.normalize = st.checkbox("Normalize by map_id mean", value=False, 
                                               help="Divide metric values by mean calculated across all parameter sets with same map_id",
                                               key="normalize_3d")
        with col2:
            self.config.metric_type = st.selectbox("Metric", list(DataManager.METRIC_CALCULATORS), key="metric_type_3d")
            self.config.max_filter = st.number_input("Max value filter", value=1000.0, step=0.1, key="max_filter_3d")
        self.config.stat_type = st.selectbox("Statistic to compute per parameter set", list(DataManager.STAT_FUNCS), key="stat_type_3d")
        
        # Add visualization options
        st.markdown("### Visualization Options")
        col1, col2 = st.columns(2)
        with col1:
            self.config.bucket_count = st.slider("Number of corner_count buckets", 
                                                min_value=3, max_value=10, value=5, 
                                                key="bucket_count_3d")
            
            # Display mode selection
            display_mode = st.radio(
                "Display mode",
                ["Points", "Surface (Local Smoothing)"],
                key="display_mode_3d",
                help="Choose to display individual data points or a smoothed surface using local methods"
            )
            
        with col2:
            use_fixed_marker_size = st.checkbox("Use fixed marker size", value=False, 
                                            help="Use a fixed marker size instead of scaling by sample count",
                                            key="fixed_marker_size_3d")
            if use_fixed_marker_size:
                fixed_marker_size = st.slider("Marker size", min_value=5, max_value=30, value=15, key="marker_size_value_3d")
            
            # Surface smoothing options (only show when surface mode is selected)
            if display_mode == "Surface (Local Smoothing)":
                surface_resolution = st.slider(
                    "Surface resolution", 
                    min_value=10, 
                    max_value=50, 
                    value=20, 
                    key="surface_resolution_3d",
                    help="Higher values create a smoother surface but take longer to render"
                )
                
                smoothing_method = st.selectbox(
                    "Smoothing method",
                    ["Nearest Neighbors", "Inverse Distance", "Linear", "Cubic"],
                    index=2,
                    key="smoothing_method_3d",
                    help="Method used for local smoothing"
                )
                
                if smoothing_method == "Nearest Neighbors":
                    neighbors_count = st.slider(
                        "Number of neighbors", 
                        min_value=2, 
                        max_value=10, 
                        value=5,
                        key="neighbors_count_3d",
                        help="Number of neighbors to consider for local averaging"
                    )
                
                show_points_with_surface = st.checkbox(
                    "Show points with surface", 
                    value=True,
                    key="show_points_with_surface_3d",
                    help="Display the original data points along with the smoothed surface"
                )
        
        # Get raw data using the DataManager
        df_raw = DataManager.create_dataframe(root_dir, self.config)
        
        # Display any errors that occurred during processing
        if 'errors' in df_raw.attrs and df_raw.attrs['errors']:
            for error in df_raw.attrs['errors']:
                st.warning(error)
        
        if df_raw.empty:
            st.warning("No valid metrics found.")
            return
        
        # Get 3D histogram data
        df_3d = DataManager.prepare_3d_histogram_data(df_raw, self.config)
        
        if df_3d.empty:
            st.warning("Could not prepare 3D histogram data. Ensure param_set is numeric.")
            return
        
        # Create a colorscale based on the variation
        max_variation = df_3d['variation'].max() if not df_3d.empty else 1.0
        variation_threshold = st.slider("Variation threshold for highlighting", 
                                       min_value=0.0, max_value=float(max_variation), 
                                       value=float(max_variation/3),
                                       key="variation_threshold_3d")
        
        marker_size_description = "Fixed size" if use_fixed_marker_size else "Number of samples in each bucket"
        
        if display_mode == "Points":
            vis_description = f"""
            ### 3D Points Visualization
            - **Color intensity**: Represents the metric value
            - **Size of markers**: {marker_size_description}
            - **Opacity**: Higher opacity indicates lower variation (more reliable data)
            
            Buckets with variation above {variation_threshold:.2f} are shown with lower opacity.
            """
        else:
            method_description = ""
            if smoothing_method == "Nearest Neighbors":
                method_description = f"using {neighbors_count} nearest neighbors for local averaging"
            elif smoothing_method == "Inverse Distance":
                method_description = "weighted by the inverse of distance"
            elif smoothing_method == "Linear":
                method_description = "using linear interpolation between points"
            elif smoothing_method == "Cubic":
                method_description = "using cubic spline interpolation"
                
            vis_description = f"""
            ### 3D Surface with Local Smoothing
            - **Color intensity**: Represents the metric value
            - **Surface**: Smoothed approximation ({method_description})
            - **Resolution**: {surface_resolution}x{surface_resolution} grid
            
            Local smoothing methods work well with noisy data by focusing on neighborhood patterns.
            """
            
        st.markdown(vis_description)
        
        # Create 3D figure
        fig_3d = go.Figure()
        
        # Convert bucket labels to numeric values for smoothing
        bucket_numeric = {}
        for i, bucket in enumerate(sorted(df_3d['corner_count_bucket'].unique())):
            bucket_numeric[bucket] = i
            
        df_3d['bucket_numeric'] = df_3d['corner_count_bucket'].map(bucket_numeric)
        
        # Display mode: Surface (Local Smoothing)
        if display_mode == "Surface (Local Smoothing)":
            # Prepare data for local smoothing
            x = df_3d['num_range_data'].values
            y = df_3d['bucket_numeric'].values
            z = df_3d['metric_value'].values
            
            # Need at least 4 points for surface creation
            if len(x) >= 4:
                try:
                    # Create a grid for the surface
                    xi = np.linspace(min(x), max(x), surface_resolution)
                    yi = np.linspace(min(y), max(y), surface_resolution)
                    xi_grid, yi_grid = np.meshgrid(xi, yi)
                    
                    # Choose the appropriate local smoothing method
                    if smoothing_method == "Nearest Neighbors":
                        # K-nearest neighbor smoothing
                        points = np.vstack((x, y)).T
                        tree = cKDTree(points)
                        zi_grid = np.zeros(xi_grid.shape)
                        
                        for i in range(xi_grid.shape[0]):
                            for j in range(xi_grid.shape[1]):
                                query_point = np.array([xi_grid[i, j], yi_grid[i, j]])
                                distances, indices = tree.query(query_point, k=neighbors_count)
                                
                                # Handle case where query point is exactly at a data point
                                if distances[0] == 0:
                                    zi_grid[i, j] = z[indices[0]]
                                else:
                                    # Average the values of k nearest neighbors
                                    zi_grid[i, j] = np.mean(z[indices])
                    else:
                        # Use scipy's griddata for other methods
                        method_map = {
                            "Inverse Distance": "nearest",  # Not exactly inverse distance but close
                            "Linear": "linear",
                            "Cubic": "cubic"
                        }
                        
                        zi_grid = griddata(
                            (x, y), z, 
                            (xi_grid, yi_grid), 
                            method=method_map[smoothing_method]
                        )
                    
                    # Create a mapping from numeric buckets back to labels for y-axis
                    y_labels = [key for key, value in sorted(bucket_numeric.items(), key=lambda item: item[1])]
                    y_label_positions = list(range(len(y_labels)))
                    
                    # Add the surface
                    fig_3d.add_trace(go.Surface(
                        x=xi,
                        y=yi,
                        z=zi_grid,
                        colorscale='Viridis',
                        opacity=0.8,
                        showscale=True,
                        colorbar=dict(title=f"{self.config.metric_type} ({self.config.stat_type})"),
                        hoverinfo='none'
                    ))
                    
                    # Customize layout
                    fig_3d.update_layout(
                        scene=dict(
                            xaxis_title='num_range_data',
                            yaxis=dict(
                                title='corner_count_bucket',
                                tickmode='array',
                                tickvals=y_label_positions,
                                ticktext=y_labels
                            ),
                            zaxis_title=f"{self.config.metric_type} ({self.config.stat_type})",
                        )
                    )
                    
                    # Also show points if requested
                    if show_points_with_surface:
                        for _, row in df_3d.iterrows():
                            opacity = 1.0 if row['variation'] <= variation_threshold else 0.3
                            
                            # Determine marker size based on user preference
                            if use_fixed_marker_size:
                                marker_size = fixed_marker_size
                            else:
                                marker_size = row['count'] * 2  # Size based on sample count, smaller than in points-only mode
                            
                            fig_3d.add_trace(go.Scatter3d(
                                x=[row['num_range_data']],
                                y=[row['bucket_numeric']],
                                z=[row['metric_value']],
                                mode='markers',
                                marker=dict(
                                    size=marker_size,
                                    color=row['metric_value'],
                                    opacity=opacity,
                                    colorscale='Viridis',
                                ),
                                text=f"num_range_data: {row['num_range_data']}<br>" +
                                     f"bucket: {row['corner_count_bucket']}<br>" +
                                     f"metric: {row['metric_value']:.2f}<br>" +
                                     f"variation: {row['variation']:.2f}<br>" +
                                     f"count: {row['count']}",
                                hoverinfo='text',
                                name=f"{row['num_range_data']}_{row['corner_count_bucket']}"
                            ))
                    
                except Exception as e:
                    st.warning(f"Could not create smoothed surface: {str(e)}")
                    st.info("Falling back to points display mode.")
                    display_mode = "Points"  # Fall back to points mode
            else:
                st.warning("Need at least 4 data points to create a smoothed surface. Falling back to points display.")
                display_mode = "Points"  # Fall back to points mode
            
        # Display mode: Points (default or fallback)
        if display_mode == "Points":
            # Add points with varying opacity based on variation
            for _, row in df_3d.iterrows():
                opacity = 1.0 if row['variation'] <= variation_threshold else 0.3
                
                # Determine marker size based on user preference
                if use_fixed_marker_size:
                    marker_size = fixed_marker_size
                else:
                    marker_size = row['count']*3  # Size based on sample count
                
                fig_3d.add_trace(go.Scatter3d(
                    x=[row['num_range_data']],
                    y=[row['corner_count_bucket']],
                    z=[row['metric_value']],
                    mode='markers',
                    marker=dict(
                        size=marker_size,
                        color=row['metric_value'],
                        opacity=opacity,
                        colorscale='Viridis',
                    ),
                    text=f"num_range_data: {row['num_range_data']}<br>" +
                         f"bucket: {row['corner_count_bucket']}<br>" +
                         f"metric: {row['metric_value']:.2f}<br>" +
                         f"variation: {row['variation']:.2f}<br>" +
                         f"count: {row['count']}",
                    hoverinfo='text',
                    name=f"{row['num_range_data']}_{row['corner_count_bucket']}"
                ))
        
        # Common layout settings
        fig_3d.update_layout(
            margin=dict(l=0, r=0, b=0, t=30),
            title=f"3D Visualization of {self.config.metric_type} by num_range_data and corner_count"
        )
        
        st.plotly_chart(fig_3d, use_container_width=True)
        
        # Display the data table
        with st.expander("Show data table"):
            st.dataframe(df_3d)
            
            # Option to download as CSV
            if not df_3d.empty:
                csv = df_3d.to_csv(index=False)
                st.download_button(
                    label="Download 3D histogram data as CSV",
                    data=csv,
                    file_name="3d_histogram_data.csv",
                    mime="text/csv",
                    key="download_3d_csv"
                )

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
        if all(map(lambda x: str(x).count('_') == 1, params_df['param_set'].tolist())):
            self.render_heatmap(params_df)
        else:
            self.render_bars(params_df)

        self.render_boxplots(params_df)

    def render_raw_data_view(self, root_dir):
        """Render the raw metrics data view."""
        st.write("Creating raw metrics dataframe...")
        
        raw_config = AnalysisConfig()
        raw_config.map_type = st.selectbox("Map type for raw metrics", DataManager.MAP_TYPES, key="map_type_raw")
        raw_config.metric_type = st.selectbox("Metric type for raw metrics", list(DataManager.METRIC_CALCULATORS), key="metric_type_raw")
        raw_config.normalize = st.checkbox("Show normalized metrics", value=False, key="normalize_raw")
        
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
                key="download_raw_csv"
            )

    def render(self):
        """Main entry point for rendering the component."""
        st.markdown("## Multi-Parameter Metric Summary")

        root_dir = SubmapAnalyzerState.get_working_path()
        if not isinstance(root_dir, Path) or not root_dir.is_dir():
            st.error("Working path must be a directory containing parameter set folders.")
            return
        
        # Create tabs for different visualizations
        tab1, tab2, tab3 = st.tabs(["Parameter Analysis", "3D Visualization", "Raw Data"])
        
        with tab1:
            # Original parameter analysis view
            self.render_metric_summary(root_dir)
        
        with tab2:
            # New 3D visualization for alternative experiment design
            self.render_3d_histogram(root_dir)
        
        with tab3:
            # Raw data view
            self.render_raw_data_view(root_dir)
