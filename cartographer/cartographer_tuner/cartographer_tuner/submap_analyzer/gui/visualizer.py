#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from io import BytesIO
import streamlit as st
from typing import Tuple, Optional

def normalize_array(array: np.ndarray) -> np.ndarray:
    """
    Normalize array values to range [0, 1] for visualization.
    
    Args:
        array: Input numpy array
        
    Returns:
        Normalized array with values between 0 and 1
    """
    min_val = array.min()
    max_val = array.max()
    
    if max_val > min_val:
        return (array - min_val) / (max_val - min_val)
    else:
        return array  # Return as is if all values are the same

def extract_intensity_and_alpha(occupancy_grid: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Extract intensity and alpha channels from occupancy grid.
    Handles both single-channel and multi-channel occupancy grids.
    
    Args:
        occupancy_grid: Occupancy grid data as numpy array
        
    Returns:
        Tuple of (intensity, alpha) arrays
    """
    # Convert to float for normalization
    grid = occupancy_grid.astype(np.float32)
    
    # Check shape to determine how to extract channels
    if len(grid.shape) == 3 and grid.shape[0] == 2:
        # We have a grid with shape (2, height, width)
        # First channel is intensity, second is alpha
        intensity = normalize_array(grid[0])
        alpha = normalize_array(grid[1])
    elif len(grid.shape) == 3 and grid.shape[0] > 2:
        # We have a grid with more channels, just use the first two
        intensity = normalize_array(grid[0])
        alpha = normalize_array(grid[1]) if grid.shape[0] > 1 else intensity
    else:
        # For single-channel data, use the same data for both
        intensity = normalize_array(grid)
        alpha = intensity.copy()
    
    return intensity, alpha

def create_submap_figure(
    occupancy_grid: np.ndarray,
    title: str = "Submap Visualization"
) -> Figure:
    """
    Create a matplotlib figure showing intensity and alpha visualizations
    side by side.
    
    Args:
        occupancy_grid: Occupancy grid data
        title: Title for the figure
        
    Returns:
        Matplotlib figure object
    """
    intensity, alpha = extract_intensity_and_alpha(occupancy_grid)
    
    fig, axes = plt.subplots(1, 2, figsize=(10, 5))
    fig.suptitle(title)
    
    # Plot intensity
    im1 = axes[0].imshow(intensity, cmap='gray')
    axes[0].set_title("Intensity")
    axes[0].axis('off')
    fig.colorbar(im1, ax=axes[0], orientation='vertical', fraction=0.05)
    
    # Plot alpha
    im2 = axes[1].imshow(alpha, cmap='gray')
    axes[1].set_title("Alpha")
    axes[1].axis('off')
    fig.colorbar(im2, ax=axes[1], orientation='vertical', fraction=0.05)
    
    fig.tight_layout()
    return fig

def display_submap(occupancy_grid: np.ndarray, title: str = "Submap Visualization"):
    """
    Display a submap in the Streamlit app.
    
    Args:
        occupancy_grid: Occupancy grid data
        title: Title for the figure
    """
    if occupancy_grid is None:
        st.warning("No submap data available.")
        return
    
    # Display shape and debug info if needed
    st.markdown(f"**Data Shape:** {occupancy_grid.shape}")
    
    try:
        fig = create_submap_figure(occupancy_grid, title)
        st.pyplot(fig)
    except Exception as e:
        st.error(f"Error visualizing submap: {e}")
        st.error("Attempting alternative visualization...")
        
        # Try alternative visualization if main one fails
        try:
            if len(occupancy_grid.shape) == 3 and occupancy_grid.shape[0] == 2:
                # Show channels separately
                col1, col2 = st.columns(2)
                with col1:
                    st.markdown("### Intensity Channel")
                    st.image(normalize_array(occupancy_grid[0]), use_column_width=True)
                with col2:
                    st.markdown("### Alpha Channel")
                    st.image(normalize_array(occupancy_grid[1]), use_column_width=True)
            else:
                st.image(normalize_array(occupancy_grid), use_column_width=True)
        except Exception as e2:
            st.error(f"Alternative visualization also failed: {e2}")
    
    # Add some basic statistics
    st.write("##### Submap Statistics")
    stats = {
        "Shape": occupancy_grid.shape,
        "Min Value": float(occupancy_grid.min()),
        "Max Value": float(occupancy_grid.max()),
        "Mean Value": float(occupancy_grid.mean()),
        "Std Deviation": float(occupancy_grid.std())
    }
    
    # Create a 2-column layout for statistics
    col1, col2 = st.columns(2)
    for i, (key, value) in enumerate(stats.items()):
        if i % 2 == 0:
            col1.metric(key, value)
        else:
            col2.metric(key, value) 