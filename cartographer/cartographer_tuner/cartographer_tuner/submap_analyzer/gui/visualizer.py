#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from io import BytesIO
import streamlit as st
from typing import Tuple, Optional

def create_bitmap_figure(
    bitmap: np.ndarray,
    title: str = "Bitmap Visualization",
    normalize: bool = False
) -> Figure:
    fig, ax = plt.subplots(figsize=(8, 8))
    fig.suptitle(title)
    
    if normalize:
        im = ax.imshow(bitmap, cmap='gray')
    else:
        im = ax.imshow(bitmap, cmap='gray', vmin=0, vmax=255)
    # ax.axis('off')
    fig.colorbar(im, ax=ax, orientation='vertical', fraction=0.05)
    
    fig.tight_layout()
    return fig

def display_bitmap(data: np.ndarray, title: str = "Bitmap Visualization", normalize: bool = False):
    st.markdown(f"**Data Shape:** {data.shape}")
    
    try:
        fig = create_bitmap_figure(data, title, normalize)
        st.pyplot(fig)
    except Exception as e:
        st.error(f"Error visualizing submap: {e}")
    