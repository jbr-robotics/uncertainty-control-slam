#!/usr/bin/env python3

import numpy as np
import gzip
from typing import Tuple, Optional
from cartographer_ros_msgs.srv import SubmapQuery

def extract_occupancy_grid(response: SubmapQuery.Response) -> Optional[np.ndarray]:
    if not isinstance(response, SubmapQuery.Response):
        raise ValueError("Invalid response type")
    
    assert response.textures and len(response.textures) > 0, "No textures in submap response"
    assert len(response.textures) == 1, "Expected exactly one texture in submap response"
    
    texture = response.textures[0]
    width = texture.width
    height = texture.height
    assert width > 0 and height > 0, f"Invalid texture dimensions: {width}x{height}"

    try:
        data = gzip.decompress(texture.cells)
        expected_size = width * height * 2
        
        assert len(data) == expected_size, f"Unexpected data size: {len(data)}, expected {expected_size}"
        
        data = np.frombuffer(data, dtype=np.uint8).reshape(height, width, 2) 
        intensity = data[:, :, 0]
        alpha = data[:, :, 1]
        
        return np.stack([intensity, alpha], axis=0)
    except Exception as e:
        print(f"Error decompressing cells data: {e}")
    
    assert False, "No usable occupancy data (cells or pixels) found"

def get_submap_resolution(response: SubmapQuery.Response) -> float:
    assert response.textures and len(response.textures) > 0, "No textures in submap response"
    return response.textures[0].resolution

def extract_submap_metadata(response: SubmapQuery.Response) -> dict:
    assert response.textures and len(response.textures) > 0, "No textures in submap response"
        
    texture = response.textures[0]
    
    return {
        'resolution': texture.resolution,
        'width': texture.width,
        'height': texture.height,
        'slice_pose': {
            'position': {
                'x': texture.slice_pose.position.x,
                'y': texture.slice_pose.position.y,
                'z': texture.slice_pose.position.z
            },
            'orientation': {
                'x': texture.slice_pose.orientation.x,
                'y': texture.slice_pose.orientation.y,
                'z': texture.slice_pose.orientation.z,
                'w': texture.slice_pose.orientation.w
            }
        },
        'version': response.submap_version
    } 