import gzip
import numpy as np
from typing import Tuple
import pickle

from cartographer_ros_msgs.srv import SubmapQuery

class Submap: 
    def __init__(self, intensity: np.ndarray, 
                 alpha: np.ndarray, 
                 position: Tuple[float, float, float], 
                 orientation: Tuple[float, float, float, float],
                 resolution: float):
        self._intensity = intensity
        self._alpha = alpha
        self._position = position
        self._orientation = orientation
        self._resolution = resolution
        assert self._intensity.shape == self._alpha.shape, "Intensity and alpha must have the same shape"
        
    @classmethod
    def from_SubmapQuery(cls, response: SubmapQuery.Response) -> 'Submap':
        if not isinstance(response, SubmapQuery.Response):
            raise ValueError("Invalid response type")
        
        assert response.textures and len(response.textures) > 0, "No textures in submap response"
        assert len(response.textures) == 1, "Expected exactly one texture in submap response"
        
        texture = response.textures[0]
        width = texture.width
        height = texture.height
        assert width > 0 and height > 0, f"Invalid texture dimensions: {width}x{height}"

        data = gzip.decompress(texture.cells)
        assert len(data) == width * height * 2, f"Unexpected data size: {len(data)}, expected {width * height * 2}"
        data = np.frombuffer(data, dtype=np.uint8).reshape(height, width, 2) 
        intensity = data[:, :, 0]
        alpha = data[:, :, 1]

        position = (texture.slice_pose.position.x, texture.slice_pose.position.y, texture.slice_pose.position.z)
        orientation = (texture.slice_pose.orientation.x, texture.slice_pose.orientation.y, texture.slice_pose.orientation.z, texture.slice_pose.orientation.w)
        resolution = texture.resolution
        return cls(intensity, alpha, position, orientation, resolution)

    @property
    def shape(self) -> Tuple[int, int]:
        return self._intensity.shape
    
    @property
    def intensity(self) -> np.ndarray:
        return self._intensity
    
    @property
    def alpha(self) -> np.ndarray:
        return self._alpha
    
    @property
    def position(self) -> Tuple[float, float, float]:
        return self._position
    
    @property
    def orientation(self) -> Tuple[float, float, float, float]:
        return self._orientation
    
    @property
    def resolution(self) -> float:
        return self._resolution
    
    @property
    def map(self) -> np.ndarray:
        # TODO: This formula may be wrong, need to check
        background = 0.5
        float_alpha = 1 - self._alpha.astype(np.float32) / 255.0
        float_intensity = self._intensity.astype(np.float32) / 255.0 
        result = background * (1.0 - float_intensity) + float_intensity * float_alpha
        return (result * 255.0).astype(np.uint8)
    
    def save(self, path: str):
        with open(path, 'wb') as f:
            pickle.dump(self, f)
    
    @classmethod
    def load(cls, path: str) -> 'Submap':
        with open(path, 'rb') as f:
            result = pickle.load(f)
        assert isinstance(result, cls), "Loaded object is not a Submap"
        return result