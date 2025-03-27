"""Message type definitions for communication between the node and GUI.

This module defines the message types used for communication between
the ROS node and the GUI components.
"""

from enum import Enum, auto
from dataclasses import dataclass
from typing import Dict, Any, Optional, List, Tuple


class MessageType(Enum):
    """Message types for communication between the node and GUI."""
    
    # Messages from node to GUI
    SUBMAP_LIST_UPDATED = auto()
    NEW_SUBMAP = auto()
    
    # Messages from GUI to node
    REQUEST_SUBMAP = auto()
    
    def __str__(self):
        """Return the string representation of the message type."""
        return self.name


@dataclass
class SubmapListUpdatedMessage:
    """Message for SUBMAP_LIST_UPDATED type."""
    count: int
    submap_ids: List[Tuple[int, int]]  # List of (trajectory_id, submap_index) tuples


@dataclass
class NewSubmapMessage:
    """Message for NEW_SUBMAP type."""
    trajectory_id: int
    submap_index: int
    file_path: str
    resolution: float
    width: int
    height: int
    # Additional fields can be added as needed


@dataclass
class RequestSubmapMessage:
    """Message for REQUEST_SUBMAP type."""
    trajectory_id: int
    submap_index: int
    # force_reload is removed as it's always True now 