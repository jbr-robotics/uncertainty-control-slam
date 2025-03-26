"""Callback type definitions for the GUI components.

This module defines the callback types used for communication between
the ViewModel and the UI components.
"""

from enum import Enum, auto


class CallbackType(Enum):
    """Callback types for communication between ViewModel and UI components."""
    
    # UI update callbacks
    ON_SUBMAP_LIST_UPDATED = auto()
    ON_SUBMAP_ADDED = auto()
    ON_SUBMAP_UPDATED = auto()
    ON_SUBMAP_SELECTED = auto()
    ON_STATUS_CHANGED = auto()
    
    def __str__(self):
        """Return the string representation of the callback type."""
        return self.name 