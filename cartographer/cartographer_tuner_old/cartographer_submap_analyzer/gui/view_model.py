"""ViewModel for the submap analyzer GUI.

This module contains the ViewModel class that handles data preparation and business logic
for the submap analyzer GUI, keeping the View layer as simple as possible.
"""

import os
import logging
import queue
from typing import Dict, List, Optional, Any, Tuple, Set, Callable
from pathlib import Path
from datetime import datetime

# Configure logging first, before any other imports
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] [%(name)s] %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S',
    force=True  # Force reconfiguration of the root logger
)
logger = logging.getLogger('submap_analyzer.gui.view_model')
logger.setLevel(logging.INFO)  # Ensure our view model logs are visible

# Now import other modules
from PIL import Image

from .message_types import (
    MessageType, 
    SubmapListUpdatedMessage, 
    NewSubmapMessage, 
    RequestSubmapMessage
)
from .callback_types import CallbackType

class SubmapViewModel:
    """ViewModel for the submap analyzer GUI.
    
    This class handles data preparation and business logic for the GUI,
    including communication with the ROS node, submap data management,
    and event handling.
    """
    
    def __init__(self, output_dir: str, update_queue: queue.Queue):
        """Initialize the ViewModel.
        
        Args:
            output_dir: Directory where submaps are saved
            update_queue: Queue for communication with the ROS node
        """
        self.output_dir = Path(output_dir)
        self.update_queue = update_queue
        
        # State tracking
        self.submaps = {}  # (trajectory_id, submap_index) -> submap data
        self.selected_submap_id = None
        
        # Callbacks for UI updates
        self.callbacks = {
            CallbackType.ON_SUBMAP_LIST_UPDATED: [],
            CallbackType.ON_SUBMAP_ADDED: [],
            CallbackType.ON_SUBMAP_UPDATED: [],
            CallbackType.ON_SUBMAP_SELECTED: [],
            CallbackType.ON_STATUS_CHANGED: [],
        }
        
        logger.info("ViewModel initialized")
    
    def register_callback(self, event_type: CallbackType, callback: Callable) -> None:
        """Register a callback for a specific event type.
        
        Args:
            event_type: Type of event to register for
            callback: Function to call when the event occurs
        """
        if event_type in self.callbacks:
            self.callbacks[event_type].append(callback)
            callback_count = len(self.callbacks[event_type])
            logger.info(f"[TRACK] Registered callback for {event_type} (total: {callback_count})")
            logger.info(f"[TRACK] Callback function: {callback.__name__ if hasattr(callback, '__name__') else str(callback)}")
        else:
            logger.warning(f"[TRACK] Unknown event type: {event_type}")
    
    def _notify(self, event_type: CallbackType, *args, **kwargs) -> None:
        """Notify all registered callbacks for an event type.
        
        Args:
            event_type: Type of event that occurred
            *args, **kwargs: Arguments to pass to the callbacks
        """
        if event_type in self.callbacks:
            callback_count = len(self.callbacks[event_type])
            logger.info(f"[TRACK] Notifying {callback_count} callbacks for event type {event_type}")
            
            for i, callback in enumerate(self.callbacks[event_type]):
                try:
                    logger.info(f"[TRACK] Executing callback {i+1}/{callback_count} for {event_type}")
                    callback(*args, **kwargs)
                    logger.info(f"[TRACK] Successfully executed callback {i+1}/{callback_count} for {event_type}")
                except Exception as e:
                    logger.error(f"[TRACK] Error in callback {i+1}/{callback_count} for {event_type}: {e}", exc_info=True)
    
    def process_updates(self) -> None:
        """Process updates from the ROS node.
        
        This method should be called periodically to check for updates
        from the ROS node and update the UI accordingly.
        """
        try:
            # Process all available updates
            while not self.update_queue.empty():
                update_type, data = self.update_queue.get_nowait()
                logger.info(f"[TRACK] Received message from queue: type={update_type}")
                
                if update_type == MessageType.SUBMAP_LIST_UPDATED:
                    count = data.count
                    submap_ids = data.submap_ids
                    
                    logger.info(f"[TRACK] Processing SUBMAP_LIST_UPDATED message with {count} submaps")
                    logger.info(f"[TRACK] Submap IDs in message: {submap_ids}")
                    
                    # Update status
                    status_text = f"Found {count} submaps"
                    self._notify(CallbackType.ON_STATUS_CHANGED, status_text)
                    logger.info(f"[TRACK] Updated status: {status_text}")
                    
                    # Request any submaps that we don't have yet
                    self._ensure_all_submaps_loaded(submap_ids)
                
                elif update_type == MessageType.NEW_SUBMAP:
                    # Process new submap data
                    if isinstance(data, dict):
                        # Handle legacy dictionary format for backward compatibility
                        logger.info(f"[TRACK] Received NEW_SUBMAP in legacy dictionary format")
                        trajectory_id = data.get('trajectory_id', 'unknown')
                        submap_index = data.get('submap_index', 'unknown')
                        logger.info(f"[TRACK] Processing submap {trajectory_id}_{submap_index} from legacy format")
                        self._process_submap_data(data)
                    else:
                        # Handle new dataclass format
                        logger.info(f"[TRACK] Received NEW_SUBMAP in dataclass format")
                        logger.info(f"[TRACK] Processing submap {data.trajectory_id}_{data.submap_index} from dataclass")
                        self._process_submap_data_from_message(data)
                
                elif update_type == MessageType.REQUEST_SUBMAP:
                    # This is a message we sent to the node, not one we need to process
                    # Just log it for debugging purposes
                    if isinstance(data, dict):
                        trajectory_id = data.get('trajectory_id', 'unknown')
                        submap_index = data.get('submap_index', 'unknown')
                    else:
                        trajectory_id = data.trajectory_id
                        submap_index = data.submap_index
                    logger.info(f"[TRACK] Sent request for submap {trajectory_id}_{submap_index}")
                
                else:
                    # Log unknown update types but don't crash
                    logger.warning(f"[TRACK] Unknown update type: {update_type}")
            
        except queue.Empty:
            pass
        except Exception as e:
            logger.error(f"[TRACK] Error processing updates: {e}", exc_info=True)
    
    def _process_submap_data_from_message(self, message: NewSubmapMessage) -> None:
        """Process submap data received from the ROS node as a dataclass message.
        
        Args:
            message: NewSubmapMessage with submap information
        """
        # Convert dataclass to dictionary for internal storage
        data = {
            'trajectory_id': message.trajectory_id,
            'submap_index': message.submap_index,
            'file_path': message.file_path,
            'resolution': message.resolution,
            'width': message.width,
            'height': message.height
        }
        logger.info(f"[TRACK] Converted dataclass to dictionary for submap {message.trajectory_id}_{message.submap_index}")
        logger.info(f"[TRACK] Dictionary data: {data}")
        self._process_submap_data(data)
    
    def _process_submap_data(self, data: Dict[str, Any]) -> None:
        """Process submap data received from the ROS node.
        
        Args:
            data: Dictionary with submap information
        """
        trajectory_id = data['trajectory_id']
        submap_index = data['submap_index']
        submap_id = (trajectory_id, submap_index)
        
        logger.info(f"[TRACK] Processing submap data for {trajectory_id}_{submap_index}")
        
        # Check if this is an update to an existing submap
        is_update = submap_id in self.submaps
        logger.info(f"[TRACK] Is this an update to existing submap? {is_update}")
        
        # Store submap data
        self.submaps[submap_id] = data
        logger.info(f"[TRACK] Stored submap data in self.submaps dictionary")
        
        # Load the image
        try:
            file_path = data['file_path']
            logger.info(f"[TRACK] Loading image from {file_path}")
            
            if not os.path.exists(file_path):
                logger.error(f"[TRACK] Image file does not exist: {file_path}")
                return
                
            # Load the image
            image = Image.open(file_path)
            logger.info(f"[TRACK] Successfully loaded image with size {image.size}")
            
            # Store the PIL image for later use
            data['pil_image'] = image
            logger.info(f"[TRACK] Stored PIL image in submap data")
            
            # Notify UI
            if is_update:
                logger.info(f"[TRACK] Notifying UI of submap update for {trajectory_id}_{submap_index}")
                self._notify(CallbackType.ON_SUBMAP_UPDATED, submap_id, data)
                status_text = f"Updated submap {trajectory_id}_{submap_index}"
            else:
                logger.info(f"[TRACK] Notifying UI of new submap for {trajectory_id}_{submap_index}")
                self._notify(CallbackType.ON_SUBMAP_ADDED, submap_id, data)
                status_text = f"Added submap {trajectory_id}_{submap_index}"
                
                # Auto-select first submap if none is selected
                if len(self.submaps) == 1:
                    logger.info(f"[TRACK] Auto-selecting first submap {trajectory_id}_{submap_index}")
                    self.select_submap(submap_id)
            
            self._notify(CallbackType.ON_STATUS_CHANGED, status_text)
            logger.info(f"[TRACK] Updated status: {status_text}")
                
        except Exception as e:
            error_msg = f"Error loading image for submap {trajectory_id}_{submap_index}: {e}"
            logger.error(f"[TRACK] {error_msg}", exc_info=True)
    
    def select_submap(self, submap_id: Tuple[int, int]) -> None:
        """Select a submap for display.
        
        Args:
            submap_id: Tuple of (trajectory_id, submap_index)
        """
        if submap_id not in self.submaps:
            logger.warning(f"Cannot select submap {submap_id}: not found")
            return
            
        self.selected_submap_id = submap_id
        self._notify(CallbackType.ON_SUBMAP_SELECTED, submap_id, self.submaps[submap_id])
        
        trajectory_id, submap_index = submap_id
        logger.info(f"Selected submap {trajectory_id}_{submap_index}")
    
    def get_submap_ids(self) -> List[Tuple[int, int]]:
        """Get a list of all submap IDs.
        
        Returns:
            List of (trajectory_id, submap_index) tuples
        """
        return sorted(self.submaps.keys())
    
    def get_submap_display_text(self, submap_id: Tuple[int, int]) -> str:
        """Get display text for a submap.
        
        Args:
            submap_id: Tuple of (trajectory_id, submap_index)
            
        Returns:
            Display text for the submap
        """
        trajectory_id, submap_index = submap_id
        return f"Trajectory {trajectory_id}, Submap {submap_index}"
    
    def get_submap_info_text(self, submap_id: Tuple[int, int]) -> str:
        """Get detailed info text for a submap.
        
        Args:
            submap_id: Tuple of (trajectory_id, submap_index)
            
        Returns:
            Detailed info text for the submap
        """
        if submap_id not in self.submaps:
            return "No submap selected"
            
        submap_data = self.submaps[submap_id]
        trajectory_id, submap_index = submap_id
        resolution = submap_data['resolution']
        width = submap_data['width']
        height = submap_data['height']
        
        info_text = f"Trajectory {trajectory_id}, Submap {submap_index} | "
        info_text += f"Resolution: {resolution:.3f}m/pixel | "
        info_text += f"Size: {width}x{height} pixels | "
        info_text += f"Physical size: {width*resolution:.2f}m x {height*resolution:.2f}m"
        
        return info_text
    
    def request_submap_reload(self, submap_id: Tuple[int, int]) -> bool:
        """Request a reload of a submap from the ROS node.
        
        Args:
            submap_id: Tuple of (trajectory_id, submap_index)
        
        Returns:
            True if the request was sent, False otherwise
        """
        if submap_id not in self.submaps:
            logger.warning(f"Cannot reload submap {submap_id}: not found")
            self._notify(CallbackType.ON_STATUS_CHANGED, f"Cannot reload: submap {submap_id} not found")
            return False
            
        trajectory_id, submap_index = submap_id
        
        # Put a request in the queue for the node to handle using the new dataclass
        message = RequestSubmapMessage(
            trajectory_id=trajectory_id,
            submap_index=submap_index
        )
        self.update_queue.put((MessageType.REQUEST_SUBMAP, message))
        
        status_text = f"Requesting fresh submap for {trajectory_id}_{submap_index}..."
        self._notify(CallbackType.ON_STATUS_CHANGED, status_text)
        logger.info(f"Requested fresh submap for {trajectory_id}_{submap_index}")
        return True

    def _ensure_all_submaps_loaded(self, submap_ids: List[Tuple[int, int]]) -> None:
        """Ensure all submaps in the list are loaded.
        
        This method checks if we have all the submaps in the list and requests
        any that are missing.
        
        Args:
            submap_ids: List of (trajectory_id, submap_index) tuples
        """
        if not submap_ids:
            logger.info("[TRACK] No submap IDs provided to _ensure_all_submaps_loaded")
            return
        
        logger.info(f"[TRACK] Ensuring all {len(submap_ids)} submaps are loaded")
        logger.info(f"[TRACK] Currently loaded submaps: {list(self.submaps.keys())}")
        
        # Find submaps that we don't have yet
        missing_submaps = [
            submap_id for submap_id in submap_ids
            if submap_id not in self.submaps
        ]
        
        logger.info(f"[TRACK] Found {len(missing_submaps)} missing submaps: {missing_submaps}")
        
        # Request missing submaps
        for submap_id in missing_submaps:
            trajectory_id, submap_index = submap_id
            logger.info(f"[TRACK] Requesting missing submap {trajectory_id}_{submap_index}")
            
            # Create request message
            message = RequestSubmapMessage(
                trajectory_id=trajectory_id,
                submap_index=submap_index
            )
            self.update_queue.put((MessageType.REQUEST_SUBMAP, message))
            logger.info(f"[TRACK] Added request for submap {trajectory_id}_{submap_index} to update queue") 